#include "camera_pose_initializer/camera_pose_initializer.hpp"

#include <ll2_decomposer/from_bin_msg.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcdless
{
CameraPoseInitializer::CameraPoseInitializer()
: Node("camera_pose_initializer"),
  cov_xx_yy_{declare_parameter("cov_xx_yy", std::vector<double>{4.0, 0.25}).data()}
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  const rclcpp::QoS map_qos = rclcpp::QoS(1).transient_local().reliable();
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  marker_module_ = std::make_unique<initializer::MarkerModule>(this);
  projector_module_ = std::make_unique<initializer::ProjectorModule>(this);

  // Publisher
  pub_initialpose_ = create_publisher<PoseCovStamped>("rectified/initialpose", 10);

  // Subscriber
  auto on_map = std::bind(&CameraPoseInitializer::on_map, this, _1);
  auto on_image = [this](Image::ConstSharedPtr msg) -> void { latest_image_msg_ = msg; };
  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, on_map);
  sub_image_ = create_subscription<Image>("/sensing/camera/undistorted/image_raw", 10, on_image);

  // Client
  ground_client_ = create_client<Ground>(
    "/localization/map/ground", rmw_qos_profile_services_default, service_callback_group_);
  semseg_client_ = create_client<SemsegSrv>(
    "/localization/semseg_srv", rmw_qos_profile_services_default, service_callback_group_);

  // Server
  auto on_service = std::bind(&CameraPoseInitializer::on_service, this, _1, _2);
  align_server_ = create_service<RequestPoseAlignment>("pcdless_align_srv", on_service);

  using namespace std::chrono_literals;
  while (!ground_client_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO_STREAM(get_logger(), "Waiting for " << ground_client_->get_service_name());
  }
  while (!semseg_client_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO_STREAM(get_logger(), "Waiting for " << semseg_client_->get_service_name());
  }
}

cv::Mat bitwise_and_3ch(const cv::Mat src1, const cv::Mat src2)
{
  std::vector<cv::Mat> src1_array;
  std::vector<cv::Mat> src2_array;
  cv::split(src1, src1_array);
  cv::split(src2, src2_array);
  std::vector<cv::Mat> dst_array;
  for (int i = 0; i < 3; i++) {
    cv::Mat dst;
    cv::bitwise_and(src1_array.at(i), src2_array.at(i), dst);
    dst_array.push_back(dst);
  }
  cv::Mat merged;
  cv::merge(dst_array, merged);
  return merged;
}

int count_nonzero(cv::Mat image_3ch)
{
  std::vector<cv::Mat> images;
  cv::split(image_3ch, images);
  int count = 0;
  for (int i = 0; i < 3; i++) {
    count += cv::countNonZero(images.at(i));
  }
  return count;
}

bool CameraPoseInitializer::estimate_pose(
  const Eigen::Vector3f & position, Eigen::Vector3f & tangent)
{
  if (!projector_module_->define_project_func()) {
    return false;
  }
  if (!lane_image_) {
    RCLCPP_WARN_STREAM(get_logger(), "vector map is not ready ");
    return false;
  }

  Image semseg_image;
  {
    // TODO: check time stamp
    if (!latest_image_msg_.has_value()) {
      RCLCPP_WARN_STREAM(get_logger(), "image is not ready");
      return false;
    }

    auto request = std::make_shared<SemsegSrv::Request>();
    request->src_image = *latest_image_msg_.value();
    auto result_future = semseg_client_->async_send_request(request);
    using namespace std::chrono_literals;
    std::future_status status = result_future.wait_for(1000ms);
    if (status == std::future_status::ready) {
      semseg_image = result_future.get()->dst_image;
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "semseg service exited unexpectedly");
      return false;
    }
  }

  cv::Mat projected_image = projector_module_->project_image(semseg_image);
  cv::Mat vectormap_image = lane_image_->create_vectormap_image(position);

  std::vector<int> scores;
  std::vector<float> angles;  // rad

  for (int i = 0; i < 30; i++) {
    cv::Mat rot = cv::getRotationMatrix2D(cv::Point2f(400, 400), (-i / 30.0 * 360), 1);
    cv::Mat rotated_image;
    cv::warpAffine(vectormap_image, rotated_image, rot, vectormap_image.size());

    cv::Mat dst = bitwise_and_3ch(rotated_image, projected_image);
    cv::Mat show_image;
    cv::hconcat(std::vector<cv::Mat>{projected_image, rotated_image, dst}, show_image);

    int count = count_nonzero(dst);
    cv::imshow("and operator", show_image);
    cv::waitKey(50);

    scores.push_back(count);
    angles.push_back(i / 15.f * M_PI);
  }

  {
    size_t max_index =
      std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));
    float max_angle = angles.at(max_index);
    tangent.x() = std::cos(max_angle);
    tangent.y() = std::sin(max_angle);
    tangent.z() = 0;
  }

  marker_module_->publish_marker(scores, angles, position);

  return true;
}

void CameraPoseInitializer::on_map(const HADMapBin & msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "subscribed binary vector map");
  lanelet::LaneletMapPtr lanelet_map = ll2_decomposer::from_bin_msg(msg);
  lane_image_ = std::make_unique<LaneImage>(lanelet_map);
}

void CameraPoseInitializer::on_service(
  const RequestPoseAlignment::Request::SharedPtr request,
  RequestPoseAlignment::Response::SharedPtr response)
{
  RCLCPP_INFO_STREAM(get_logger(), "CameraPoseInitializer on_service");
  response->success = false;

  const auto query_pos = request->pose_with_covariance.pose.pose.position;
  auto ground_request = std::make_shared<Ground::Request>();
  ground_request->point.x = query_pos.x;
  ground_request->point.y = query_pos.y;

  using namespace std::chrono_literals;
  auto result_future = ground_client_->async_send_request(ground_request);
  std::future_status status = result_future.wait_for(1000ms);
  if (status == std::future_status::ready) {
    RCLCPP_ERROR_STREAM(get_logger(), "get height service exited ");
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "get height service exited unexpectedly");
    return;
  }

  // Retrieve 3d position
  const auto position = result_future.get()->pose.position;
  Eigen::Vector3f pos_vec3f;
  pos_vec3f << position.x, position.y, position.z;
  RCLCPP_INFO_STREAM(get_logger(), "get initial position " << pos_vec3f.transpose());

  // Estimate orientation
  Eigen::Vector3f tangent;
  if (estimate_pose(pos_vec3f, tangent)) {
    response->success = true;
    response->pose_with_covariance = create_rectified_initial_pose(pos_vec3f, tangent);
  }
}

CameraPoseInitializer::PoseCovStamped CameraPoseInitializer::create_rectified_initial_pose(
  const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent)
{
  PoseCovStamped msg;
  msg.header.frame_id = "map";
  msg.pose.pose.position.x = pos.x();
  msg.pose.pose.position.y = pos.y();
  msg.pose.pose.position.z = pos.z();

  float theta = std::atan2(tangent.y(), tangent.x());

  msg.pose.pose.orientation.w = std::cos(theta / 2);
  msg.pose.pose.orientation.x = 0;
  msg.pose.pose.orientation.y = 0;
  msg.pose.pose.orientation.z = std::sin(theta / 2);

  Eigen::Matrix2f cov;
  cov << cov_xx_yy_(0), 0, 0, cov_xx_yy_(1);
  Eigen::Rotation2D r(theta);
  cov = r * cov * r.inverse();

  msg.pose.covariance.at(6 * 0 + 0) = cov(0, 0);
  msg.pose.covariance.at(6 * 0 + 1) = cov(0, 1);
  msg.pose.covariance.at(6 * 1 + 0) = cov(1, 0);
  msg.pose.covariance.at(6 * 1 + 1) = cov(1, 1);
  msg.pose.covariance.at(6 * 5 + 5) = 0.0076;  // 0.0076 = (5deg)^2
  return msg;
}

}  // namespace pcdless