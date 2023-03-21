#include "camera_pose_initializer/camera_pose_initializer.hpp"

#include <ll2_decomposer/from_bin_msg.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless
{
CameraPoseInitializer::CameraPoseInitializer()
: Node("camera_pose_initializer"),
  cov_xx_yy_{declare_parameter("cov_xx_yy", std::vector<double>{4.0, 0.25}).data()}
{
  using std::placeholders::_1;
  const rclcpp::QoS map_qos = rclcpp::QoS(1).transient_local().reliable();
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Publisher
  pub_initialpose_ = create_publisher<PoseCovStamped>("rectified/initialpose", 10);
  pub_marker_ = create_publisher<Marker>("init/marker", 10);

  // Subscriber
  auto on_initialpose = std::bind(&CameraPoseInitializer::on_initial_pose, this, _1);
  sub_initialpose_ =
    create_subscription<PoseCovStamped>("initialpose", 10, std::move(on_initialpose));
  auto on_map = std::bind(&CameraPoseInitializer::on_map, this, _1);
  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, on_map);

  // Client
  ground_cli_ = create_client<Ground>(
    "/localization/map/ground", rmw_qos_profile_services_default, service_callback_group_);

  using namespace std::chrono_literals;
  while (!ground_cli_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO_STREAM(get_logger(), "Waiting for service...");
  }
}

void CameraPoseInitializer::on_initial_pose(const PoseCovStamped & initialpose)
{
  const auto query_pos = initialpose.pose.pose.position;
  auto req = std::make_shared<Ground::Request>();
  req->point.x = query_pos.x;
  req->point.y = query_pos.y;

  using namespace std::chrono_literals;
  auto result_future = ground_cli_->async_send_request(req);
  std::future_status status = result_future.wait_for(1000ms);
  if (status == std::future_status::ready) {
    RCLCPP_INFO_STREAM(get_logger(), "get ground height service exited successfully");
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "get height service exited unexpectedly");
    return;
  }

  //

  const auto position = result_future.get()->pose.position;
  Eigen::Vector3f pos_vec3f;
  pos_vec3f << position.x, position.y, position.z;

  std::cout << "get initial position" << pos_vec3f.transpose() << std::endl;

  auto orientation = initialpose.pose.pose.orientation;
  float theta = 2 * std::atan2(orientation.z, orientation.w);
  Eigen::Vector3f tangent;
  tangent << std::cos(theta), std::sin(theta), 0;

  publish_rectified_initial_pose(pos_vec3f, tangent, initialpose);
}

void CameraPoseInitializer::on_map(const HADMapBin & msg)
{
  lanelet::LaneletMapPtr lanelet_map = ll2_decomposer::from_bin_msg(msg);

  const std::set<std::string> visible_labels = {
    "zebra_marking",      "virtual",   "line_thin", "line_thick",
    "pedestrian_marking", "stop_line", "curbstone"};

  for (lanelet::LineString3d & line : lanelet_map->lineStringLayer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;

    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;
    // TODO:
  }
}

void CameraPoseInitializer::publish_rectified_initial_pose(
  const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent,
  const PoseCovStamped & raw_initialpose)
{
  PoseCovStamped msg = raw_initialpose;
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

  pub_initialpose_->publish(msg);
}

}  // namespace pcdless