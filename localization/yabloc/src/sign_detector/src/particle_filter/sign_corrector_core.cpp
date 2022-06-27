#include "common/util.hpp"
#include "particle_filter/sign_corrector.hpp"
#include "sign_detector/ll2_util.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

namespace particle_filter
{
SignCorrector::SignCorrector()
: AbstCorrector("sign_corrector"), blur_size_(declare_parameter<int>("blur_size", 3))
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  using std::placeholders::_1;
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();
  auto cb_map = std::bind(&SignCorrector::mapCallback, this, _1);
  auto cb_pose = std::bind(&SignCorrector::poseCallback, this, _1);
  auto cb_info = std::bind(&SignCorrector::infoCallback, this, _1);
  auto cb_image = std::bind(&SignCorrector::imageCallback, this, _1);
  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, cb_map);
  sub_pose_ = create_subscription<PoseStamped>("/particle_pose", 10, cb_pose);
  sub_info_ = create_subscription<CameraInfo>("/camera_info", 10, cb_info);
  sub_image_ = create_subscription<Image>("/image", 10, cb_image);

  const rclcpp::QoS latch_qos = rclcpp::QoS{1}.transient_local();
  pub_image_ = create_publisher<Image>("/sign_image", 10);
  pub_marker_ = create_publisher<MarkerArray>("/sign_marker", latch_qos);
}

void SignCorrector::mapCallback(const HADMapBin & msg)
{
  lanelet_map_ = fromBinMsg(msg);
  publishSignMarker();
}

void SignCorrector::poseCallback(const PoseStamped & msg) { extractNearSign(msg); }

void SignCorrector::extractNearSign(const PoseStamped & pose_stamped)
{
  if (lanelet_map_ == nullptr) return;

  linestrings_ = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

  const rclcpp::Time stamp = pose_stamped.header.stamp;
  auto pos = pose_stamped.pose.position;
  Eigen::Vector3f position;
  position << pos.x, pos.y, pos.z;

  float w = pose_stamped.pose.orientation.w;
  float z = pose_stamped.pose.orientation.z;
  Eigen::Matrix3f rotation =
    Eigen::AngleAxisf(2.f * std::atan2(z, w), Eigen::Vector3f::UnitZ()).matrix();

  const std::unordered_set<std::string> visible_labels = {"sign-board"};

  for (lanelet::LineString3d & line : lanelet_map_->lineStringLayer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;

    std::optional<lanelet::ConstPoint3d> from = std::nullopt;
    for (const lanelet::ConstPoint3d p : line) {
      Eigen::Vector3f p_vec(p.x(), p.y(), p.z());

      Eigen::Vector3f relative = rotation.transpose() * (p_vec - position);
      if (
        (relative.x() > 0) & (relative.x() < 50) & (std::abs(relative.y()) < 10) &
        from.has_value()) {
        pcl::PointNormal pn;
        pn.x = from->x();
        pn.y = from->y();
        pn.z = from->z();
        pn.normal_x = p.x();
        pn.normal_y = p.y();
        pn.normal_z = p.z();
        linestrings_->push_back(pn);
      }
      from = p;
    }
  }
}

void SignCorrector::infoCallback(const CameraInfo & msg)
{
  camera_info_ = msg;
  listenExtrinsicTf(msg.header.frame_id);
}

void SignCorrector::execute(const rclcpp::Time & stamp, cv::Mat image)
{
  if (linestrings_ == nullptr) return;
  if (!camera_info_.has_value()) return;
  if (!camera_extrinsic_.has_value()) return;

  std::optional<ParticleArray> opt_array = this->getSyncronizedParticleArray(stamp);
  if (!opt_array.has_value()) return;
  auto dt = (stamp - opt_array->header.stamp);
  if (std::abs(dt.seconds()) > 0.1)
    RCLCPP_WARN_STREAM(
      get_logger(), "Timestamp gap between image and particles is LARGE " << dt.seconds());

  RCLCPP_WARN_STREAM(get_logger(), "There are visible sign-board around here");

  Eigen::Matrix3f K =
    Eigen::Map<Eigen::Matrix<double, 3, 3>>(camera_info_->k.data()).cast<float>().transpose();
  Eigen::Affine3f T = camera_extrinsic_.value();

  cv::Mat board_image = cv::Mat::zeros(image.size(), CV_8UC3);

  for (const Particle & p : opt_array->particles) {
    Eigen::Affine3f transform = util::pose2Affine(p.pose);
    auto project = [K, T, transform](const Eigen::Vector3f & xyz) -> std::optional<cv::Point2i> {
      Eigen::Vector3f from_camera = K * T.inverse() * transform.inverse() * xyz;
      if (from_camera.z() < 1e-3f) return std::nullopt;
      Eigen::Vector3f uv1 = from_camera /= from_camera.z();
      return cv::Point2i(uv1.x(), uv1.y());
    };

    for (const auto pn : *linestrings_) {
      auto from_opt = project(pn.getVector3fMap());
      auto to_opt = project(pn.getNormalVector3fMap());
      if (from_opt.has_value() & to_opt.has_value())
        cv::line(board_image, from_opt.value(), to_opt.value(), cv::Scalar(0, 255, 255), 1);
    }
  }
  cv::addWeighted(image, 0.8, board_image, 0.8, 1, board_image);
  util::publishImage(*pub_image_, board_image, stamp);
}

void SignCorrector::imageCallback(const Image & msg)
{
  const rclcpp::Time stamp = msg.header.stamp;
  cv::Mat image = util::decompress2CvMat(msg);

  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  cv::Size kernel_size(2 * blur_size_ + 1, 2 * blur_size_ + 1);
  cv::GaussianBlur(gray_image, gray_image, kernel_size, 0, 0);
  cv::Mat edge_image;
  cv::Laplacian(gray_image, edge_image, CV_16SC1, 5);

  cv::Mat rgb_edge_image;
  cv::convertScaleAbs(edge_image, rgb_edge_image, 1.0f);
  cv::applyColorMap(rgb_edge_image, rgb_edge_image, cv::COLORMAP_JET);

  execute(stamp, rgb_edge_image);
}

void SignCorrector::listenExtrinsicTf(const std::string & frame_id)
{
  try {
    geometry_msgs::msg::TransformStamped ts =
      tf_buffer_->lookupTransform("base_link", frame_id, tf2::TimePointZero);
    Eigen::Vector3f p;
    p.x() = ts.transform.translation.x;
    p.y() = ts.transform.translation.y;
    p.z() = ts.transform.translation.z;

    Eigen::Quaternionf q;
    q.w() = ts.transform.rotation.w;
    q.x() = ts.transform.rotation.x;
    q.y() = ts.transform.rotation.y;
    q.z() = ts.transform.rotation.z;

    camera_extrinsic_ = Eigen::Affine3f::Identity();
    camera_extrinsic_->translation() = p;
    camera_extrinsic_->matrix().topLeftCorner(3, 3) = q.toRotationMatrix();
  } catch (tf2::TransformException & ex) {
  }
}

void SignCorrector::publishSignMarker()
{
  const std::unordered_set<std::string> visible_labels = {"sign-board"};

  lanelet::LineStrings3d sign_boards;
  for (const lanelet::LineString3d & line : lanelet_map_->lineStringLayer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;
    sign_boards.push_back(line);
  }

  MarkerArray marker_array;
  int id = 0;
  for (const lanelet::LineString3d & sign_boards : sign_boards) {
    Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = get_clock()->now();
    marker.type = Marker::LINE_STRIP;
    marker.color = util::color(1.0f, 1.0f, 1.0f, 1.0f);
    marker.scale.x = 0.1;
    marker.id = id++;

    for (const lanelet::ConstPoint3d & p : sign_boards) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x();
      gp.y = p.y();
      gp.z = p.z();
      marker.points.push_back(gp);
    }
    marker_array.markers.push_back(marker);
  }

  pub_marker_->publish(marker_array);
}
}  // namespace particle_filter