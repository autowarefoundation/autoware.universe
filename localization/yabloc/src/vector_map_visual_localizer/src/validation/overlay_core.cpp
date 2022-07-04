#include "common/util.hpp"
#include "map/ll2_util.hpp"
#include "validation/overlay.hpp"

#include <eigen3/Eigen/StdVector>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <sophus/geometry.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

namespace validation
{
void Overlay::ll2Callback(const PointCloud2 & msg) { pcl::fromROSMsg(msg, ll2_cloud_); }

Overlay::Overlay() : Node("overlay"), pose_buffer_{40}, tf_subscriber_(get_clock())
{
  using std::placeholders::_1;
  // Subscriber
  {
    auto cb_info = std::bind(&Overlay::infoCallback, this, _1);
    auto cb_image = std::bind(&Overlay::imageCallback, this, _1);
    auto cb_particle = std::bind(&Overlay::poseCallback, this, _1);
    auto cb_ll2 = std::bind(&Overlay::ll2Callback, this, _1);
    auto cb_lsd = std::bind(&Overlay::lsdCallback, this, _1);

    sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/sensing/camera/traffic_light/camera_info", 10, cb_info);
    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
      "/sensing/camera/traffic_light/image_raw/compressed", 10, cb_image);
    sub_pose_ = create_subscription<PoseStamped>("/particle_pose", 10, cb_particle);
    sub_ll2_ = create_subscription<PointCloud2>("/ll2_cloud", 10, cb_ll2);
    sub_lsd_ = create_subscription<PointCloud2>("/lsd_cloud", 10, cb_lsd);
    sub_sign_board_ = create_subscription<PointCloud2>(
      "/sign_board", 10,
      [this](const PointCloud2 & msg) -> void { pcl::fromROSMsg(msg, sign_board_); });
  }
  // Publisher
  {
    pub_vis_ = create_publisher<Marker>("/marker", 10);
    pub_image_ = create_publisher<sensor_msgs::msg::Image>("/overlay_image", 10);
  }
}

void Overlay::infoCallback(const sensor_msgs::msg::CameraInfo & msg)
{
  info_ = msg;
  camera_extrinsic_ = tf_subscriber_(info_->header.frame_id, "base_link");
}

void Overlay::imageCallback(const sensor_msgs::msg::Image & msg)
{
  cv::Mat image = util::decompress2CvMat(msg);
  const rclcpp::Time stamp = msg.header.stamp;

  // Search synchronized pose
  float min_dt = std::numeric_limits<float>::max();
  geometry_msgs::msg::PoseStamped synched_pose;
  for (auto pose : pose_buffer_) {
    auto dt = (rclcpp::Time(pose.header.stamp) - stamp);
    auto abs_dt = std::abs(dt.seconds());
    if (abs_dt < min_dt) {
      min_dt = abs_dt;
      synched_pose = pose;
    }
  }
  if (min_dt > 0.1) return;
  auto latest_pose_stamp = rclcpp::Time(pose_buffer_.back().header.stamp);
  RCLCPP_INFO_STREAM(
    get_logger(), "dt: " << min_dt << " image:" << stamp.nanoseconds()
                         << " latest_pose:" << latest_pose_stamp.nanoseconds());

  drawOverlay(image, synched_pose.pose, stamp);
}

void Overlay::lsdCallback(const PointCloud2 & msg)
{
  const rclcpp::Time stamp = msg.header.stamp;

  // Search synchronized pose
  float min_dt = std::numeric_limits<float>::max();
  geometry_msgs::msg::PoseStamped synched_pose;
  for (auto pose : pose_buffer_) {
    auto dt = (rclcpp::Time(pose.header.stamp) - stamp);
    auto abs_dt = std::abs(dt.seconds());
    if (abs_dt < min_dt) {
      min_dt = abs_dt;
      synched_pose = pose;
    }
  }
  if (min_dt > 0.1) return;
  auto latest_pose_stamp = rclcpp::Time(pose_buffer_.back().header.stamp);

  LineSegments lsd_cloud;
  pcl::fromROSMsg(msg, lsd_cloud);
  makeVisMarker(lsd_cloud, synched_pose.pose, stamp);
}

void Overlay::drawOverlay(const cv::Mat & image, const Pose & pose, const rclcpp::Time & stamp)
{
  if (ll2_cloud_.empty()) return;

  Eigen::Affine3f transform = util::pose2Affine(pose);

  cv::Mat overlayed_image = cv::Mat::zeros(image.size(), CV_8UC3);
  drawOverlaySignBoard(overlayed_image, pose, stamp);

  Eigen::Matrix3f K =
    Eigen::Map<Eigen::Matrix<double, 3, 3> >(info_->k.data()).cast<float>().transpose();
  Eigen::Affine3f T = camera_extrinsic_.value();

  auto project = [K, T, transform](const Eigen::Vector3f & xyz) -> std::optional<cv::Point2i> {
    Eigen::Vector3f from_camera = K * T.inverse() * transform.inverse() * xyz;
    if (from_camera.z() < 1e-3f) return std::nullopt;
    Eigen::Vector3f uv1 = from_camera /= from_camera.z();
    return cv::Point2i(uv1.x(), uv1.y());
  };

  LineSegments near_segments = extractNaerLineSegments(pose);
  for (const pcl::PointNormal & pn : near_segments) {
    auto p1 = project(pn.getArray3fMap()), p2 = project(pn.getNormalVector3fMap());
    if (!p1.has_value() || !p2.has_value()) continue;
    cv::line(overlayed_image, p1.value(), p2.value(), cv::Scalar(0, 255, 255), 2);
  }

  cv::Mat show_image;
  cv::addWeighted(image, 0.8, overlayed_image, 0.8, 1, show_image);
  util::publishImage(*pub_image_, show_image, stamp);
}

void Overlay::poseCallback(const geometry_msgs::msg::PoseStamped & msg)
{
  pose_buffer_.push_back(msg);
}

void Overlay::makeVisMarker(const LineSegments & ls, const Pose & pose, const rclcpp::Time & stamp)
{
  Marker marker;
  marker.type = Marker::LINE_LIST;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.pose = pose;
  marker.scale.x = 0.1;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.7f;

  for (const auto pn : ls) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = pn.x;
    p1.y = pn.y;
    p1.z = pn.z;
    p2.x = pn.normal_x;
    p2.y = pn.normal_y;
    p2.z = pn.normal_z;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }
  pub_vis_->publish(marker);
}

Overlay::LineSegments Overlay::extractNaerLineSegments(const Pose & pose)
{
  Eigen::Vector3f pose_vector;
  pose_vector << pose.position.x, pose.position.y, pose.position.z;

  // Compute distance between pose and linesegment of linestring
  auto checkIntersection = [this, pose_vector](const pcl::PointNormal & pn) -> bool {
    const float max_range = 40;

    const Eigen::Vector3f from = pn.getVector3fMap() - pose_vector;
    const Eigen::Vector3f to = pn.getNormalVector3fMap() - pose_vector;
    Eigen::Vector3f dir = to - from;
    float inner = from.dot(dir);
    if (std::abs(inner) < 1e-3f) {
      return from.norm() < 1.42 * max_range;
    }

    float mu = std::clamp(dir.squaredNorm() / inner, 0.f, 1.0f);
    Eigen::Vector3f nearest = from + dir * mu;
    return nearest.norm() < 2 * 1.42 * max_range;
  };

  LineSegments near_linestring;
  for (const pcl::PointNormal & pn : ll2_cloud_) {
    if (checkIntersection(pn)) {
      near_linestring.push_back(pn);
    }
  }
  return near_linestring;
}

void Overlay::drawOverlaySignBoard(cv::Mat & image, const Pose & pose, const rclcpp::Time & stamp)
{
  Eigen::Matrix3f K =
    Eigen::Map<Eigen::Matrix<double, 3, 3> >(info_->k.data()).cast<float>().transpose();
  Eigen::Affine3f T = camera_extrinsic_.value();

  Eigen::Affine3f transform = util::pose2Affine(pose);
  auto project = [K, T, transform](const Eigen::Vector3f & xyz) -> std::optional<cv::Point2i> {
    Eigen::Vector3f from_camera = K * T.inverse() * transform.inverse() * xyz;
    if (from_camera.z() < 1e-3f) return std::nullopt;
    Eigen::Vector3f uv1 = from_camera /= from_camera.z();
    return cv::Point2i(uv1.x(), uv1.y());
  };

  for (const pcl::PointNormal & pn : sign_board_) {
    auto p1 = project(pn.getArray3fMap()), p2 = project(pn.getNormalVector3fMap());
    if (!p1.has_value() || !p2.has_value()) continue;
    cv::line(image, p1.value(), p2.value(), cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace validation