#include "vmvl_validation/overlay.hpp"

#include <eigen3/Eigen/StdVector>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <sophus/geometry.hpp>
#include <vml_common/cv_decompress.hpp>
#include <vml_common/pose_conversions.hpp>
#include <vml_common/pub_sub.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace vmvl_validation
{
Overlay::Overlay() : Node("overlay"), tf_subscriber_(get_clock()), pose_buffer_{40}
{
  using std::placeholders::_1;

  // Subscriber
  auto cb_info = std::bind(&Overlay::infoCallback, this, _1);
  auto cb_image = std::bind(&Overlay::imageCallback, this, _1);
  auto cb_lsd = std::bind(&Overlay::lsdCallback, this, _1);
  auto cb_pose = [this](const PoseStamped & msg) -> void { pose_buffer_.push_back(msg); };
  auto cb_ground = [this](const Float32Array & msg) -> void { ground_plane_.set(msg); };

  sub_ground_plane_ = create_subscription<Float32Array>("ground", 10, cb_ground);
  sub_image_ = create_subscription<Image>("src_image", 10, cb_image);
  sub_pose_ = create_subscription<PoseStamped>("particle_pose", 10, cb_pose);
  sub_lsd_ = create_subscription<PointCloud2>("projected_lsd_cloud", 10, cb_lsd);
  sub_info_ = create_subscription<CameraInfo>("src_info", 10, cb_info);
  sub_sign_board_ = create_subscription<PointCloud2>(
    "ll2_sign_board", 10,
    [this](const PointCloud2 & msg) -> void { pcl::fromROSMsg(msg, sign_board_); });
  sub_ll2_ = create_subscription<PointCloud2>(
    "ll2_road_marking", 10,
    [this](const PointCloud2 & msg) -> void { pcl::fromROSMsg(msg, ll2_cloud_); });

  // Publisher
  pub_vis_ = create_publisher<Marker>("projected_marker", 10);
  pub_image_ = create_publisher<sensor_msgs::msg::Image>("overlay_image", 10);
}

void Overlay::infoCallback(const CameraInfo & msg)
{
  info_ = msg;
  camera_extrinsic_ = tf_subscriber_(info_->header.frame_id, "base_link");
}

void Overlay::imageCallback(const sensor_msgs::msg::Image & msg)
{
  cv::Mat image = vml_common::decompress2CvMat(msg);
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

  cv::Mat overlayed_image = cv::Mat::zeros(image.size(), CV_8UC3);
  drawOverlayLineSegments(overlayed_image, pose, extractNaerLineSegments(pose, ll2_cloud_));
  drawOverlayLineSegments(overlayed_image, pose, extractNaerLineSegments(pose, sign_board_));

  cv::Mat show_image;
  cv::addWeighted(image, 0.8, overlayed_image, 0.8, 1, show_image);
  vml_common::publishImage(*pub_image_, show_image, stamp);
}

void Overlay::drawOverlayLineSegments(
  cv::Mat & image, const Pose & pose, const LineSegments & near_segments)
{
  Eigen::Matrix3f K =
    Eigen::Map<Eigen::Matrix<double, 3, 3> >(info_->k.data()).cast<float>().transpose();
  Eigen::Affine3f T = camera_extrinsic_.value();

  Eigen::Affine3f transform = ground_plane_.alignWithSlope(vml_common::pose2Affine(pose));

  auto projectLineSegment =
    [K, T, transform](
      const Eigen::Vector3f & p1,
      const Eigen::Vector3f & p2) -> std::tuple<bool, cv::Point2i, cv::Point2i> {
    Eigen::Vector3f from_camera1 = K * T.inverse() * transform.inverse() * p1;
    Eigen::Vector3f from_camera2 = K * T.inverse() * transform.inverse() * p2;
    bool p1_is_visible = from_camera1.z() > 1e-3f;
    bool p2_is_visible = from_camera2.z() > 1e-3f;
    if ((!p1_is_visible) && (!p2_is_visible)) return {false, cv::Point2i{}, cv::Point2i{}};

    Eigen::Vector3f uv1, uv2;
    if (p1_is_visible) uv1 = from_camera1 / from_camera1.z();
    if (p2_is_visible) uv2 = from_camera2 / from_camera2.z();

    if ((p1_is_visible) && (p2_is_visible))
      return {true, cv::Point2i(uv1.x(), uv1.y()), cv::Point2i(uv2.x(), uv2.y())};

    Eigen::Vector3f tangent = from_camera2 - from_camera1;
    float mu = (1e-3f - from_camera1.z()) / (tangent.z());
    if (!p1_is_visible) {
      from_camera1 = from_camera1 + mu * tangent;
      uv1 = from_camera1 / from_camera1.z();
    }
    if (!p2_is_visible) {
      from_camera2 = from_camera1 + mu * tangent;
      uv2 = from_camera2 / from_camera2.z();
    }
    return {true, cv::Point2i(uv1.x(), uv1.y()), cv::Point2i(uv2.x(), uv2.y())};
  };

  for (const pcl::PointNormal & pn : near_segments) {
    auto [success, u1, u2] = projectLineSegment(pn.getVector3fMap(), pn.getNormalVector3fMap());
    if (success) cv::line(image, u1, u2, cv::Scalar(0, 255, 255), 2);
  }
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

Overlay::LineSegments Overlay::extractNaerLineSegments(
  const Pose & pose, const LineSegments & linesegments)
{
  Eigen::Vector3f pose_vector;
  pose_vector << pose.position.x, pose.position.y, pose.position.z;

  // Compute distance between pose and linesegment of linestring
  auto checkIntersection = [this, pose_vector](const pcl::PointNormal & pn) -> bool {
    const float max_range = 80;

    const Eigen::Vector3f from = pn.getVector3fMap() - pose_vector;
    const Eigen::Vector3f to = pn.getNormalVector3fMap() - pose_vector;

    Eigen::Vector3f tangent = to - from;
    if (tangent.squaredNorm() < 1e-3f) {
      return from.norm() < 1.42 * max_range;
    }

    float inner = from.dot(tangent);
    float mu = std::clamp(inner / tangent.squaredNorm(), -1.0f, 0.0f);
    Eigen::Vector3f nearest = from - tangent * mu;
    return nearest.norm() < 1.42 * max_range;
  };

  LineSegments near_linestrings;
  for (const pcl::PointNormal & pn : linesegments) {
    if (checkIntersection(pn)) {
      near_linestrings.push_back(pn);
    }
  }
  return near_linestrings;
}

}  // namespace vmvl_validation