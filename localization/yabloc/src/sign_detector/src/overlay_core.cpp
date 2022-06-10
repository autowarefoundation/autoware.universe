#include "common/util.hpp"
#include "sign_detector/ll2_util.hpp"
#include "sign_detector/overlay.hpp"

#include <eigen3/Eigen/StdVector>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <sophus/geometry.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

void Overlay::cloudPoseCallback(const CloudWithPose & msg) { latest_cloud_with_pose_ = msg; }

void Overlay::infoCallback(const sensor_msgs::msg::CameraInfo & msg)
{
  info_ = msg;
  listenExtrinsicTf(info_->header.frame_id);
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

  overlay(image, synched_pose.pose, stamp);
}

void Overlay::cloudCallback(const PointCloud2 & msg)
{
  using LineSegment = pcl::PointCloud<pcl::PointNormal>;
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

void Overlay::overlay(const cv::Mat & image, const Pose & pose, const rclcpp::Time & stamp)
{
  if (!latest_cloud_with_pose_.has_value()) return;

  LineSegments ll2_cloud;
  pcl::fromROSMsg(latest_cloud_with_pose_->cloud, ll2_cloud);
  Eigen::Affine3f query_tf = util::pose2Affine(pose);
  Eigen::Affine3f ref_tf = util::pose2Affine(latest_cloud_with_pose_->pose);
  Eigen::Affine3f transform = ref_tf.inverse() * query_tf;

  cv::Mat overlayed_image = cv::Mat::zeros(image.size(), CV_8UC3);

  Eigen::Matrix3f K =
    Eigen::Map<Eigen::Matrix<double, 3, 3> >(info_->k.data()).cast<float>().transpose();
  Eigen::Affine3f T = camera_extrinsic_.value();
  auto project = [K, T, transform](const Eigen::Vector3f & xyz) -> std::optional<cv::Point2i> {
    Eigen::Vector3f from_camera = K * T.inverse() * transform.inverse() * xyz;
    if (from_camera.z() < 1e-3f) return std::nullopt;
    Eigen::Vector3f uv1 = from_camera /= from_camera.z();
    return cv::Point2i(uv1.x(), uv1.y());
  };

  for (const pcl::PointNormal & pn : ll2_cloud) {
    auto p1 = project(pn.getArray3fMap()), p2 = project(pn.getNormalVector3fMap());
    if (!p1.has_value() || !p2.has_value()) continue;
    cv::line(overlayed_image, p1.value(), p2.value(), cv::Scalar(0, 255, 255), 2);
  }

  cv::Mat show_image;
  cv::addWeighted(image, 0.8, overlayed_image, 0.8, 1, show_image);
  util::publishImage(*pub_image_, show_image, stamp);
}

void Overlay::listenExtrinsicTf(const std::string & frame_id)
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
