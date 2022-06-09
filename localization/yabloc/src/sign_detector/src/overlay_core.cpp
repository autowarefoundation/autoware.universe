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
  cv::imshow("hoge", image);
  cv::waitKey(1);

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
  RCLCPP_INFO_STREAM(get_logger(), "dt: " << min_dt);
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