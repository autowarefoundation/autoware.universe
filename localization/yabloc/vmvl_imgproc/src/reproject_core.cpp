#include "vmvl_imgproc/reproject.hpp"

#include <vml_common/cv_decompress.hpp>
#include <vml_common/pub_sub.hpp>
#include <vml_common/timer.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace imgproc
{
Reprojector::Reprojector() : Node("reprojector"), info_(this), tf_subscriber_(this->get_clock())
{
  using std::placeholders::_1;

  // Subscriber
  auto on_image = std::bind(&Reprojector::onImage, this, _1);
  auto on_lsd = std::bind(&Reprojector::onLineSegments, this, _1);
  sub_image_ = create_subscription<Image>("/sensing/camera/undistorted/image_raw", 10, on_image);
  sub_lsd_ = create_subscription<PointCloud2>("lsd_cloud", 10, on_lsd);

  // Publisher
  pub_image_ = create_publisher<Image>("reprojected_image", 10);
}

void Reprojector::onTwist(TwistStamped::ConstSharedPtr msg) { twist_list_.push_back(msg); }

void Reprojector::onImage(Image::ConstSharedPtr msg)
{
  image_list_.push_back(msg);

  cv::Mat image = vml_common::decompress2CvMat(*msg);
  vml_common::publishImage(*pub_image_, image, msg->header.stamp);
}

void Reprojector::onLineSegments(const PointCloud2 & msg)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr lsd{new pcl::PointCloud<pcl::PointNormal>()};
  pcl::fromROSMsg(msg, *lsd);
  RCLCPP_INFO_STREAM(get_logger(), "segments size:" << lsd->size());
}

void Reprojector::reproject(const Image & old_image_msg, const Image & cur_image_msg)
{
  std::cout << "start reproject()" << std::endl;

  // Check intrinsic & extrinsic
  std::optional<Eigen::Affine3f> camera_extrinsic = tf_subscriber_(info_.getFrameId(), "base_link");
  if (!camera_extrinsic.has_value()) return;
  if (info_.isCameraInfoNullOpt()) return;
  const Eigen::Matrix3f K = info_.intrinsic();
  const Eigen::Matrix3f Kinv = K.inverse();

  std::cout << "extrinsic & intrinsic are ready" << std::endl;

  cv::Mat old_image = vml_common::decompress2CvMat(old_image_msg);
  cv::Mat cur_image = vml_common::decompress2CvMat(cur_image_msg);
  rclcpp::Time old_stamp{old_image_msg.header.stamp};
  rclcpp::Time cur_stamp{cur_image_msg.header.stamp};

  // Compute travel distance
  Sophus::SE3f pose = accumulateTravelDistance(old_stamp, cur_stamp);
  std::cout << "relative pose: " << pose.translation().transpose() << ", "
            << pose.unit_quaternion().coeffs().transpose() << std::endl;

  // Compute homogrpahy matrix
  Eigen::Matrix3f H;  // TODO:

  // Reproject linesegments

  std::cout << "finish reproject()" << std::endl;
}

Sophus::SE3f Reprojector::accumulateTravelDistance(
  const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp)
{
  // TODO: Honestly, this accumulation does not provide accurate relative pose

  Sophus::SE3f pose;

  rclcpp::Time last_stamp = from_stamp;
  for (auto itr = twist_list_.begin(); itr != twist_list_.end(); ++itr) {
    rclcpp::Time stamp{(*itr)->header.stamp};
    if (stamp < from_stamp) continue;
    if (stamp > to_stamp) break;

    double dt = (stamp - last_stamp).seconds();
    const auto & linear = (*itr)->twist.linear;
    const auto & angular = (*itr)->twist.angular;

    Eigen::Vector3f v(linear.x, linear.y, linear.z);
    Eigen::Vector3f w(angular.x, angular.y, angular.z);

    pose *= Sophus::SE3f{Sophus::SO3f::exp(w * dt), dt * v};
    last_stamp = stamp;
  }

  return pose;
}

void Reprojector::popObsoleteMsg(const rclcpp::Time & stamp)
{
  if (image_list_.size() < 10) {
    return;
  }

  rclcpp::Time oldest_stamp = image_list_.front()->header.stamp;
  image_list_.pop_front();

  for (auto itr = twist_list_.begin(); itr != twist_list_.end();) {
    if (rclcpp::Time((*itr)->header.stamp) > oldest_stamp) break;
    itr = twist_list_.erase(itr);
  }
}

}  // namespace imgproc
