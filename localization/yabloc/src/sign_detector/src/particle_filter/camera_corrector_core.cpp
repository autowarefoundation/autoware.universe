#include "common/util.hpp"
#include "particle_filter/camera_corrector.hpp"

#include <opencv4/opencv2/imgproc.hpp>

#include <pcl_conversions/pcl_conversions.h>
namespace particle_filter
{
cv::Point2f CameraParticleCorrector::toCvPoint(const Eigen::Vector3f & p)
{
  const cv::Size center(image_size_ / 2, image_size_ / 2);
  cv::Point2f pt;
  pt.x = -p.y() / max_range_ * center.width + center.width;
  pt.y = -p.x() / max_range_ * center.height + 2 * center.height;
  return pt;
}

void CameraParticleCorrector::lsdCallback(const sensor_msgs::msg::PointCloud2 & lsd_msg)
{
  const rclcpp::Time stamp = lsd_msg.header.stamp;
  std::optional<ParticleArray> opt_array = this->getSyncronizedParticleArray(stamp);

  if (!opt_array.has_value()) return;
  if (!latest_cloud_with_pose_.has_value()) return;

  float dt = (stamp - opt_array->header.stamp).seconds();
  RCLCPP_INFO_STREAM(get_logger(), "opt_arary stamp delay: " << dt);

  LineSegment lsd_cloud, ll2_cloud;
  pcl::fromROSMsg(lsd_msg, lsd_cloud);
  pcl::fromROSMsg(latest_cloud_with_pose_->cloud, ll2_cloud);

  cv::Mat ll2_image = buildLl2Image(ll2_cloud);
  cv::Mat rgb_ll2_image;
  cv::applyColorMap(ll2_image, rgb_ll2_image, cv::COLORMAP_JET);
  util::publishImage(*image_pub_, rgb_ll2_image, stamp);
}

void CameraParticleCorrector::ll2Callback(const CloudWithPose & ll2_msg)
{
  latest_cloud_with_pose_ = ll2_msg;
}

cv::Mat CameraParticleCorrector::buildLl2Image(const LineSegment & cloud)
{
  cv::Mat image = 255 - cv::Mat::ones(cv::Size(image_size_, image_size_), CV_8UC1);
  for (const auto pn : cloud) {
    cv::line(
      image, toCvPoint(pn.getVector3fMap()), toCvPoint(pn.getNormalVector3fMap()),
      cv::Scalar::all(0), 1);
  }
  cv::Mat distance;
  cv::distanceTransform(image, distance, cv::DIST_L2, 3);
  cv::threshold(distance, distance, 100, 100, cv::THRESH_TRUNC);
  distance.convertTo(distance, CV_8UC1, -2.55, 255);
  return gamma_converter(distance);
}

}  // namespace particle_filter