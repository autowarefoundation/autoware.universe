#include "modularized_particle_filter/correction/camera_pose_corrector.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

cv::Mat cloud2Image(const sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(msg, cloud);

  cv::Mat image = cv::Mat::zeros(cv::Size(800, 800), CV_8UC1);
  for (const auto p : cloud) {
    image.at<uchar>(p.y, p.x) = 255;
  }
  return image;
}

void CameraPoseCorrector::syncrhoCallback(
  const PointCloud2 & msg1, const CloudPose & msg2, const ParticleArray & particles)
{
  RCLCPP_INFO_STREAM(
    this->get_logger(), "ll2: " << msg1.header.stamp.sec << "." << msg1.header.stamp.nanosec);
  RCLCPP_INFO_STREAM(
    this->get_logger(), "lsd: " << msg2.header.stamp.sec << "." << msg2.header.stamp.nanosec);

  cv::Mat ll2_image = cloud2Image(msg1);
  cv::Mat lsd_image = cloud2Image(msg2.cloud);

  cv::Mat match_image;
  cv::Mat zero_image = cv::Mat::zeros(ll2_image.size(), CV_8UC1);
  cv::merge(std::vector<cv::Mat>{ll2_image, lsd_image, zero_image}, match_image);
  publishImage(match_image, msg1.header.stamp);
}

void CameraPoseCorrector::publishImage(const cv::Mat & image, const rclcpp::Time & stamp)
{
  cv_bridge::CvImage raw_image;
  raw_image.header.stamp = stamp;
  raw_image.header.frame_id = "map";
  raw_image.encoding = "bgr8";
  raw_image.image = image;
  image_pub_->publish(*raw_image.toImageMsg());
}