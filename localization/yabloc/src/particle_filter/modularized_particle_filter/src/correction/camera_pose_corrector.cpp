#include "modularized_particle_filter/correction/camera_pose_corrector.hpp"

#include <cv_bridge/cv_bridge.h>

#include <iostream>

void CameraPoseCorrector::syncrhoCallback(
  const sensor_msgs::msg::Image & msg1, const sensor_msgs::msg::Image & msg2)
{
  RCLCPP_INFO_STREAM(
    this->get_logger(), "ll2: " << msg1.header.stamp.sec << "." << msg1.header.stamp.nanosec);
  RCLCPP_INFO_STREAM(
    this->get_logger(), "lsd: " << msg2.header.stamp.sec << "." << msg2.header.stamp.nanosec);
  cv::Mat ll2_image = cv_bridge::toCvCopy(msg1, "bgr8")->image;
  cv::Mat lsd_image = cv_bridge::toCvCopy(msg2, "rgb8")->image;

  cv::Mat match_image = ll2_image + lsd_image;
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