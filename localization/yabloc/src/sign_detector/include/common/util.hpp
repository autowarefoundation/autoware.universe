#pragma once
#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/publisher.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace util
{
cv::Mat decompress2CvMat(const sensor_msgs::msg::Image & img);

sensor_msgs::msg::Image::ConstSharedPtr decompress2RosMsg(
  const sensor_msgs::msg::CompressedImage & compressed_img);

cv::Mat decompress2CvMat(const sensor_msgs::msg::CompressedImage & compressed_img);
sensor_msgs::msg::Image::ConstSharedPtr decompress2RosMsg(
  const sensor_msgs::msg::CompressedImage & compressed_img);

void publishImage(
  rclcpp::Publisher<sensor_msgs::msg::Image> & publisher, const cv::Mat & image,
  const rclcpp::Time & stamp);

Eigen::Affine3f pose2Affine(const geometry_msgs::msg::Pose & pose);
}  // namespace util