#pragma once
#include <opencv4/opencv2/core.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace pcdless::common
{
cv::Mat decompress_to_cv_mat(const sensor_msgs::msg::Image & img);

sensor_msgs::msg::Image::ConstSharedPtr decompress_to_ros_msg(
  const sensor_msgs::msg::CompressedImage & compressed_img, const std::string & encoding = "bgr8");

cv::Mat decompress_to_cv_mat(const sensor_msgs::msg::CompressedImage & compressed_img);

}  // namespace pcdless::common