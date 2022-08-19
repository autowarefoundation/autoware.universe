#pragma once
#include <opencv4/opencv2/core.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace vml_common
{
cv::Mat decompress2CvMat(const sensor_msgs::msg::Image & img);

sensor_msgs::msg::Image::ConstSharedPtr decompress2RosMsg(
  const sensor_msgs::msg::CompressedImage & compressed_img);

cv::Mat decompress2CvMat(const sensor_msgs::msg::CompressedImage & compressed_img);

}  // namespace vml_common