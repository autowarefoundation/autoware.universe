#pragma once
#include <opencv4/opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <optional>

namespace pcdless::hsv_extractor
{
class HsvExtractor : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  HsvExtractor();

private:
  const int threshold_;
  const float saturation_gain_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_lsd_;

  void on_image(const sensor_msgs::msg::Image & msg);
  void execute(const cv::Mat & image, const rclcpp::Time & stamp);
};
}  // namespace pcdless::hsv_extractor