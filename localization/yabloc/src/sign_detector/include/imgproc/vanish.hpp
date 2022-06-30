#pragma once
#include <lsd/lsd.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace impgroc
{
class VanishPoint : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  VanishPoint();

private:
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  std::optional<CameraInfo> info_{std::nullopt};
  cv::Ptr<cv::lsd::LineSegmentDetector> lsd_{nullptr};

  void callbackImage(const Image & msg);
  void callbackInfo(const CameraInfo & msg) { info_ = msg; }
};
}  // namespace impgroc