#pragma once
#include "imgproc/ransac_vanish_point.hpp"

#include <eigen3/Eigen/StdVector>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace imgproc
{
class VanishPoint : public rclcpp::Node
{
public:
  using Imu = sensor_msgs::msg::Imu;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  VanishPoint();

private:
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;

  std::optional<CameraInfo> info_{std::nullopt};
  RansacVanishPoint ransac_vanish_point_;

  void callbackImu(const Imu & msg);
  void callbackImage(const Image & msg);
};
}  // namespace imgproc