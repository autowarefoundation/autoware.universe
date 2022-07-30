#pragma once
#include <eigen3/Eigen/StdVector>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

class BaseCameraInfoNode : public rclcpp::Node
{
public:
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  BaseCameraInfoNode(
    const std::string node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  bool isCameraInfoReady() const;
  bool isCameraInfoNullOpt() const;

  Eigen::Matrix3f intrinsic() const;

private:
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  std::optional<CameraInfo> opt_info_;
};
