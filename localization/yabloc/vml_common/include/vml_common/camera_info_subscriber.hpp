#pragma once
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <optional>
namespace vml_common
{
class CameraInfoSubscriber
{
public:
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  CameraInfoSubscriber(rclcpp::Node * node);

  bool isCameraInfoReady() const;

  bool isCameraInfoNullOpt() const;

  Eigen::Matrix3f intrinsic() const;

  // This member function DOES NOT check isCameraInfoReady()
  std::string getFrameId() const;

private:
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  std::optional<CameraInfo> opt_info_;
};
}  // namespace vml_common