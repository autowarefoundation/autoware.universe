#pragma once
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <optional>
namespace pcdless::common
{
class CameraInfoSubscriber
{
public:
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  CameraInfoSubscriber(rclcpp::Node * node);

  bool is_camera_info_ready() const;

  bool is_camera_info_nullopt() const;

  Eigen::Vector2i size() const;

  Eigen::Matrix3f intrinsic() const;

  // This member function DOES NOT check isCameraInfoReady()
  // If it it not ready, throw bad optional access
  std::string get_frame_id() const;

private:
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  std::optional<CameraInfo> opt_info_;
};
}  // namespace pcdless::common