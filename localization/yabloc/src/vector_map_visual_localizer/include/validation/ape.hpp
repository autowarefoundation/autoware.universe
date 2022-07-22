#pragma once
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>

namespace validation
{
class AbsolutePoseError : public rclcpp::Node
{
public:
  using String = std_msgs::msg::String;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  AbsolutePoseError();

private:
  const std::string reference_bags_path_;
  rclcpp::Publisher<String>::SharedPtr pub_string_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_pose_cov_stamped_;

  void poseCallback(const PoseCovStamped & pose_cov);
};
}  // namespace validation