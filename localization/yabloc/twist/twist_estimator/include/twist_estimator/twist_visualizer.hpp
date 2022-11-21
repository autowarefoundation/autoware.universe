#pragma once
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sophus/geometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <boost/circular_buffer.hpp>

#include <optional>

namespace pcdless::twist_visualizer
{
class TwistVisualizer : public rclcpp::Node
{
public:
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Path = nav_msgs::msg::Path;

  TwistVisualizer();

private:
  rclcpp::Publisher<Path>::SharedPtr pub_path_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_stamped_;

  boost::circular_buffer<Sophus::SE3f> odoms_;

  std::optional<rclcpp::Time> last_twist_stamp_{std::nullopt};
  Sophus::SE3f last_odom_;

  void on_twist_stamped(const TwistStamped & msg);

  void publish_path(const rclcpp::Time & stmap);

  Pose se3f_to_pose_msg(const Sophus::SE3f & pose);
};
}  // namespace pcdless::twist_visualizer