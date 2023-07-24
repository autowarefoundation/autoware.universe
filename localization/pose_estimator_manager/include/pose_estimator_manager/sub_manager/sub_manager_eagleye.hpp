#pragma once
#include "pose_estimator_manager/base_pose_estimator_sub_manager.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace multi_pose_estimator
{
class SubManagerEagleye : public BasePoseEstimatorSubManager
{
public:
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  SubManagerEagleye(rclcpp::Node * node) : BasePoseEstimatorSubManager(node)
  {
    eagleye_is_enabled_ = true;

    using std::placeholders::_1;
    auto on_pose = std::bind(&SubManagerEagleye::on_pose, this, _1);
    sub_pose_ = node->create_subscription<PoseCovStamped>("~/input/pose", 5, on_pose);
    pub_pose_ = node->create_publisher<PoseCovStamped>("~/output/pose", 5);
  }

  void set_enable(bool enabled) override { eagleye_is_enabled_ = enabled; }

private:
  bool eagleye_is_enabled_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_pose_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_pose_;

  void on_pose(PoseCovStamped::ConstSharedPtr msg)
  {
    if (eagleye_is_enabled_) {
      pub_pose_->publish(*msg);
    }
  }
};
}  // namespace multi_pose_estimator
