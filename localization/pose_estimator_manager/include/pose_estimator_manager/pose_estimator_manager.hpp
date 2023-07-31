#ifndef POSE_ESTIMATOR_MANAGER__POSE_ESTIMATOR_MANAGER_HPP_
#define POSE_ESTIMATOR_MANAGER__POSE_ESTIMATOR_MANAGER_HPP_

#include "pose_estimator_manager/base_pose_estimator_sub_manager.hpp"
#include "pose_estimator_manager/switch_rule/base_switch_rule.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace multi_pose_estimator
{
class PoseEstimatorManager : public rclcpp::Node
{
public:
  using SetBool = std_srvs::srv::SetBool;
  using String = std_msgs::msg::String;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  PoseEstimatorManager();

private:
  const std::unordered_set<PoseEstimatorName> running_estimator_list_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_array_;
  rclcpp::Publisher<String>::SharedPtr pub_debug_string_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::unordered_map<PoseEstimatorName, BasePoseEstimatorSubManager::SharedPtr> sub_managers_;

  std::shared_ptr<BaseSwitchRule> switch_rule_{nullptr};

  void toggle_all(bool enabled);
  void toggle_each(const std::unordered_map<PoseEstimatorName, bool> & toggle_list);

  void on_timer();

  void load_switch_rule();
};
}  // namespace multi_pose_estimator

#endif /* POSE_ESTIMATOR_MANAGER__POSE_ESTIMATOR_MANAGER_HPP_ */