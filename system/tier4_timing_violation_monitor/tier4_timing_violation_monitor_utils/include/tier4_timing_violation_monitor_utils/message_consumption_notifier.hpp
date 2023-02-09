#ifndef TIER4_TIMING_VIOLATION_MONITOR_UTILS__MESSAGE_CONSUMPTION_NOTIFIER_HPP_
#define TIER4_TIMING_VIOLATION_MONITOR_UTILS__MESSAGE_CONSUMPTION_NOTIFIER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace tier4_timing_violation_monitor_utils
{
class MessageConsumptionNotifier
{
  using MttMsg = builtin_interfaces::msg::Time;
  using MTTPublisher = rclcpp::Publisher<MttMsg>;

public:
  RCLCPP_PUBLIC
  explicit MessageConsumptionNotifier(
    rclcpp::Node * node,
    const std::string & notification_topic,
    const rclcpp::QoS & notification_qos);

  void notify(const builtin_interfaces::msg::Time & consumed_msg_stamp);

private:
  std::shared_ptr<MTTPublisher> pub_;
};

}   // namespace tier4_timing_violation_monitor_utils

#endif  // TIER4_TIMING_VIOLATION_MONITOR_UTILS__MESSAGE_CONSUMPTION_NOTIFIER_HPP_
