#include "tier4_timing_violation_monitor_utils/message_consumption_notifier.hpp"

using tier4_timing_violation_monitor_utils::MessageConsumptionNotifier;

MessageConsumptionNotifier::MessageConsumptionNotifier(
  rclcpp::Node * node,
  const std::string & notification_topic,
  const rclcpp::QoS & notification_qos)
{
  pub_ = node->create_publisher<MttMsg>(notification_topic, notification_qos);
}

void MessageConsumptionNotifier::notify(const builtin_interfaces::msg::Time & consumed_msg_stamp)
{
  // TODO(y-okumura-isp): construct message
  pub_->publish(consumed_msg_stamp);
}
