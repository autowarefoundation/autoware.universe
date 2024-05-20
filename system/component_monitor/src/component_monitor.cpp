#include "component_monitor/component_monitor.hpp"

namespace autoware::component_monitor
{
ComponentMonitor::ComponentMonitor(const rclcpp::NodeOptions & node_options)
: Node("component_monitor", node_options)
{
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ComponentMonitor::timer_callback, this));
  cpu_pub_ = create_publisher<std_msgs::msg::Float32>("cpu_usage", rclcpp::SensorDataQoS());

  const auto p = bp::search_path("top");
  if (p.empty()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Couldn't find 'top' in path.");
    rclcpp::shutdown();
  }
}

void ComponentMonitor::timer_callback()
{
  RCLCPP_INFO_STREAM(get_logger(), "Callback");
}
}  // namespace autoware::component_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::component_monitor::ComponentMonitor)
