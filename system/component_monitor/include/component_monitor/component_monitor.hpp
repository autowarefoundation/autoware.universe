#pragma once

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "boost/filesystem.hpp"
#include "boost/process.hpp"

namespace bp = boost::process;
namespace fs = boost::filesystem;

namespace autoware::component_monitor
{

class ComponentMonitor : public rclcpp::Node
{
public:
  explicit ComponentMonitor(const rclcpp::NodeOptions & node_options);

private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cpu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mem_pub_;

  pid_t pid_;
};

}  // namespace autoware::component_monitor
// autoware
