#pragma once

#include "rclcpp/rclcpp.hpp"

#include "autoware_internal_msgs/msg/system_usage.hpp"
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
  std::stringstream run_command(const std::string & cmd);
  std::vector<std::string> get_fields(std::stringstream & std_out);
  void get_cpu_usage();
  void get_mem_usage();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<autoware_internal_msgs::msg::SystemUsage>::SharedPtr usage_pub_;

  pid_t pid_;
  autoware_internal_msgs::msg::SystemUsage usage_msg_{};
};

}  // namespace autoware::component_monitor
// autoware
