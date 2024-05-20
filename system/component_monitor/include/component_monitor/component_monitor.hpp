#pragma once

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"

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
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cpu_pub_;
};

}  // namespace autoware::component_monitor
// autoware
