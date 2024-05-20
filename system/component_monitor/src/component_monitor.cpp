#include "component_monitor/component_monitor.hpp"

namespace autoware::component_monitor
{
ComponentMonitor::ComponentMonitor(const rclcpp::NodeOptions & node_options)
: Node("component_monitor", node_options)
{
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ComponentMonitor::timer_callback, this));
  usage_pub_ = create_publisher<autoware_internal_msgs::msg::SystemUsage>(
    "component_system_usage", rclcpp::SensorDataQoS());

  const auto p = bp::search_path("top");
  if (p.empty()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Couldn't find 'top' in path.");
    rclcpp::shutdown();
  }

  pid_ = getpid();
}

void ComponentMonitor::timer_callback()
{
  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error setting flags on out_fd");
    return;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error setting flags on err_fd");
    return;
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  std::string cmd = "/bin/sh -c \"top -b -n 2 -d 0.2 -p ";
  cmd += std::to_string(pid_);
  cmd += " | tail -1 | awk '{print $9} {print $10}'\"";

  // RCLCPP_INFO_STREAM(get_logger(), "cmd: " << cmd);
  bp::child c(cmd, bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    RCLCPP_ERROR_STREAM(get_logger(), "Error running command: " << os.str());
    return;
  }

  std::string cpu, mem;
  std::getline(is_out, cpu);
  std::getline(is_out, mem);

  std::replace(cpu.begin(), cpu.end(), ',', '.');
  std::replace(mem.begin(), mem.end(), ',', '.');

  RCLCPP_INFO_STREAM(get_logger(), "CPU: " << cpu);
  RCLCPP_INFO_STREAM(get_logger(), "MEM: " << mem);

  if (usage_pub_->get_subscription_count() > 0) {
    auto usage_msg = autoware_internal_msgs::msg::SystemUsage();
    usage_msg.cpu_usage_rate = std::strtof(cpu.c_str(), nullptr);
    usage_msg.mem_usage_rate = std::strtof(mem.c_str(), nullptr);
    usage_pub_->publish(usage_msg);
  }
}
}  // namespace autoware::component_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::component_monitor::ComponentMonitor)
