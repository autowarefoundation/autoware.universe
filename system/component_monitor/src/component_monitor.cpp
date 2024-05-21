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
  get_cpu_usage();
  get_mem_usage();

  usage_pub_->publish(usage_msg_);
}

std::stringstream ComponentMonitor::run_command(const std::string & cmd)
{
  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error setting flags on out_fd");
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error setting flags on err_fd");
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  bp::child c(cmd, bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    RCLCPP_ERROR_STREAM(get_logger(), "Error running command: " << os.str());
  }

  std::stringstream os;
  os << is_out.rdbuf();
  return os;
}

std::vector<std::string> ComponentMonitor::get_fields(std::stringstream & std_out)
{
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(std_out, line)) {
    lines.push_back(line);
  }

  std::istringstream last_line(lines.back());
  std::string word;
  std::vector<std::string> words;
  while (last_line >> word) {
    words.push_back(word);
  }

  return words;
}

void ComponentMonitor::get_cpu_usage()
{
  std::string cmd{"pidstat -u -p "};
  cmd += std::to_string(pid_);

  auto std_out = run_command(cmd);
  auto fields = get_fields(std_out);

  auto cpu_rate = fields[7];
  std::replace(cpu_rate.begin(), cpu_rate.end(), ',', '.');

  usage_msg_.cpu_usage_rate = std::strtof(cpu_rate.c_str(), nullptr);
}

void ComponentMonitor::get_mem_usage()
{
  std::string cmd{"pidstat -r -p "};
  cmd += std::to_string(pid_);

  auto std_out = run_command(cmd);
  auto fields = get_fields(std_out);

  auto mem_usage = fields[6];
  auto mem_usage_rate = fields[7];

  std::replace(mem_usage.begin(), mem_usage.end(), ',', '.');
  std::replace(mem_usage_rate.begin(), mem_usage_rate.end(), ',', '.');

  usage_msg_.mem_usage_bytes = std::strtof(mem_usage.c_str(), nullptr);
  usage_msg_.mem_usage_rate = std::strtof(mem_usage_rate.c_str(), nullptr);
}

}  // namespace autoware::component_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::component_monitor::ComponentMonitor)
