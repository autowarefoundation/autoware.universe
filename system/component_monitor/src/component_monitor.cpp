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

  const auto p = bp::search_path("pidstat");
  if (p.empty()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Couldn't find 'pidstat' in path.");
    rclcpp::shutdown();
  }

  pid_ = getpid();
}

void ComponentMonitor::timer_callback()
{
  get_cpu_usage();
  get_mem_usage();

  if (usage_pub_->get_subscription_count() > 0) usage_pub_->publish(usage_msg_);
}

std::stringstream ComponentMonitor::run_command(const std::string & cmd) const
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

  auto env = boost::this_process::environment();
  env["LC_NUMERIC"] = "en_US.UTF-8";

  bp::environment child_env = env;
  bp::child c(cmd, child_env, bp::std_out > is_out, bp::std_err > is_err);
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

float ComponentMonitor::to_float(const std::string & str)
{
  return std::strtof(str.c_str(), nullptr);
}

uint32_t ComponentMonitor::to_uint32(const std::string & str)
{
  return std::strtoul(str.c_str(), nullptr, 10);
}

/**
 * @brief Get CPU usage of the component.
 *
 * @details The output of `pidstat -u -p PID` is like below:
 *
 * Linux 6.5.0-35-generic (leopc) 	21-05-2024 	_x86_64_	(16 CPU)
 * 14:54:52      UID       PID    %usr %system  %guest   %wait    %CPU   CPU  Command
 * 14:54:52     1000      3280    0,00    0,00    0,00    0,00    0,00     0  awesome
 *
 * We get the 7th field, which is %CPU.
 */
void ComponentMonitor::get_cpu_usage()
{
  std::string cmd{"pidstat -u -p "};
  cmd += std::to_string(pid_);

  auto std_out = run_command(cmd);
  const auto fields = get_fields(std_out);

  const auto & cpu_rate = fields[7];
  usage_msg_.cpu_usage_rate = to_float(cpu_rate);
}

/**
 * @brief Get memory usage of the component.
 *
 * @details The output of `pidstat -r -p PID` is like below:
 *
 * Linux 6.5.0-35-generic (leopc) 	21-05-2024 	_x86_64_	(16 CPU)
 * 14:54:52      UID       PID  minflt/s  majflt/s     VSZ     RSS   %MEM  Command
 * 14:54:52     1000      3280   2426,99      0,00 1820884 1243692   1,90  awesome
 *
 * We get the 6th and 7th fields, which are RSS and %MEM, respectively.
 */
void ComponentMonitor::get_mem_usage()
{
  std::string cmd{"pidstat -r -p "};
  cmd += std::to_string(pid_);

  auto std_out = run_command(cmd);
  const auto fields = get_fields(std_out);

  const auto & mem_usage = fields[6];
  const auto & mem_usage_rate = fields[7];

  usage_msg_.mem_usage_bytes = to_uint32(mem_usage);
  usage_msg_.mem_usage_rate = to_float(mem_usage_rate);
}

}  // namespace autoware::component_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::component_monitor::ComponentMonitor)
