// Copyright 2024 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "component_monitor_node.hpp"

namespace autoware::component_monitor
{
ComponentMonitor::ComponentMonitor(const rclcpp::NodeOptions & node_options)
: Node("component_monitor", node_options)
{
  usage_pub_ = create_publisher<autoware_internal_msgs::msg::SystemUsage>(
    "component_system_usage", rclcpp::SensorDataQoS());

  const auto p = bp::search_path("top");
  if (p.empty()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Couldn't find 'top' in path.");
    rclcpp::shutdown();
  }

  pid_ = getpid();
  usage_msg_.pid = pid_;

  rclcpp::Rate loop_rate{10.0};
  try {
    while (rclcpp::ok()) {
      monitor();
      loop_rate.sleep();
    }
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "An unknown error occured.");
  }
}

void ComponentMonitor::monitor()
{
  if (usage_pub_->get_subscription_count() == 0) return;

  get_stats();
  get_cpu_usage();
  get_mem_usage();
  publish();
}

void ComponentMonitor::publish()
{
  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = "component_monitor";
  usage_msg_.header = header;
  usage_pub_->publish(usage_msg_);
}

/**
 * @brief Get system usage of the component.
 *
 * @details The output of top -b -d 0.1 -n 1 -p PID` is like below:
 *
 * top - 15:32:33 up  4:34,  1 user,  load average: 3,11, 4,51, 3,67
 * Tasks:   1 total,   0 running,   1 sleeping,   0 stopped,   0 zombie
 * %Cpu(s):  0,0 us,  0,8 sy,  0,0 ni, 99,2 id,  0,0 wa,  0,0 hi,  0,0 si,  0,0 st
 * MiB Mem :  63996,4 total,  11536,7 free,  34437,3 used,  18022,4 buff/cache
 * MiB Swap:   2048,0 total,    894,2 free,   1153,8 used.  27854,7 avail Mem
 *
 *   PID USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND
 *  2837 meb       20   0 2681404   1,0g  34796 S   0,0   1,6  19:48.66 awesome
 *
 * We get 5th, 8th, and 9th fields, which are RES, %CPU, and %MEM, respectively.
 *
 */
void ComponentMonitor::get_stats()
{
  std::string cmd{"top -b -d 0.1 -n 1 -p "};
  cmd += std::to_string(pid_);

  auto std_out = run_command(cmd);
  fields_ = get_fields(std_out);
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
  env["LC_NUMERIC"] = "en_US.UTF-8";  // To make sure that decimal separator is a dot.

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

void ComponentMonitor::get_cpu_usage()
{
  const auto & cpu_rate = fields_[8];
  usage_msg_.cpu_usage_rate = to_float(cpu_rate);
}

void ComponentMonitor::get_mem_usage()
{
  auto & mem_usage = fields_[5];
  const auto & mem_usage_rate = fields_[9];

  uint64_t mem_usage_kib{};
  switch (mem_usage.back()) {
    case 'm':
      mem_usage.pop_back();
      mem_usage_kib = mib_to_kib(to_uint32(mem_usage));
      break;
    case 'g':
      mem_usage.pop_back();
      mem_usage_kib = gib_to_kib(to_uint32(mem_usage));
      break;
    case 't':
      mem_usage.pop_back();
      mem_usage_kib = tib_to_kib(to_uint32(mem_usage));
      break;
    case 'p':
      mem_usage.pop_back();
      mem_usage_kib = pib_to_kib(to_uint32(mem_usage));
      break;
    case 'e':
      mem_usage.pop_back();
      mem_usage_kib = eib_to_kib(to_uint32(mem_usage));
      break;
    default:
      mem_usage_kib = to_uint32(mem_usage);
  }

  usage_msg_.mem_usage_kib = mem_usage_kib;
  usage_msg_.mem_usage_rate = to_float(mem_usage_rate);
}

float ComponentMonitor::to_float(const std::string & str)
{
  return std::strtof(str.c_str(), nullptr);
}

uint32_t ComponentMonitor::to_uint32(const std::string & str)
{
  return std::strtoul(str.c_str(), nullptr, 10);
}

// cspell:ignore mebibytes,gibibytes,tebibytes,pebibytes,exbibytes
uint64_t ComponentMonitor::mib_to_kib(uint64_t mebibytes)
{
  return mebibytes * 1024;
}

uint64_t ComponentMonitor::gib_to_kib(uint64_t gibibytes)
{
  return gibibytes * 1024 * 1024;
}

uint64_t ComponentMonitor::tib_to_kib(uint64_t tebibytes)
{
  return tebibytes * 1024ULL * 1024ULL * 1024ULL;
}

uint64_t ComponentMonitor::pib_to_kib(uint64_t pebibytes)
{
  return pebibytes * 1024ULL * 1024ULL * 1024ULL * 1024ULL;
}

uint64_t ComponentMonitor::eib_to_kib(uint64_t exbibytes)
{
  return exbibytes * 1024ULL * 1024ULL * 1024ULL * 1024ULL * 1024ULL;
}

}  // namespace autoware::component_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::component_monitor::ComponentMonitor)
