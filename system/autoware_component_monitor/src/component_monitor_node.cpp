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
  usage_pub_ = create_publisher<msg_t>("component_system_usage", rclcpp::SensorDataQoS());

  const auto p = bp::search_path("top");
  if (p.empty()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Couldn't find 'top' in path.");
    rclcpp::shutdown();
  }

  const auto pid = getpid();

  rclcpp::Rate loop_rate{10.0};
  try {
    while (rclcpp::ok()) {
      monitor(pid);
      loop_rate.sleep();
    }
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "An unknown error occurred.");
  }
}

void ComponentMonitor::monitor(const pid_t & pid) const
{
  if (usage_pub_->get_subscription_count() == 0) return;

  msg_t usage_msg{};

  auto fields = get_stats(pid);
  usage_msg.cpu_usage_percentage = get_cpu_percentage(fields);

  const auto [total_mem_bytes, free_mem_bytes] = get_system_memory(fields);
  usage_msg.total_memory_bytes = total_mem_bytes;
  usage_msg.free_memory_bytes = free_mem_bytes;

  const auto [used_mem_bytes, mem_usage_percentage] = get_process_memory(fields);
  usage_msg.used_memory_bytes = used_mem_bytes;
  usage_msg.memory_usage_percentage = mem_usage_percentage;

  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = "component_monitor";
  usage_msg.header = header;
  usage_msg.pid = pid;
  usage_pub_->publish(usage_msg);
}

/**
 * @brief Get system usage of the component.
 *
 * @details The output of top -b -n 1 -E k -p PID` is like below:
 *
 * top - 13:57:26 up  3:14,  1 user,  load average: 1,09, 1,10, 1,04
 * Tasks:   1 total,   0 running,   1 sleeping,   0 stopped,   0 zombie
 * %Cpu(s):  0,0 us,  0,8 sy,  0,0 ni, 99,2 id,  0,0 wa,  0,0 hi,  0,0 si,  0,0 st
 * KiB Mem : 65532208 total, 35117428 free, 17669824 used, 12744956 buff/cache
 * KiB Swap: 39062524 total, 39062524 free,        0 used. 45520816 avail Mem
 *
 *     PID USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND
 *    3352 meb       20   0 2905940   1,2g  39292 S   0,0   2,0  23:24.01 awesome
 *
 * We get 5th, 8th, and 9th fields, which are RES, %CPU, and %MEM, respectively.
 *
 */
fields_t ComponentMonitor::get_stats(const pid_t & pid) const
{
  std::string top_cmd{"top -b -n 1 -E k -p "};
  top_cmd += std::to_string(pid);

  auto std_out = run_command(top_cmd);
  return get_fields(std_out);
}

/**
 * @brief Run a terminal command and return the standard output.
 *
 * @param cmd The terminal command to run
 * @return  The standard output of the command
 */
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

/**
 * @brief Get fields from the standard output.
 *
 * @param std_out The standard output
 * @return The fields of the output
 */
fields_t ComponentMonitor::get_fields(std::stringstream & std_out)
{
  fields_t fields;
  std::string line;

  while (std::getline(std_out, line)) {
    std::istringstream iss{line};
    std::string word;
    std::vector<std::string> words;

    while (iss >> word) {
      words.push_back(word);
    }

    fields.push_back(words);
  }

  return fields;
}

/**
 * @brief Get the CPU usage percentage.
 *
 * @param fields The fields of the standard output from the terminal command
 * @return The CPU usage percentage
 */
float ComponentMonitor::get_cpu_percentage(const fields_t & fields)
{
  const auto & cpu_percentage = fields.back()[8];
  return to_float(cpu_percentage);
}

/**
 * @brief Get the system memory usage.
 *
 * @param fields The fields of the standard output from the terminal command
 * @return The total physical memory and free physical memory
 */
std::pair<uint64_t, uint64_t> ComponentMonitor::get_system_memory(const fields_t & fields)
{
  // System wide memory usage
  const auto total_memory_bytes = kib_to_bytes(to_uint64(fields[3][3]));
  const auto free_memory_bytes = kib_to_bytes(to_uint64(fields[3][5]));

  return std::pair{total_memory_bytes, free_memory_bytes};
}

/**
 * @brief Get the process memory usage.
 *
 * @param fields The fields of the standard output from the terminal command
 * @return The used memory by the process and the memory usage percentage
 */
std::pair<uint64_t, float> ComponentMonitor::get_process_memory(fields_t & fields)
{
  // Process specific memory usage
  auto & mem_usage = fields.back()[5];
  const auto & memory_usage_percentage = fields.back()[9];

  uint64_t used_memory_bytes{};
  switch (mem_usage.back()) {
    case 'm':
      mem_usage.pop_back();
      used_memory_bytes = mib_to_bytes(to_uint64(mem_usage));
      break;
    case 'g':
      mem_usage.pop_back();
      used_memory_bytes = gib_to_bytes(to_uint64(mem_usage));
      break;
    case 't':
      mem_usage.pop_back();
      used_memory_bytes = tib_to_bytes(to_uint64(mem_usage));
      break;
    case 'p':
      mem_usage.pop_back();
      used_memory_bytes = pib_to_bytes(to_uint64(mem_usage));
      break;
    case 'e':
      mem_usage.pop_back();
      used_memory_bytes = eib_to_bytes(to_uint64(mem_usage));
      break;
    default:
      used_memory_bytes = to_uint64(mem_usage);
  }

  return std::pair{used_memory_bytes, to_float(memory_usage_percentage)};
}

float ComponentMonitor::to_float(const std::string & str)
{
  return std::strtof(str.c_str(), nullptr);
}

uint64_t ComponentMonitor::to_uint64(const std::string & str)
{
  return std::strtoul(str.c_str(), nullptr, 10);
}

// cSpell:ignore mebibytes, gibibytes, tebibytes, pebibytes, exbibytes

uint64_t ComponentMonitor::kib_to_bytes(const uint64_t kibibytes)
{
  return kibibytes * 1024;
}

uint64_t ComponentMonitor::mib_to_bytes(const uint64_t mebibytes)
{
  return mebibytes * 1024 * 1024;
}

uint64_t ComponentMonitor::gib_to_bytes(const uint64_t gibibytes)
{
  return gibibytes * 1024 * 1024 * 1024;
}

uint64_t ComponentMonitor::tib_to_bytes(const uint64_t tebibytes)
{
  return tebibytes * 1024ULL * 1024ULL * 1024ULL * 1024ULL;
}

uint64_t ComponentMonitor::pib_to_bytes(const uint64_t pebibytes)
{
  return pebibytes * 1024ULL * 1024ULL * 1024ULL * 1024ULL * 1024ULL;
}

uint64_t ComponentMonitor::eib_to_bytes(const uint64_t exbibytes)
{
  return exbibytes * 1024ULL * 1024ULL * 1024ULL * 1024ULL * 1024ULL * 1024ULL;
}

}  // namespace autoware::component_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::component_monitor::ComponentMonitor)
