// Copyright 2020 Autoware Foundation
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

/**
 * @file memory_monitor.cpp
 * @brief Memory monitor class
 */

#include "system_monitor/mem_monitor/mem_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <boost/process.hpp>

#include <fmt/format.h>

#include <string>
#include <vector>

namespace bp = boost::process;

MemMonitor::MemMonitor(const rclcpp::NodeOptions & options)
: Node("mem_monitor", options),
  updater_(this),
  available_size_(declare_parameter<int>("available_size", 1024) * 1024 * 1024),
  usage_timeout_(declare_parameter<int>("usage_timeout", 5)),
  usage_timeout_expired_(false),
  ecc_timeout_(declare_parameter<int>("ecc_timeout", 5)),
  ecc_timeout_expired_(false)
{
  using namespace std::literals::chrono_literals;

  gethostname(hostname_, sizeof(hostname_));
  updater_.setHardwareID(hostname_);
  updater_.add("Memory Usage", this, &MemMonitor::checkUsage);

  // Start timer to execute checkUsage and checkEcc
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  usage_timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&MemMonitor::onUsageTimer, this), timer_callback_group_);

  // Enable ECC error detection if edac-utils package is installed
  if (!bp::search_path("edac-util").empty()) {
    updater_.add("Memory ECC", this, &MemMonitor::checkEcc);
    ecc_timer_ = rclcpp::create_timer(
      this, get_clock(), 1s, std::bind(&MemMonitor::onEccTimer, this), timer_callback_group_);
  }
}

void MemMonitor::update()
{
  updater_.force_update();
}

void MemMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::string error_str;
  std::map<std::string, size_t> map;
  double elapsed_ms;

  // thread-safe copy
  {
    std::lock_guard<std::mutex> lock(usage_mutex_);
    error_str = usage_error_str_;
    map = usage_map_;
    elapsed_ms = usage_elapsed_ms_;
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, "readUsage error");
    stat.add("readUsage", error_str);
    return;
  }

  // Check if Memory Usage is sound state
  int level;
  if (map["Mem: usage"] > map["Total: used+"]) {
    level = DiagStatus::OK;
  } else if (map["Mem: available"] >= available_size_) {
    level = DiagStatus::WARN;
  } else {
    level = DiagStatus::ERROR;
  }

  for (auto itr = map.begin(); itr != map.end(); ++itr) {
    if (itr->first == "Mem: usage") {
      stat.addf(itr->first, "%.2f%%", static_cast<double>(itr->second));
    } else {
      stat.add(itr->first, toHumanReadable(std::to_string(itr->second)));
    }
  }

  // Check timeout has expired regarding executing readUsage
  bool timeout_expired = false;
  {
    std::lock_guard<std::mutex> lock(usage_timeout_mutex_);
    timeout_expired = usage_timeout_expired_;
  }

  if (!timeout_expired) {
    stat.summary(level, usage_dict_.at(level));
  } else {
    stat.summary(DiagStatus::WARN, "readUsage timeout expired");
  }

  stat.addf("execution time", "%f ms", elapsed_ms);
}

void MemMonitor::checkEcc(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::string error_str;
  std::string pipe2_error_str;
  std::string output;
  double elapsed_ms;

  // thread-safe copy
  {
    std::lock_guard<std::mutex> lock(ecc_mutex_);
    error_str = ecc_error_str_;
    output = ecc_output_;
    elapsed_ms = ecc_elapsed_ms_;
  }

  if (!pipe2_error_str.empty()) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", pipe2_error_str);
    return;
  }
  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, "edac_util error");
    stat.add("edac_util", error_str);
    return;
  }

  /*
   Output example of `edac-util --quiet`
   edac-util generates output if error occurred, otherwise no output
   mc0: 3 Uncorrected Errors with no DIMM info
   mc0: 3 Corrected Errors with no DIMM info
   */
  std::istringstream iss(ecc_output_);
  std::string line;

  while (std::getline(iss, line)) {
    if (line.find("Uncorrected") != std::string::npos) {
      stat.summary(DiagStatus::ERROR, line);
      return;
    } else if (line.find("Corrected") != std::string::npos) {
      stat.summary(DiagStatus::WARN, line);
      return;
    }
  }

  // Check timeout has expired regarding executing edac-util command
  bool timeout_expired = false;
  {
    std::lock_guard<std::mutex> lock(ecc_timeout_mutex_);
    timeout_expired = ecc_timeout_expired_;
  }

  if (!timeout_expired) {
    stat.summary(DiagStatus::OK, "OK");
  } else {
    stat.summary(DiagStatus::WARN, "edac-util command timeout expired");
  }

  stat.addf("execution time", "%f ms", elapsed_ms);
}

void MemMonitor::readMemInfo(std::unordered_map<std::string, size_t> & memInfo)
{
  std::ifstream file("/proc/meminfo");

  if (!file.is_open()) {
    throw std::runtime_error("Could not open /proc/meminfo");
  }

  std::string line;
  while (std::getline(file, line)) {
    std::size_t pos = line.find(':');
    if (pos != std::string::npos) {
      std::string key = line.substr(0, pos);
      try {
        size_t value = std::stoll(line.substr(pos + 1)) * 1024;
        memInfo[key] = value;
      } catch (const std::invalid_argument & e) {
        throw std::runtime_error("Invalid value in /proc/meminfo: " + line);
      } catch (const std::out_of_range & e) {
        throw std::runtime_error("Value out of range in /proc/meminfo: " + line);
      }
    } else {
      throw std::runtime_error("Invalid line in /proc/meminfo: " + line);
    }
  }
}

std::string MemMonitor::readUsage(std::map<std::string, size_t> & map)
{
  // Get total amount of free and used memory
  std::unordered_map<std::string, size_t> memInfo;
  size_t mem_total = 0;
  size_t mem_free = 0;
  size_t mem_shared = 0;
  size_t mem_available = 0;
  size_t slab_reclaimable = 0;
  size_t buffers = 0;
  size_t cached = 0;
  size_t swap_total = 0;
  size_t swap_free = 0;

  try {
    readMemInfo(memInfo);
  } catch (const std::exception & e) {
    return e.what();
  }

  try {
    mem_total = memInfo.at("MemTotal");
    mem_free = memInfo.at("MemFree");
    mem_shared = memInfo.at("Shmem");
    mem_available = memInfo.at("MemAvailable");
    slab_reclaimable = memInfo.at("SReclaimable");
    buffers = memInfo.at("Buffers");
    cached = memInfo.at("Cached");
    swap_total = memInfo.at("SwapTotal");
    swap_free = memInfo.at("SwapFree");
  } catch (const std::out_of_range & e) {
    return e.what();
  }

  if (mem_total == 0) {
    return "Usage calculate error: mem_info is zero";
  }

  float usage = 1.0f - static_cast<double>(mem_available) / mem_total;
  size_t mem_buff_and_cache = buffers + cached + slab_reclaimable;
  size_t mem_used = mem_total - mem_free - mem_buff_and_cache;
  map["Mem: usage"] = usage * 1e+2;
  map["Mem: total"] = mem_total;
  map["Mem: used"] = mem_used;
  map["Mem: free"] = mem_free;
  map["Mem: shared"] = mem_shared;
  map["Mem: buff/cache"] = mem_buff_and_cache;
  map["Mem: available"] = mem_available;

  size_t swap_used = swap_total - swap_free;
  map["Swap: total"] = swap_total;
  map["Swap: used"] = swap_used;
  map["Swap: free"] = swap_free;

  size_t total_total = mem_total + swap_total;
  size_t total_used = mem_used + swap_used;
  size_t total_free = mem_free + swap_free;
  size_t used_plus = total_used + mem_shared;
  map["Total: total"] = total_total;
  map["Total: used"] = total_used;
  map["Total: free"] = total_free;
  map["Total: used+"] = used_plus;

  return "";
}

std::string MemMonitor::executeEdacUtil(std::string & output, std::string & pipe2_error_str)
{
  std::string result = "";
  std::ostringstream os;

  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    pipe2_error_str = std::string(strerror(errno));
    return result;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    pipe2_error_str = std::string(strerror(errno));
    return result;
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  bp::child c("edac-util --quiet", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();
  if (c.exit_code() != 0) {
    is_err >> os.rdbuf();
    result = os.str().c_str();
    return result;
  }
  is_out >> os.rdbuf();
  output = os.str().c_str();
  return result;
}

void MemMonitor::onUsageTimer()
{
  // Start to measure elapsed time
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("execution_time");

  std::string error_str;
  std::map<std::string, size_t> map;

  // Start timeout timer for executing readUsage
  {
    std::lock_guard<std::mutex> lock(usage_timeout_mutex_);
    usage_timeout_expired_ = false;
  }
  usage_timeout_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::seconds(usage_timeout_),
    std::bind(&MemMonitor::onUsageTimeout, this));

  error_str = readUsage(map);

  // Returning from readUsage, stop timeout timer
  usage_timeout_timer_->cancel();

  const double elapsed_ms = stop_watch.toc("execution_time");

  // thread-safe copy
  {
    std::lock_guard<std::mutex> lock(usage_mutex_);
    usage_error_str_ = error_str;
    usage_map_ = map;
    usage_elapsed_ms_ = elapsed_ms;
  }
}

void MemMonitor::onUsageTimeout()
{
  RCLCPP_WARN(get_logger(), "Read Memory Usage Timeout occurred.");
  std::lock_guard<std::mutex> lock(usage_timeout_mutex_);
  usage_timeout_expired_ = true;
}

void MemMonitor::onEccTimer()
{
  // Start to measure elapsed time
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("execution_time");

  std::string error_str;
  std::string pipe2_error_str;
  std::string output;

  // Start timeout timer for executing edac-util command
  {
    std::lock_guard<std::mutex> lock(ecc_timeout_mutex_);
    ecc_timeout_expired_ = false;
  }
  ecc_timeout_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::seconds(ecc_timeout_),
    std::bind(&MemMonitor::onEccTimeout, this));

  error_str = executeEdacUtil(output, pipe2_error_str);

  // Returning from edac-util command, stop timeout timer
  ecc_timeout_timer_->cancel();

  const double elapsed_ms = stop_watch.toc("execution_time");

  // thread-safe copy
  {
    std::lock_guard<std::mutex> lock(ecc_mutex_);
    ecc_error_str_ = error_str;
    ecc_pipe2_error_str_ = pipe2_error_str;
    ecc_output_ = output;
    ecc_elapsed_ms_ = elapsed_ms;
  }
}

void MemMonitor::onEccTimeout()
{
  RCLCPP_WARN(get_logger(), "Execute edac-util Timeout occurred.");
  std::lock_guard<std::mutex> lock(ecc_timeout_mutex_);
  ecc_timeout_expired_ = true;
}

std::string MemMonitor::toHumanReadable(const std::string & str)
{
  const char * units[] = {"B", "K", "M", "G", "T"};
  int count = 0;
  double size = std::atol(str.c_str());

  while (size > 1024) {
    size /= 1024;
    ++count;
  }
  const char * format = (count >= 3 || (size > 0 && size < 10)) ? "{:.1f}{}" : "{:.0f}{}";
  return fmt::format(format, size, units[count]);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MemMonitor)
