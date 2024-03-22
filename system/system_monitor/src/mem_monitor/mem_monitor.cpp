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

#include <boost/process.hpp>

#include <fmt/format.h>

#include <string>
#include <vector>

namespace bp = boost::process;

MemMonitor::MemMonitor(const rclcpp::NodeOptions & options)
: Node("mem_monitor", options),
  updater_(this),
  available_size_(declare_parameter<int>("available_size", 1024) * 1024 * 1024)
{
  gethostname(hostname_, sizeof(hostname_));
  updater_.setHardwareID(hostname_);
  updater_.add("Memory Usage", this, &MemMonitor::checkUsage);

  // Enable ECC error detection if edac-utils package is installed
  if (!bp::search_path("edac-util").empty()) {
    updater_.add("Memory ECC", this, &MemMonitor::checkEcc);
  }
}

void MemMonitor::update()
{
  updater_.force_update();
}

std::unordered_map<std::string, size_t> readMemInfo()
{
  std::unordered_map<std::string, size_t> memInfo;
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

  return memInfo;
}

void MemMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

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
    memInfo = readMemInfo();
  } catch (const std::exception & e) {
    stat.summary(DiagStatus::ERROR, e.what());
    stat.add("read file error", "Error opening /proc/meminfo or parsing line in it.");
    return;
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
    stat.summary(DiagStatus::ERROR, e.what());
    stat.add("unordered_map::at", "Error reading a key of memory info");
    return;
  }

  float usage = 1.0f - static_cast<double>(mem_available) / mem_total;
  size_t mem_buff_and_cache = buffers + cached + slab_reclaimable;
  size_t mem_used = mem_total - mem_free - mem_buff_and_cache;
  stat.addf("Mem: usage", "%.2f%%", usage * 1e+2);
  stat.add("Mem: total", toHumanReadable(std::to_string(mem_total)));
  stat.add("Mem: used", toHumanReadable(std::to_string(mem_used)));
  stat.add("Mem: free", toHumanReadable(std::to_string(mem_free)));
  stat.add("Mem: shared", toHumanReadable(std::to_string(mem_shared)));
  stat.add("Mem: buff/cache", toHumanReadable(std::to_string(mem_buff_and_cache)));
  stat.add("Mem: available", toHumanReadable(std::to_string(mem_available)));

  size_t swap_used = swap_total - swap_free;
  stat.add("Swap: total", toHumanReadable(std::to_string(swap_total)));
  stat.add("Swap: used", toHumanReadable(std::to_string(swap_used)));
  stat.add("Swap: free", toHumanReadable(std::to_string(swap_free)));

  size_t total_total = mem_total + swap_total;
  size_t total_used = mem_used + swap_used;
  size_t total_free = mem_free + swap_free;
  size_t used_plus = total_used + mem_shared;
  stat.add("Total: total", toHumanReadable(std::to_string(total_total)));
  stat.add("Total: used", toHumanReadable(std::to_string(total_used)));
  stat.add("Total: free", toHumanReadable(std::to_string(total_free)));
  stat.add("Total: used+", toHumanReadable(std::to_string(used_plus)));

  int level;
  if (mem_total > used_plus) {
    level = DiagStatus::OK;
  } else if (mem_available >= available_size_) {
    level = DiagStatus::WARN;
  } else {
    level = DiagStatus::ERROR;
  }

  stat.summary(level, usage_dict_.at(level));

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void MemMonitor::checkEcc(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", strerror(errno));
    return;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", strerror(errno));
    return;
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  bp::child c("edac-util --quiet", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();
  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    stat.summary(DiagStatus::ERROR, "edac-util error");
    stat.add("edac-util", os.str().c_str());
    return;
  }

  std::string line;

  /*
   Output example of `edac-util --quiet`
   edac-util generates output if error occurred, otherwise no output
   mc0: 3 Uncorrected Errors with no DIMM info
   mc0: 3 Corrected Errors with no DIMM info
   */
  while (std::getline(is_out, line)) {
    if (line.find("Uncorrected") != std::string::npos) {
      stat.summary(DiagStatus::ERROR, line);
      return;
    } else if (line.find("Corrected") != std::string::npos) {
      stat.summary(DiagStatus::WARN, line);
      return;
    }
  }

  stat.summary(DiagStatus::OK, "OK");
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
