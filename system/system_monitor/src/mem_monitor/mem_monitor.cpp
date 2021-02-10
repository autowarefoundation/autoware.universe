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

#include <string>
#include <vector>

#include "boost/process.hpp"

#include "fmt/format.h"

#include "system_monitor/mem_monitor/mem_monitor.hpp"

namespace bp = boost::process;

MemMonitor::MemMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  updater_(this),
  usage_warn_(declare_parameter<float>("usage_warn", 0.95)),
  usage_error_(declare_parameter<float>("usage_error", 0.99))
{
  gethostname(hostname_, sizeof(hostname_));
  updater_.setHardwareID(hostname_);
  updater_.add("Memory Usage", this, &MemMonitor::checkUsage);
}

void MemMonitor::update()
{
  updater_.force_update();
}

void MemMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Get total amount of free and used memory
  bp::ipstream is_out;
  bp::ipstream is_err;
  bp::child c("free -tb", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();
  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    stat.summary(DiagStatus::ERROR, "free error");
    stat.add("free", os.str().c_str());
    return;
  }

  int level = DiagStatus::OK;
  std::string line;
  int index = 0;
  std::vector<std::string> list;
  float usage;

  while (std::getline(is_out, line) && !line.empty()) {
    // Skip header
    if (index <= 0) {
      ++index;
      continue;
    }

    boost::split(list, line, boost::is_space(), boost::token_compress_on);

    // Physical memory
    if (index == 1) {
      // used divided by total is usage
      usage = std::atof(list[2].c_str()) / std::atof(list[1].c_str());

      if (usage >= usage_error_) {
        level = DiagStatus::ERROR;
      } else if (usage >= usage_warn_) {
        level = DiagStatus::WARN;
      }

      stat.addf(fmt::format("{} usage", list[0]), "%.2f%%", usage * 1e+2);
    }

    stat.add(fmt::format("{} total", list[0]), toHumanReadable(list[1]));
    stat.add(fmt::format("{} used", list[0]), toHumanReadable(list[2]));
    stat.add(fmt::format("{} free", list[0]), toHumanReadable(list[3]));

    ++index;
  }

  stat.summary(level, usage_dict_.at(level));
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
  const char * format = (size > 0 && size < 10) ? "{:.1f}{}" : "{:.0f}{}";
  return fmt::format(format, size, units[count]);
}
