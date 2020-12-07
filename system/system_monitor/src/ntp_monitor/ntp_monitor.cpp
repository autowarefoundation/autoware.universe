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
 * @file ntp_monitor.cpp
 * @brief NTP monitor class
 */

#include "system_monitor/ntp_monitor/ntp_monitor.hpp"
#include "boost/filesystem.hpp"
#include "boost/format.hpp"
#include "boost/process.hpp"
#include "boost/regex.hpp"
#include <string>

namespace bp = boost::process;
namespace fs = boost::filesystem;

NTPMonitor::NTPMonitor(const std::string & node_name, const rclcpp::NodeOptions & options) :
Node(node_name, options),
updater_(this),
server_(declare_parameter<std::string>("server", "ntp.ubuntu.com")),
offset_warn_(declare_parameter<float>("offset_warn", 0.1)),
offset_error_(declare_parameter<float>("offset_error", 5.0))
{
  gethostname(hostname_, sizeof(hostname_));

  // Check if command exists
  fs::path p = bp::search_path("ntpdate");
  ntpdate_exists_ = (p.empty()) ? false : true;

  updater_.setHardwareID(hostname_);
  updater_.add("NTP Offset", this, &NTPMonitor::checkOffset);
}

void NTPMonitor::update()
{
    updater_.force_update();
}

void NTPMonitor::checkOffset(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!ntpdate_exists_) {
    stat.summary(DiagStatus::ERROR, "ntpdate error");
    stat.add(
      "ntpdate",
      "Command 'ntpdate' not found, but can be installed with: sudo apt install ntpdate");
    return;
  }

  int level = DiagStatus::OK;

  // Query NTP server
  bp::ipstream is_out;
  bp::ipstream is_err;
  bp::child c(
    (boost::format("ntpdate -q %1%") % server_).str(), bp::std_out > is_out, bp::std_err > is_err);
  c.wait();
  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    stat.summary(DiagStatus::ERROR, "ntpdate error");
    stat.add("ntpdate", os.str().c_str());
    return;
  }

  std::string line;
  float offset = 0.0f;
  float delay = 0.0f;
  boost::smatch match;
  const boost::regex filter("^server.*offset ([-+]?\\d+\\.\\d+), delay ([-+]?\\d+\\.\\d+)");

  while (std::getline(is_out, line) && !line.empty()) {
    if (boost::regex_match(line, match, filter)) {
      float ofs = std::atof(match[1].str().c_str());
      float dly = std::atof(match[2].str().c_str());
      // Choose better network performance
      if (dly > delay) {
        offset = ofs;
        delay = dly;
      }
    }
  }

  // Check an earlier offset as well
  float abs = std::abs(offset);
  if (abs >= offset_error_)
    level = DiagStatus::ERROR;
  else if (abs >= offset_warn_)
    level = DiagStatus::WARN;

  stat.addf("NTP Offset", "%.6f sec", offset);
  stat.addf("NTP Delay", "%.6f sec", delay);
  stat.summary(level, offset_dict_.at(level));
}
