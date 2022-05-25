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
 * @file _cpu_monitor.cpp
 * @brief  CPU monitor class
 */

#include "system_monitor/ecu_monitor/adlink_ecu_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <msr_reader/msr_reader.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <regex>
#include <string>
#include <vector>
#include <regex>

namespace bp = boost::process;

ECUMonitor::ECUMonitor(const rclcpp::NodeOptions & options) : ECUMonitorBase("ecu_monitor", options)
{
  voltage_warn_ = declare_parameter<float>("low cmos battery warn", 2.9);
  voltage_error_ = declare_parameter<float>("low cmos battery error", 2.7);

  updater_.add("ECU CMOS Battery Voltage", this, &ECUMonitor::checkVoltage);
  gethostname(hostname_, sizeof(hostname_));
  // Check if command exists
  fs::path p = bp::search_path("sensors");
  sensors_exists_ = (p.empty()) ? false : true;
}

static float getVoltage() {
    bp::ipstream is_out;
    bp::ipstream is_err;
    fs::path p = bp::search_path("sensors");
    bp::child c(p.string(), bp::std_out > is_out, bp::std_err > is_err);
    c.wait();

    if(RCUTILS_UNLIKELY(c.exit_code() != 0)) {//failed to execute sensors 
      return 0;
    }
    std::string line;
    std::regex re(R"((\d+).(\d+))"); //in7:             3.06 V  (min =  +0.00 V, max =  +4.08 V)
    for(int i = 0; i < 200 && std::getline(is_out, line); i++) {
        auto voltageStringPos = line.find("in7:");
        if( voltageStringPos != std::string::npos) {
            std::smatch match;
            std::regex_search(line, match, re);
            auto voltageString = match.str(); 
            return std::stof(voltageString);
        }
    }
    return 0;//failed to read voltage
}

void ECUMonitor::checkVoltage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if ( RCUTILS_UNLIKELY(!sensors_exists_) ) {
    stat.summary(DiagStatus::ERROR, "sensors error");
    stat.add(
      "sensors", "Command 'sensors' not found, but can be installed with: 'sudo apt install lm-sensors' and 'sudo sensors-detect'");
    return;
  }

  auto v = getVoltage();

  stat.add("CMOS battey voltage", fmt::format("{}",v));
  if( RCUTILS_UNLIKELY(v < voltage_warn_) ) {
    stat.summary(DiagStatus::WARN, "LOW BATTERY");
  } else {
    stat.summary(DiagStatus::OK, "OK");
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ECUMonitor)
