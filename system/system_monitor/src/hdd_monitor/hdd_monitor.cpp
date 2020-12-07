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
 * @file hdd_monitor.cpp
 * @brief HDD monitor class
 */

#include "hdd_reader/hdd_reader.hpp"
#include "system_monitor/hdd_monitor/hdd_monitor.hpp"
#include <algorithm>
#include "boost/algorithm/string.hpp"
#include "boost/archive/text_iarchive.hpp"
#include "boost/format.hpp"
#include "boost/process.hpp"
#include <string>
#include <vector>

namespace bp = boost::process;

HDDMonitor::HDDMonitor(const std::string & node_name, const rclcpp::NodeOptions & options) :
Node(node_name, options),
updater_(this),
usage_warn_(declare_parameter<float>("usage_warn", 0.9)),
usage_error_(declare_parameter<float>("usage_error", 1.1)),
hdd_reader_port_(declare_parameter<int>("hdd_reader_port", 7635))
{
  gethostname(hostname_, sizeof(hostname_));

  getTempParams();

  updater_.setHardwareID(hostname_);
  updater_.add("HDD Temperature", this, &HDDMonitor::checkTemp);
  updater_.add("HDD Usage", this, &HDDMonitor::checkUsage);
}

void HDDMonitor::update()
{
    updater_.force_update();
}

void HDDMonitor::checkTemp(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (temp_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  // Create a new socket
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    stat.summary(DiagStatus::ERROR, "socket error");
    stat.add("socket", strerror(errno));
    return;
  }

  // Specify the receiving timeouts until reporting an error
  struct timeval tv;
  tv.tv_sec = 10;
  tv.tv_usec = 0;
  int ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "setsockopt error");
    stat.add("setsockopt", strerror(errno));
    close(sock);
    return;
  }

  // Connect the socket referred to by the file descriptor
  sockaddr_in addr;
  memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(hdd_reader_port_);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  ret = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "connect error");
    stat.add("connect", strerror(errno));
    close(sock);
    return;
  }

  // Receive messages from a socket
  char buf[1024] = "";
  ret = recv(sock, buf, sizeof(buf) - 1, 0);
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "recv error");
    stat.add("recv", strerror(errno));
    close(sock);
    return;
  }
  // No data received
  if (ret == 0) {
    stat.summary(DiagStatus::ERROR, "recv error");
    stat.add("recv", "No data received");
    close(sock);
    return;
  }

  // Close the file descriptor FD
  ret = close(sock);
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "close error");
    stat.add("close", strerror(errno));
    return;
  }

  // Restore HDD information list
  HDDInfoList list;

  try {
    std::istringstream iss(buf);
    boost::archive::text_iarchive oa(iss);
    oa >> list;
  } catch (const std::exception & e) {
    stat.summary(DiagStatus::ERROR, "recv error");
    stat.add("recv", e.what());
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  std::string error_str = "";

  for (auto itr = temp_params_.begin(); itr != temp_params_.end(); ++itr, ++index) {
    // Retrieve HDD information
    auto itrh = list.find(itr->first);
    if (itrh == list.end()) {
      stat.add((boost::format("HDD %1%: status") % index).str(), "hdd_reader error");
      stat.add((boost::format("HDD %1%: name") % index).str(), itr->first.c_str());
      stat.add((boost::format("HDD %1%: hdd_reader") % index).str(), strerror(ENOENT));
      error_str = "hdd_reader error";
      continue;
    }

    if (itrh->second.error_code_ != 0) {
      stat.add((boost::format("HDD %1%: status") % index).str(), "hdd_reader error");
      stat.add((boost::format("HDD %1%: name") % index).str(), itr->first.c_str());
      stat.add(
        (boost::format("HDD %1%: hdd_reader") % index).str(), strerror(itrh->second.error_code_));
      error_str = "hdd_reader error";
      continue;
    }

    float temp = static_cast<float>(itrh->second.temp_);

    level = DiagStatus::OK;
    if (temp >= itr->second.temp_error_)
      level = DiagStatus::ERROR;
    else if (temp >= itr->second.temp_warn_)
      level = DiagStatus::WARN;

    stat.add((boost::format("HDD %1%: status") % index).str(), temp_dict_.at(level));
    stat.add((boost::format("HDD %1%: name") % index).str(), itr->first.c_str());
    stat.add((boost::format("HDD %1%: model") % index).str(), itrh->second.model_.c_str());
    stat.add((boost::format("HDD %1%: serial") % index).str(), itrh->second.serial_.c_str());
    stat.addf((boost::format("HDD %1%: temperature") % index).str(), "%.1f DegC", temp);

    whole_level = std::max(whole_level, level);
  }

  if (!error_str.empty())
    stat.summary(DiagStatus::ERROR, error_str);
  else
    stat.summary(whole_level, temp_dict_.at(whole_level));
}

void HDDMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Get summary of disk space usage of ext4
  bp::ipstream is_out;
  bp::ipstream is_err;
  bp::child c("df -Pht ext4", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();
  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    stat.summary(DiagStatus::ERROR, "df error");
    stat.add("df", os.str().c_str());
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;

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

    usage = std::atof(boost::trim_copy_if(list[4], boost::is_any_of("%")).c_str()) * 1e-2;

    level = DiagStatus::OK;
    if (usage >= usage_error_)
      level = DiagStatus::ERROR;
    else if (usage >= usage_warn_)
      level = DiagStatus::WARN;

    stat.add((boost::format("HDD %1%: status") % (index - 1)).str(), usage_dict_.at(level));
    stat.add((boost::format("HDD %1%: filesystem") % (index - 1)).str(), list[0].c_str());
    stat.add((boost::format("HDD %1%: size") % (index - 1)).str(), list[1].c_str());
    stat.add((boost::format("HDD %1%: used") % (index - 1)).str(), list[2].c_str());
    stat.add((boost::format("HDD %1%: avail") % (index - 1)).str(), list[3].c_str());
    stat.add((boost::format("HDD %1%: use") % (index - 1)).str(), list[4].c_str());
    stat.add((boost::format("HDD %1%: mounted on") % (index - 1)).str(), list[5].c_str());

    whole_level = std::max(whole_level, level);
    ++index;
  }

  stat.summary(whole_level, usage_dict_.at(whole_level));
}

void HDDMonitor::getTempParams(void)
{
    const auto num_disks = this->declare_parameter("num_disks", 0);
    for(auto i = 0; i < num_disks; ++i){
        const auto prefix = "disks.disk" + std::to_string(i);
        TempParam param;
        param.temp_warn_ = declare_parameter(prefix + ".temp_warn").get<float>();
        param.temp_error_ = declare_parameter(prefix + ".temp_error").get<float>();
        const auto name = declare_parameter(prefix + ".name").get<std::string>();
        temp_params_[name] = param;
    }
}
