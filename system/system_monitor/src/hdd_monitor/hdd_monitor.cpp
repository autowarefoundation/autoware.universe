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

#include "system_monitor/hdd_monitor/hdd_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace bp = boost::process;

HddMonitor::HddMonitor(const rclcpp::NodeOptions & options)
: Node("hdd_monitor", options),
  updater_(this),
  socket_path_(declare_parameter("socket_path", hdd_reader_service::socket_path)),
  last_hdd_stat_update_time_{0, 0, this->get_clock()->get_clock_type()}
{
  using namespace std::literals::chrono_literals;

  gethostname(hostname_, sizeof(hostname_));

  // Get HDD parameters
  get_hdd_params();

  // Update HDD connections
  update_hdd_connections();

  // Get HDD information from HDD reader for the first time
  update_hdd_info_list();

  updater_.setHardwareID(hostname_);
  updater_.add("HDD Temperature", this, &HddMonitor::check_smart_temperature);
  updater_.add("HDD PowerOnHours", this, &HddMonitor::check_smart_power_on_hours);
  updater_.add("HDD TotalDataWritten", this, &HddMonitor::check_smart_total_data_written);
  updater_.add("HDD RecoveredError", this, &HddMonitor::check_smart_recovered_error);
  updater_.add("HDD Usage", this, &HddMonitor::check_usage);
  updater_.add("HDD ReadDataRate", this, &HddMonitor::check_read_data_rate);
  updater_.add("HDD WriteDataRate", this, &HddMonitor::check_write_data_rate);
  updater_.add("HDD ReadIOPS", this, &HddMonitor::check_read_iops);
  updater_.add("HDD WriteIOPS", this, &HddMonitor::check_write_iops);
  updater_.add("HDD Connection", this, &HddMonitor::check_connection);

  // Start HDD transfer measurement
  start_hdd_transfer_measurement();

  timer_ = rclcpp::create_timer(this, get_clock(), 1s, std::bind(&HddMonitor::on_timer, this));
}

void HddMonitor::check_smart_temperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_smart(stat, HddSmartInfoItem::TEMPERATURE);
}

void HddMonitor::check_smart_power_on_hours(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_smart(stat, HddSmartInfoItem::POWER_ON_HOURS);
}

void HddMonitor::check_smart_total_data_written(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_smart(stat, HddSmartInfoItem::TOTAL_DATA_WRITTEN);
}

void HddMonitor::check_smart_recovered_error(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_smart(stat, HddSmartInfoItem::RECOVERED_ERROR);
}

void HddMonitor::check_smart(
  diagnostic_updater::DiagnosticStatusWrapper & stat, HddSmartInfoItem item)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  // Return error if connection diagnostic indicates error
  if (connect_diag_.level != DiagStatus::OK) {
    stat.summary(connect_diag_.level, connect_diag_.message);
    for (const auto & e : connect_diag_.values) {
      stat.add(e.key, e.value);
    }
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  std::string error_str;
  std::string key_str;
  std::string val_str;

  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr, ++index) {
    if (!hdd_connected_flags_[itr->first]) {
      continue;
    }

    // Retrieve HDD information
    auto hdd_itr = hdd_info_list_.find(itr->second.disk_device);
    if (hdd_itr == hdd_info_list_.end()) {
      stat.add(fmt::format("HDD {}: status", index), "hdd_reader error");
      stat.add(fmt::format("HDD {}: name", index), itr->second.part_device.c_str());
      stat.add(fmt::format("HDD {}: hdd_reader", index), strerror(ENOENT));
      error_str = "hdd_reader error";
      continue;
    }

    if (hdd_itr->second.error_code != 0) {
      stat.add(fmt::format("HDD {}: status", index), "hdd_reader error");
      stat.add(fmt::format("HDD {}: name", index), itr->second.part_device.c_str());
      stat.add(fmt::format("HDD {}: hdd_reader", index), strerror(hdd_itr->second.error_code));
      error_str = "hdd_reader error";
      continue;
    }

    switch (item) {
      case HddSmartInfoItem::TEMPERATURE: {
        auto temp = static_cast<float>(hdd_itr->second.temp);

        level = DiagStatus::OK;
        if (temp >= itr->second.temp_error) {
          level = DiagStatus::ERROR;
        } else if (temp >= itr->second.temp_warn) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: temperature", index);
        if (hdd_itr->second.is_valid_temp) {
          val_str = fmt::format("{:.1f} DegC", temp);
        } else {
          val_str = "not available";
        }
      } break;
      case HddSmartInfoItem::POWER_ON_HOURS: {
        auto power_on_hours = static_cast<int64_t>(hdd_itr->second.power_on_hours);

        level = DiagStatus::OK;
        if (power_on_hours >= itr->second.power_on_hours_warn) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: power on hours", index);
        if (hdd_itr->second.is_valid_power_on_hours) {
          val_str = fmt::format("{} Hours", hdd_itr->second.power_on_hours);
        } else {
          val_str = "not available";
        }
      } break;
      case HddSmartInfoItem::TOTAL_DATA_WRITTEN: {
        auto total_data_written = static_cast<uint64_t>(hdd_itr->second.total_data_written);

        level = DiagStatus::OK;
        if (total_data_written >= itr->second.total_data_written_warn) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: total data written", index);
        if (hdd_itr->second.is_valid_total_data_written) {
          val_str = fmt::format("{}", hdd_itr->second.total_data_written);
        } else {
          val_str = "not available";
        }
      } break;
      case HddSmartInfoItem::RECOVERED_ERROR: {
        auto recovered_error = static_cast<int32_t>(hdd_itr->second.recovered_error);
        if (initial_recovered_errors_.find(itr->first) == initial_recovered_errors_.end()) {
          initial_recovered_errors_[itr->first] = recovered_error;
        }
        recovered_error -= initial_recovered_errors_[itr->first];

        level = DiagStatus::OK;
        if (recovered_error >= itr->second.recovered_error_warn) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: recovered error", index);
        if (hdd_itr->second.is_valid_recovered_error) {
          val_str = fmt::format("{}", hdd_itr->second.recovered_error);
        } else {
          val_str = "not available";
        }
      } break;
      default:
        break;
    }

    stat.add(
      fmt::format("HDD {}: status", index), smart_dicts_[static_cast<uint32_t>(item)].at(level));
    stat.add(fmt::format("HDD {}: name", index), itr->second.disk_device.c_str());
    stat.add(fmt::format("HDD {}: model", index), hdd_itr->second.model.c_str());
    stat.add(fmt::format("HDD {}: serial", index), hdd_itr->second.serial.c_str());
    stat.addf(key_str, val_str.c_str());

    whole_level = std::max(whole_level, level);
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(whole_level, smart_dicts_[static_cast<uint32_t>(item)].at(whole_level));
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void HddMonitor::check_usage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  int hdd_index = 0;
  int whole_level = DiagStatus::OK;
  std::string error_str = "";

  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr, ++hdd_index) {
    if (!hdd_connected_flags_[itr->first]) {
      continue;
    }

    // boost::process create file descriptor without O_CLOEXEC required for multithreading.
    // So create file descriptor with O_CLOEXEC and pass it to boost::process.
    int out_fd[2];
    if (pipe2(out_fd, O_CLOEXEC) != 0) {
      error_str = "pipe2 error";
      stat.add(fmt::format("HDD {}: status", hdd_index), "pipe2 error");
      stat.add(fmt::format("HDD {}: name", hdd_index), itr->first.c_str());
      stat.add(fmt::format("HDD {}: pipe2", hdd_index), strerror(errno));
      continue;
    }
    bp::pipe out_pipe{out_fd[0], out_fd[1]};
    bp::ipstream is_out{std::move(out_pipe)};

    int err_fd[2];
    if (pipe2(err_fd, O_CLOEXEC) != 0) {
      error_str = "pipe2 error";
      stat.add(fmt::format("HDD {}: status", hdd_index), "pipe2 error");
      stat.add(fmt::format("HDD {}: name", hdd_index), itr->first.c_str());
      stat.add(fmt::format("HDD {}: pipe2", hdd_index), strerror(errno));
      continue;
    }
    bp::pipe err_pipe{err_fd[0], err_fd[1]};
    bp::ipstream is_err{std::move(err_pipe)};

    // Invoke shell to use shell wildcard expansion
    bp::child c(
      "/bin/sh", "-c", fmt::format("df -Pm {}*", itr->second.part_device.c_str()),
      bp::std_out > is_out, bp::std_err > is_err);
    c.wait();

    if (c.exit_code() != 0) {
      std::ostringstream os;
      is_err >> os.rdbuf();
      error_str = "df error";
      stat.add(fmt::format("HDD {}: status", hdd_index), "df error");
      stat.add(fmt::format("HDD {}: name", hdd_index), itr->second.part_device.c_str());
      stat.add(fmt::format("HDD {}: df", hdd_index), os.str().c_str());
      continue;
    }

    int level = DiagStatus::OK;
    std::string line;
    int index = 0;
    std::vector<std::string> list;
    int avail;

    while (std::getline(is_out, line) && !line.empty()) {
      // Skip header
      if (index <= 0) {
        ++index;
        continue;
      }

      boost::split(list, line, boost::is_space(), boost::token_compress_on);

      try {
        avail = std::stoi(list[3]);
      } catch (std::exception & e) {
        avail = -1;
        error_str = e.what();
        stat.add(fmt::format("HDD {}: status", hdd_index), "avail string error");
      }

      if (avail <= itr->second.free_error) {
        level = DiagStatus::ERROR;
      } else if (avail <= itr->second.free_warn) {
        level = DiagStatus::WARN;
      } else {
        level = DiagStatus::OK;
      }

      stat.add(fmt::format("HDD {}: status", hdd_index), usage_dict_.at(level));
      stat.add(fmt::format("HDD {}: filesystem", hdd_index), list[0].c_str());
      stat.add(fmt::format("HDD {}: size", hdd_index), (list[1] + " MiB").c_str());
      stat.add(fmt::format("HDD {}: used", hdd_index), (list[2] + " MiB").c_str());
      stat.add(fmt::format("HDD {}: avail", hdd_index), (list[3] + " MiB").c_str());
      stat.add(fmt::format("HDD {}: use", hdd_index), list[4].c_str());
      std::string mounted = list[5];
      if (list.size() > 6) {
        std::string::size_type pos = line.find("% /");
        if (pos != std::string::npos) {
          mounted = line.substr(pos + 2);  // 2 is "% " length
        }
      }
      stat.add(fmt::format("HDD {}: mounted on", hdd_index), mounted.c_str());

      whole_level = std::max(whole_level, level);
      ++index;
    }
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(whole_level, usage_dict_.at(whole_level));
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void HddMonitor::check_read_data_rate(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_statistics(stat, HddStatItem::READ_DATA_RATE);
}

void HddMonitor::check_write_data_rate(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_statistics(stat, HddStatItem::WRITE_DATA_RATE);
}

void HddMonitor::check_read_iops(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_statistics(stat, HddStatItem::READ_IOPS);
}

void HddMonitor::check_write_iops(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_statistics(stat, HddStatItem::WRITE_IOPS);
}

void HddMonitor::check_statistics(
  diagnostic_updater::DiagnosticStatusWrapper & stat, HddStatItem item)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  int hdd_index = 0;
  int whole_level = DiagStatus::OK;
  std::string error_str;
  std::string key_str;
  std::string val_str;

  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr, ++hdd_index) {
    if (!hdd_connected_flags_[itr->first]) {
      continue;
    }

    int level = DiagStatus::OK;

    switch (item) {
      case HddStatItem::READ_DATA_RATE: {
        float read_data_rate = hdd_stats_[itr->first].read_data_rate_MBs;

        if (read_data_rate >= itr->second.read_data_rate_warn) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: data rate of read", hdd_index);
        val_str = fmt::format("{:.2f} MB/s", read_data_rate);
      } break;
      case HddStatItem::WRITE_DATA_RATE: {
        float write_data_rate = hdd_stats_[itr->first].write_data_rate_MBs;

        if (write_data_rate >= itr->second.write_data_rate_warn) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: data rate of write", hdd_index);
        val_str = fmt::format("{:.2f} MB/s", write_data_rate);
      } break;
      case HddStatItem::READ_IOPS: {
        float read_iops = hdd_stats_[itr->first].read_iops;

        if (read_iops >= itr->second.read_iops_warn) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: IOPS of read", hdd_index);
        val_str = fmt::format("{:.2f} IOPS", read_iops);
      } break;
      case HddStatItem::WRITE_IOPS: {
        float write_iops = hdd_stats_[itr->first].write_iops;

        if (write_iops >= itr->second.write_iops_warn) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: IOPS of write", hdd_index);
        val_str = fmt::format("{:.2f} IOPS", write_iops);
      } break;
      default:
        break;
    }

    if (!hdd_stats_[itr->first].error_str.empty()) {
      error_str = hdd_stats_[itr->first].error_str;
      stat.add(fmt::format("HDD {}: status", hdd_index), error_str);
      stat.add(fmt::format("HDD {}: name", hdd_index), itr->second.disk_device.c_str());
    } else {
      stat.add(
        fmt::format("HDD {}: status", hdd_index),
        stat_dicts_[static_cast<uint32_t>(item)].at(level));
      stat.add(fmt::format("HDD {}: name", hdd_index), itr->second.disk_device.c_str());
      stat.add(key_str, val_str.c_str());
    }

    whole_level = std::max(whole_level, level);
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(whole_level, stat_dicts_[static_cast<uint32_t>(item)].at(whole_level));
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void HddMonitor::check_connection(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  int hdd_index = 0;
  int whole_level = DiagStatus::OK;

  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr, ++hdd_index) {
    int level = DiagStatus::OK;

    if (!hdd_connected_flags_[itr->first]) {
      level = DiagStatus::WARN;
    }

    stat.add(fmt::format("HDD {}: status", hdd_index), connection_dict_.at(level));
    stat.add(fmt::format("HDD {}: name", hdd_index), itr->second.disk_device);
    stat.add(fmt::format("HDD {}: mount point", hdd_index), itr->first.c_str());

    whole_level = std::max(whole_level, level);
  }

  stat.summary(whole_level, connection_dict_.at(whole_level));

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void HddMonitor::get_hdd_params()
{
  const auto num_disks = this->declare_parameter("num_disks", 0);
  for (auto i = 0; i < num_disks; ++i) {
    const auto prefix = "disks.disk" + std::to_string(i);
    const auto mount_point = declare_parameter<std::string>(prefix + ".name", "/");

    HddParam param;
    param.temp_warn = declare_parameter<float>(prefix + ".temp_warn", 55.0f);
    param.temp_error = declare_parameter<float>(prefix + ".temp_error", 70.0f);
    param.power_on_hours_warn = declare_parameter<int>(prefix + ".power_on_hours_warn", 3000000);
    param.total_data_written_safety_factor =
      declare_parameter<float>(prefix + ".total_data_written_safety_factor", 0.05f);
    int64_t total_data_written_warn_org =
      declare_parameter<int64_t>(prefix + ".total_data_written_warn", 4915200);
    param.total_data_written_warn = static_cast<uint64_t>(
      total_data_written_warn_org * (1.0f - param.total_data_written_safety_factor));
    param.recovered_error_warn = declare_parameter<int>(prefix + ".recovered_error_warn", 1);
    param.free_warn = declare_parameter<int>(prefix + ".free_warn", 5120);
    param.free_error = declare_parameter<int>(prefix + ".free_error", 100);
    param.read_data_rate_warn = declare_parameter<float>(prefix + ".read_data_rate_warn", 360.0);
    param.write_data_rate_warn = declare_parameter<float>(prefix + ".write_data_rate_warn", 103.5);
    param.read_iops_warn = declare_parameter<float>(prefix + ".read_iops_warn", 63360.0);
    param.write_iops_warn = declare_parameter<float>(prefix + ".write_iops_warn", 24120.0);
    param.temp_attribute_id =
      static_cast<uint8_t>(declare_parameter<int>(prefix + ".temp_attribute_id", 0xC2));
    param.power_on_hours_attribute_id =
      static_cast<uint8_t>(declare_parameter<int>(prefix + ".power_on_hours_attribute_id", 0x09));
    param.total_data_written_attribute_id = static_cast<uint8_t>(
      declare_parameter<int>(prefix + ".total_data_written_attribute_id", 0xF1));
    param.recovered_error_attribute_id =
      static_cast<uint8_t>(declare_parameter<int>(prefix + ".recovered_error_attribute_id", 0xC3));

    hdd_params_[mount_point] = param;

    HddStat stat;
    hdd_stats_[mount_point] = stat;
  }
}

std::string HddMonitor::get_device_from_mount_point(const std::string & mount_point)
{
  // boost::process create file descriptor without O_CLOEXEC required for multithreading.
  // So create file descriptor with O_CLOEXEC and pass it to boost::process.
  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to execute pipe2. %s", strerror(errno));
    return "";
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to execute pipe2. %s", strerror(errno));
    return "";
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  bp::child c(
    "/bin/sh", "-c", fmt::format("findmnt -n -o SOURCE {}", mount_point.c_str()),
    bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to execute findmnt. %s", mount_point.c_str());
    return "";
  }

  std::string line;
  if (!std::getline(is_out, line)) {
    RCLCPP_ERROR(get_logger(), "Failed to find device name. %s", mount_point.c_str());
    return "";
  }

  return line;
}

void HddMonitor::update_hdd_connections()
{
  for (auto & hdd_param : hdd_params_) {
    const auto mount_point = hdd_param.first;
    hdd_connected_flags_[mount_point] = false;

    // Get device name from mount point
    hdd_param.second.part_device = get_device_from_mount_point(mount_point);
    if (!hdd_param.second.part_device.empty()) {
      // Check the existence of device file
      std::error_code error_code;
      if (std::filesystem::exists(hdd_param.second.part_device, error_code)) {
        hdd_connected_flags_[mount_point] = true;

        // Remove index number of partition for passing device name to hdd-reader
        if (boost::starts_with(hdd_param.second.part_device, "/dev/sd")) {
          const std::regex pattern("\\d+$");
          hdd_param.second.disk_device =
            std::regex_replace(hdd_param.second.part_device, pattern, "");
        } else if (boost::starts_with(hdd_param.second.part_device, "/dev/nvme")) {
          const std::regex pattern("p\\d+$");
          hdd_param.second.disk_device =
            std::regex_replace(hdd_param.second.part_device, pattern, "");
        }

        const std::regex raw_pattern(".*/");
        hdd_stats_[mount_point].device =
          std::regex_replace(hdd_param.second.disk_device, raw_pattern, "");
      } else {
        // Deal with the issue that file system remains mounted when a drive is actually
        // disconnected.
        if (unmount_device(hdd_param.second.part_device)) {
          RCLCPP_ERROR(
            get_logger(), "Failed to unmount device : %s", hdd_param.second.part_device.c_str());
        }
      }
    }
  }
}

int HddMonitor::unmount_device(std::string & device)
{
  // Connect to hdd-reader service
  if (!connect_service()) {
    connect_diag_.summary(DiagStatus::ERROR, "connect error");
    close_connection();
    return -1;
  }

  // Send data to hdd-reader service
  if (!send_data(hdd_reader_service::Request::UNMOUNT_DEVICE, device)) {
    close_connection();
    connect_diag_.summary(DiagStatus::ERROR, "write error");
    return -1;
  }

  // Receive data from hdd-reader service
  int response = EXIT_SUCCESS;
  receive_data(response);

  // Close connection with hdd-reader service
  close_connection();

  return response;
}

void HddMonitor::on_timer()
{
  update_hdd_connections();
  update_hdd_info_list();
  update_hdd_statistics();
}

void HddMonitor::update_hdd_info_list()
{
  // Clear diagnostic information for connection
  connect_diag_.clear();
  connect_diag_.clearSummary();

  hdd_reader_service::AttributeIdParameterList parameters;
  for (const auto & hdd_param : hdd_params_) {
    // Skip if device is disconnected
    if (!hdd_connected_flags_[hdd_param.first]) {
      continue;
    }

    // Create parameters for hdd-reader service
    hdd_reader_service::AttributeIdParameter parameter{};
    parameter.temperature_id = hdd_param.second.temp_attribute_id;
    parameter.power_on_hours_id = hdd_param.second.power_on_hours_attribute_id;
    parameter.total_data_written_id = hdd_param.second.total_data_written_attribute_id;
    parameter.recovered_error_id = hdd_param.second.recovered_error_attribute_id;

    parameters[hdd_param.second.disk_device] = parameter;
  }

  // Connect to hdd-reader service
  if (!connect_service()) {
    connect_diag_.summary(DiagStatus::ERROR, "connect error");
    close_connection();
    return;
  }

  // Send data to hdd-reader service
  if (!send_data(hdd_reader_service::Request::GET_HDD_INFORMATION, parameters)) {
    close_connection();
    connect_diag_.summary(DiagStatus::ERROR, "write error");
    return;
  }

  // Receive data from hdd-reader service
  receive_data(hdd_info_list_);

  // Close connection with hdd-reader service
  close_connection();
}

void HddMonitor::start_hdd_transfer_measurement()
{
  for (auto & hdd_stat : hdd_stats_) {
    hdd_stat.second.error_str = "";

    if (!hdd_connected_flags_[hdd_stat.first]) {
      continue;
    }

    SysfsDevStat sysfs_dev_stat{};
    if (read_sysfs_device_stat(hdd_stat.second.device, sysfs_dev_stat)) {
      hdd_stat.second.error_str = "stat file read error";
      continue;
    }
    hdd_stat.second.last_sysfs_dev_stat = sysfs_dev_stat;
  }

  last_hdd_stat_update_time_ = this->now();
}

void HddMonitor::update_hdd_statistics()
{
  double duration_sec = (this->now() - last_hdd_stat_update_time_).seconds();

  for (auto & hdd_stat : hdd_stats_) {
    hdd_stat.second.error_str = "";

    if (!hdd_connected_flags_[hdd_stat.first]) {
      continue;
    }

    SysfsDevStat sysfs_dev_stat{};
    if (read_sysfs_device_stat(hdd_stat.second.device, sysfs_dev_stat)) {
      hdd_stat.second.error_str = "stat file read error";
      continue;
    }

    SysfsDevStat & last_sysfs_dev_stat = hdd_stat.second.last_sysfs_dev_stat;

    hdd_stat.second.read_data_rate_MBs = get_increase_sysfs_device_stat_value_per_sec(
      sysfs_dev_stat.rd_sectors, last_sysfs_dev_stat.rd_sectors, duration_sec);
    hdd_stat.second.read_data_rate_MBs /= 2048;
    hdd_stat.second.write_data_rate_MBs = get_increase_sysfs_device_stat_value_per_sec(
      sysfs_dev_stat.wr_sectors, last_sysfs_dev_stat.wr_sectors, duration_sec);
    hdd_stat.second.write_data_rate_MBs /= 2048;
    hdd_stat.second.read_iops = get_increase_sysfs_device_stat_value_per_sec(
      sysfs_dev_stat.rd_ios, last_sysfs_dev_stat.rd_ios, duration_sec);
    hdd_stat.second.write_iops = get_increase_sysfs_device_stat_value_per_sec(
      sysfs_dev_stat.wr_ios, last_sysfs_dev_stat.wr_ios, duration_sec);

    hdd_stat.second.last_sysfs_dev_stat = sysfs_dev_stat;
  }

  last_hdd_stat_update_time_ = this->now();
}

double HddMonitor::get_increase_sysfs_device_stat_value_per_sec(
  uint64_t cur_val, uint64_t last_val, double duration_sec)
{
  if (cur_val > last_val && duration_sec > 0.0) {
    return static_cast<double>(cur_val - last_val) / duration_sec;
  }
  return 0.0;
}

int HddMonitor::read_sysfs_device_stat(const std::string & device, SysfsDevStat & sysfs_dev_stat)
{
  int ret = -1;
  unsigned int ios_pgr, tot_ticks, rq_ticks, wr_ticks;
  uint64_t rd_ios, rd_merges_or_rd_sec, wr_ios, wr_merges;
  uint64_t rd_sec_or_wr_ios, wr_sec, rd_ticks_or_wr_sec;
  uint64_t dc_ios, dc_merges, dc_sec, dc_ticks;

  std::string filename("/sys/block/");
  filename += device + "/stat";
  FILE * fp = fopen(filename.c_str(), "r");
  if (fp == nullptr) {
    return ret;
  }

  int i = fscanf(
    fp, "%lu %lu %lu %lu %lu %lu %lu %u %u %u %u %lu %lu %lu %lu", &rd_ios, &rd_merges_or_rd_sec,
    &rd_sec_or_wr_ios, &rd_ticks_or_wr_sec, &wr_ios, &wr_merges, &wr_sec, &wr_ticks, &ios_pgr,
    &tot_ticks, &rq_ticks, &dc_ios, &dc_merges, &dc_sec, &dc_ticks);

  if (i >= 7) {
    sysfs_dev_stat.rd_ios = rd_ios;
    sysfs_dev_stat.rd_sectors = rd_sec_or_wr_ios;
    sysfs_dev_stat.wr_ios = wr_ios;
    sysfs_dev_stat.wr_sectors = wr_sec;
    ret = 0;
  }

  fclose(fp);
  return ret;
}

bool HddMonitor::connect_service()
{
  local::stream_protocol::endpoint endpoint(socket_path_);
  socket_ = std::make_unique<local::stream_protocol::socket>(io_service_);

  // Connect socket
  boost::system::error_code error_code;
  socket_->connect(endpoint, error_code);

  if (error_code) {
    RCLCPP_ERROR(get_logger(), "Failed to connect socket. %s", error_code.message().c_str());
    return false;
  }

  return true;
}

template <class T>
bool HddMonitor::send_data(hdd_reader_service::Request request, T & parameter)
{
  std::ostringstream out_stream;
  boost::archive::text_oarchive archive(out_stream);
  archive & request;
  archive & parameter;

  // Write data to socket
  boost::system::error_code error_code;
  socket_->write_some(
    boost::asio::buffer(out_stream.str().c_str(), out_stream.str().length()), error_code);

  if (error_code) {
    RCLCPP_ERROR(get_logger(), "Failed to write data to socket. %s", error_code.message().c_str());
    return false;
  }

  return true;
}

template <class T>
void HddMonitor::receive_data(T & received)
{
  uint8_t request_id = hdd_reader_service::Request::NONE;

  // Read data from socket
  char buffer[10240]{};
  boost::system::error_code error_code;
  socket_->read_some(boost::asio::buffer(buffer, sizeof(buffer)), error_code);

  if (error_code) {
    RCLCPP_ERROR(
      get_logger(), "Failed to read data from socket. %s\n", error_code.message().c_str());
    return;
  }

  // Restore device status list
  try {
    std::istringstream in_stream(buffer);
    boost::archive::text_iarchive archive(in_stream);
    archive >> request_id;
    archive >> received;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to restore message. %s\n", e.what());
  }
}

void HddMonitor::close_connection()
{
  // Close socket
  socket_->close();
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(HddMonitor)
