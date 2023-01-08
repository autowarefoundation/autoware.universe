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

#include <fmt/format.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

HddMonitor::HddMonitor(const rclcpp::NodeOptions & options)
: Node("hdd_monitor", options),
  updater_(this),
  hostname_(),
  socket_path_(declare_parameter("socket_path", hdd_reader_service::socket_path)),
  last_hdd_stat_update_time_{0, 0, this->get_clock()->get_clock_type()}
{
  using namespace std::literals::chrono_literals;

  gethostname(hostname_, sizeof(hostname_));

  // Get HDD parameters
  get_hdd_params();

  // Update HDD connections
  update_hdd_connections();

  // Get HDD information from HDD reader
  update_hdd_info_list();

  updater_.setHardwareID(hostname_);
  updater_.add("HDD Connection", this, &HddMonitor::check_connection);
  updater_.add("HDD Temperature", this, &HddMonitor::check_smart_temperature);
  updater_.add("HDD PowerOnHours", this, &HddMonitor::check_smart_power_on_hours);
  updater_.add("HDD TotalDataWritten", this, &HddMonitor::check_smart_total_data_written);
  updater_.add("HDD RecoveredError", this, &HddMonitor::check_smart_recovered_error);
  updater_.add("HDD Usage", this, &HddMonitor::check_usage);
  updater_.add("HDD ReadDataRate", this, &HddMonitor::check_read_data_rate);
  updater_.add("HDD WriteDataRate", this, &HddMonitor::check_write_data_rate);
  updater_.add("HDD ReadIOPS", this, &HddMonitor::check_read_iops);
  updater_.add("HDD WriteIOPS", this, &HddMonitor::check_write_iops);

  // Start HDD transfer measurement
  start_hdd_transfer_measurement();

  timer_ = rclcpp::create_timer(this, get_clock(), 1s, std::bind(&HddMonitor::on_timer, this));
}

void HddMonitor::check_connection(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  // Error occurs when communicating with hdd-reader service
  if (communication_diag_.level != DiagStatus::OK) {
    stat.summary(communication_diag_.level, communication_diag_.message);
    for (const auto & item : communication_diag_.values) {
      stat.add(item.key, item.value);
    }
    return;
  }

  int index = 0;
  int whole_level = DiagStatus::OK;
  std::string error_message;

  // Loop in number of mount points
  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr, ++index) {
    int level = DiagStatus::OK;

    // Set warning if device is disconnected
    if (!hdd_connected_flags_[itr->first]) {
      level = DiagStatus::WARN;
    }

    // Retrieve HDD information
    auto hdd_itr = hdd_info_list_.find(itr->second.disk_device);

    // Something happens before opening device
    if (hdd_itr == hdd_info_list_.end()) {
      stat.add(fmt::format("HDD {}: status", index), "hdd_reader error");
      stat.add(fmt::format("HDD {}: name", index), itr->second.part_device.c_str());
      stat.add(fmt::format("HDD {}: hdd_reader", index), strerror(ENOENT));
      error_message = "hdd_reader error";
      continue;
    }
    // Something happens after opening device
    if (hdd_itr->second.error_code != 0) {
      stat.add(fmt::format("HDD {}: status", index), "hdd_reader error");
      stat.add(fmt::format("HDD {}: name", index), itr->second.part_device.c_str());
      stat.add(fmt::format("HDD {}: hdd_reader", index), strerror(hdd_itr->second.error_code));
      error_message = "hdd_reader error";
      continue;
    }

    stat.add(fmt::format("HDD {}: status", index), connection_dict_.at(level));
    stat.add(fmt::format("HDD {}: mount point", index), itr->first.c_str());
    stat.add(fmt::format("HDD {}: name", index), itr->second.disk_device);
    stat.add(fmt::format("HDD {}: model", index), hdd_itr->second.model.c_str());
    stat.add(fmt::format("HDD {}: serial", index), hdd_itr->second.serial.c_str());

    whole_level = std::max(whole_level, level);
  }

  if (!error_message.empty()) {
    stat.summary(DiagStatus::ERROR, error_message);
  } else {
    stat.summary(whole_level, connection_dict_.at(whole_level));
  }
}

void HddMonitor::check_smart_temperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_smart(stat, CheckType::TEMPERATURE);
}

void HddMonitor::check_smart_power_on_hours(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_smart(stat, CheckType::POWER_ON_HOURS);
}

void HddMonitor::check_smart_total_data_written(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_smart(stat, CheckType::TOTAL_DATA_WRITTEN);
}

void HddMonitor::check_smart_recovered_error(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_smart(stat, CheckType::RECOVERED_ERROR);
}

void HddMonitor::check_smart(diagnostic_updater::DiagnosticStatusWrapper & stat, CheckType type)
{
  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  int whole_level = DiagStatus::OK;
  int index = 0;

  // Loop in number of mount points
  for (const auto & hdd_param : hdd_params_) {
    // Skip if device is disconnected
    if (!hdd_connected_flags_[hdd_param.first]) continue;

    const auto & param = hdd_param.second;

    // Get HDD information
    hdd_reader_service::HddInformation info;
    // Skip if error occurs
    if (!get_hdd_information(param.disk_device, info)) continue;

    std::string key;
    std::string value;
    int level = DiagStatus::OK;

    switch (type) {
      case CheckType::TEMPERATURE:
        level = set_smart_temperature(param, info, index, key, value);
        break;
      case CheckType::POWER_ON_HOURS:
        level = set_smart_power_on_hours(param, info, index, key, value);
        break;
      case CheckType::TOTAL_DATA_WRITTEN:
        level = set_smart_total_data_written(param, info, index, key, value);
        break;
      case CheckType::RECOVERED_ERROR:
        level = set_smart_recovered_error(param, info, index, key, value);
        break;
      default:
        break;
    }

    stat.add(fmt::format("HDD {}: status", index), hdd_dicts_[static_cast<int>(type)].at(level));
    stat.add(fmt::format("HDD {}: name", index), param.disk_device.c_str());
    stat.add(key, value.c_str());

    whole_level = std::max(whole_level, level);
    ++index;
  }

  stat.summary(whole_level, hdd_dicts_[static_cast<int>(type)].at(whole_level));
}

int HddMonitor::set_smart_temperature(
  const HddParam & param, const hdd_reader_service::HddInformation & info, int index,
  std::string & key, std::string & value)
{
  int level = DiagStatus::OK;
  key = fmt::format("HDD {}: temperature", index);

  // Temperature not supported
  if (!info.is_valid_temp) {
    value = "not available";
    return level;
  }

  auto temp = static_cast<float>(info.temp);
  if (temp >= param.temp_error) {
    level = DiagStatus::ERROR;
  } else if (temp >= param.temp_warn) {
    level = DiagStatus::WARN;
  }

  value = fmt::format("{:.1f} DegC", temp);
  return level;
}

int HddMonitor::set_smart_power_on_hours(
  const HddParam & param, const hdd_reader_service::HddInformation & info, int index,
  std::string & key, std::string & value)
{
  int level = DiagStatus::OK;
  key = fmt::format("HDD {}: power on hours", index);

  // Power On Hours not supported
  if (!info.is_valid_power_on_hours) {
    value = "not available";
    return level;
  }

  if (info.power_on_hours >= param.power_on_hours_warn) {
    level = DiagStatus::WARN;
  }

  value = fmt::format("{} Hours", info.power_on_hours);
  return level;
}

int HddMonitor::set_smart_total_data_written(
  const HddParam & param, const hdd_reader_service::HddInformation & info, int index,
  std::string & key, std::string & value)
{
  int level = DiagStatus::OK;
  key = fmt::format("HDD {}: total data written", index);

  // Total LBAs Written not supported
  if (!info.is_valid_total_data_written) {
    value = "not available";
    return level;
  }

  if (info.total_data_written >= param.total_data_written_warn) {
    level = DiagStatus::WARN;
  }

  value = fmt::format("{}", info.total_data_written);
  return level;
}

int HddMonitor::set_smart_recovered_error(
  const HddParam & param, const hdd_reader_service::HddInformation & info, int index,
  std::string & key, std::string & value)
{
  int level = DiagStatus::OK;
  key = fmt::format("HDD {}: recovered error", index);

  // Recovered Error not supported
  if (!info.is_valid_recovered_error) {
    value = "not available";
    return level;
  }

  if (info.recovered_error >= param.recovered_error_warn) {
    level = DiagStatus::WARN;
  }

  value = fmt::format("{}", info.recovered_error);
  return level;
}

void HddMonitor::check_usage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  int index = 0;
  int whole_level = DiagStatus::OK;
  std::string error_message;

  // Loop in number of mount points
  for (const auto & hdd_param : hdd_params_) {
    // Skip if device is disconnected
    if (!hdd_connected_flags_[hdd_param.first]) {
      continue;
    }

    HddParam param = hdd_param.second;

    bp::ipstream is_out;
    int ret = run_disk_free_command(param.part_device, is_out, error_message);
    if (ret != EXIT_SUCCESS) {
      stat.add(fmt::format("HDD {}: status", index), error_message);
      stat.add(fmt::format("HDD {}: name", index), hdd_param.first.c_str());
      stat.add(fmt::format("HDD {}: message", index), strerror(ret));
      ++index;
      continue;
    }

    int level = DiagStatus::OK;
    std::string line;
    std::vector<std::string> list;
    uint32_t avail = 0;

    // Skip header
    std::getline(is_out, line);

    while (std::getline(is_out, line) && !line.empty()) {
      boost::split(list, line, boost::is_space(), boost::token_compress_on);

      try {
        avail = std::stoi(list[3]);
      } catch (std::exception & e) {
        avail = -1;
        error_message = e.what();
        stat.add(fmt::format("HDD {}: status", index), "avail string error");
      }

      if (avail <= param.free_error) {
        level = DiagStatus::ERROR;
      } else if (avail <= param.free_warn) {
        level = DiagStatus::WARN;
      } else {
        level = DiagStatus::OK;
      }

      stat.add(fmt::format("HDD {}: status", index), usage_dict_.at(level));
      stat.add(fmt::format("HDD {}: mount point", index), hdd_param.first.c_str());
      stat.add(fmt::format("HDD {}: filesystem", index), list[0].c_str());
      stat.add(fmt::format("HDD {}: size", index), (list[1] + " MiB").c_str());
      stat.add(fmt::format("HDD {}: used", index), (list[2] + " MiB").c_str());
      stat.add(fmt::format("HDD {}: avail", index), (list[3] + " MiB").c_str());
      stat.add(fmt::format("HDD {}: use", index), list[4].c_str());

      whole_level = std::max(whole_level, level);
    }

    ++index;
  }

  if (!error_message.empty()) {
    stat.summary(DiagStatus::ERROR, error_message);
  } else {
    stat.summary(whole_level, usage_dict_.at(whole_level));
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

int HddMonitor::run_disk_free_command(
  const std::string & partition, bp::ipstream & is_out, std::string & error_message)
{
  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    error_message = "pipe2 error";
    return errno;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  is_out = std::move(out_pipe);

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    error_message = "pipe2 error";
    return errno;
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  bp::child c(
    "/bin/sh", "-c", fmt::format("df -Pm {}*", partition.c_str()), bp::std_out > is_out,
    bp::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    error_message = "df error";
    return c.exit_code();
  }

  return EXIT_SUCCESS;
}

void HddMonitor::check_read_data_rate(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_statistics(stat, CheckType::READ_DATA_RATE);
}

void HddMonitor::check_write_data_rate(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_statistics(stat, CheckType::WRITE_DATA_RATE);
}

void HddMonitor::check_read_iops(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_statistics(stat, CheckType::READ_IOPS);
}

void HddMonitor::check_write_iops(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  check_statistics(stat, CheckType::WRITE_IOPS);
}

void HddMonitor::check_statistics(
  diagnostic_updater::DiagnosticStatusWrapper & stat, CheckType type)
{
  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  int whole_level = DiagStatus::OK;
  int index = 0;
  std::string error_message;

  // Loop in number of mount points
  for (const auto & hdd_param : hdd_params_) {
    // Skip if device is disconnected
    if (!hdd_connected_flags_[hdd_param.first]) continue;

    const auto & param = hdd_param.second;

    HddStatistics statistics = hdd_stats_[hdd_param.first];

    // Error occurs
    if (!statistics.error_message.empty()) {
      stat.add(fmt::format("HDD {}: status", index), statistics.error_message);
      stat.add(fmt::format("HDD {}: name", index), param.disk_device.c_str());
      error_message = statistics.error_message;
      ++index;
      continue;
    }

    std::string key;
    std::string value;
    int level = DiagStatus::OK;

    switch (type) {
      case CheckType::READ_DATA_RATE:
        level = set_statistics_read_data_rate(param, statistics, index, key, value);
        break;
      case CheckType::WRITE_DATA_RATE:
        level = set_statistics_write_data_rate(param, statistics, index, key, value);
        break;
      case CheckType::READ_IOPS:
        level = set_statistics_read_iops(param, statistics, index, key, value);
        break;
      case CheckType::WRITE_IOPS:
        level = set_statistics_write_iops(param, statistics, index, key, value);
        break;
      default:
        break;
    }

    stat.add(fmt::format("HDD {}: status", index), hdd_dicts_[static_cast<int>(type)].at(level));
    stat.add(fmt::format("HDD {}: name", index), param.disk_device.c_str());
    stat.add(key, value.c_str());

    whole_level = std::max(whole_level, level);
    ++index;
  }

  if (!error_message.empty()) {
    stat.summary(DiagStatus::ERROR, error_message);
  } else {
    stat.summary(whole_level, hdd_dicts_[static_cast<int>(type)].at(whole_level));
  }
}

int HddMonitor::set_statistics_read_data_rate(
  const HddParam & param, const HddStatistics & stat, int index, std::string & key,
  std::string & value)
{
  int level = DiagStatus::OK;

  if (stat.read_data_rate_MBs >= param.read_data_rate_warn) {
    level = DiagStatus::WARN;
  }
  key = fmt::format("HDD {}: data rate of read", index);
  value = fmt::format("{:.2f} MB/s", stat.read_data_rate_MBs);
  return level;
}

int HddMonitor::set_statistics_write_data_rate(
  const HddParam & param, const HddStatistics & stat, int index, std::string & key,
  std::string & value)
{
  int level = DiagStatus::OK;

  if (stat.write_data_rate_MBs >= param.write_data_rate_warn) {
    level = DiagStatus::WARN;
  }
  key = fmt::format("HDD {}: data rate of write", index);
  value = fmt::format("{:.2f} MB/s", stat.write_data_rate_MBs);
  return level;
}

int HddMonitor::set_statistics_read_iops(
  const HddParam & param, const HddStatistics & stat, int index, std::string & key,
  std::string & value)
{
  int level = DiagStatus::OK;

  if (stat.read_iops >= param.read_iops_warn) {
    level = DiagStatus::WARN;
  }
  key = fmt::format("HDD {}: IOPS of read", index);
  value = fmt::format("{:.2f} IOPS", stat.read_iops);
  return level;
}

int HddMonitor::set_statistics_write_iops(
  const HddParam & param, const HddStatistics & stat, int index, std::string & key,
  std::string & value)
{
  int level = DiagStatus::OK;

  if (stat.write_iops >= param.write_iops_warn) {
    level = DiagStatus::WARN;
  }
  key = fmt::format("HDD {}: IOPS of write", index);
  value = fmt::format("{:.2f} IOPS", stat.write_iops);
  return level;
}

void HddMonitor::get_hdd_params()
{
  const auto num_disks = this->declare_parameter("num_disks", 0);
  for (auto i = 0; i < num_disks; ++i) {
    const auto prefix = "disks.disk" + std::to_string(i);
    const auto mount_point = declare_parameter<std::string>(prefix + ".name", "/");

    HddParam param;

    // Related to S.M.A.R.T
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

    // Usage
    param.free_warn = declare_parameter<int>(prefix + ".free_warn", 5120);
    param.free_error = declare_parameter<int>(prefix + ".free_error", 100);

    // Statistics
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

    HddStatistics stat;
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
    // Disconnection is within expectation, so we do not treat as an error
    RCLCPP_INFO(get_logger(), "Failed to execute findmnt. %s", mount_point.c_str());
    return "";
  }

  std::string line;
  if (!std::getline(is_out, line)) {
    RCLCPP_ERROR(get_logger(), "Failed to find device name. %s", mount_point.c_str());
    return "";
  }

  return line;
}

bool HddMonitor::get_hdd_information(
  const std::string & device, hdd_reader_service::HddInformation & info)
{
  // Retrieve HDD information
  auto hdd_itr = hdd_info_list_.find(device);
  if (hdd_itr == hdd_info_list_.end()) {
    return false;
  }

  if (hdd_itr->second.error_code != 0) {
    return false;
  }

  info = hdd_itr->second;
  return true;
}

void HddMonitor::update_hdd_connections()
{
  for (auto & hdd_param : hdd_params_) {
    const auto mount_point = hdd_param.first;
    hdd_connected_flags_[mount_point] = false;

    // Get device name from mount point
    hdd_param.second.part_device = get_device_from_mount_point(mount_point);
    // Device not found or error
    if (hdd_param.second.part_device.empty()) {
      return;
    }
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

int HddMonitor::unmount_device(std::string & device)
{
  // Connect to hdd-reader service
  if (!connect_service()) {
    close_connection();
    return -1;
  }

  // Send data to hdd-reader service
  if (!send_data(hdd_reader_service::Request::UNMOUNT_DEVICE, device)) {
    close_connection();
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
  // Update HDD connections
  update_hdd_connections();
  // Update HDD information list
  update_hdd_info_list();
  // Update HDD statistics
  update_hdd_statistics();
}

void HddMonitor::update_hdd_info_list()
{
  // Clear diagnostic status for communication errors
  communication_diag_.clear();
  communication_diag_.clearSummary();

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
    close_connection();
    return;
  }

  // Send data to hdd-reader service
  if (!send_data(hdd_reader_service::Request::GET_HDD_INFORMATION, parameters)) {
    close_connection();
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
    hdd_stat.second.error_message = "";

    // Skip if device is disconnected
    if (!hdd_connected_flags_[hdd_stat.first]) {
      continue;
    }

    DeviceStatistics statistics{};
    int ret = get_device_statistics(hdd_stat.second.device, statistics);
    if (ret != EXIT_SUCCESS) {
      hdd_stat.second.error_message = fmt::format("Failed to open stat. {}}", strerror(ret));
      continue;
    }

    hdd_stat.second.last_statistics = statistics;
  }

  last_hdd_stat_update_time_ = this->now();
}

void HddMonitor::update_hdd_statistics()
{
  double duration_sec = (this->now() - last_hdd_stat_update_time_).seconds();

  for (auto & hdd_stat : hdd_stats_) {
    hdd_stat.second.error_message = "";

    // Skip if device is disconnected
    if (!hdd_connected_flags_[hdd_stat.first]) {
      continue;
    }

    DeviceStatistics statistics{};
    int ret = get_device_statistics(hdd_stat.second.device, statistics);
    if (ret != EXIT_SUCCESS) {
      hdd_stat.second.error_message = fmt::format("Failed to open stat. {}}", strerror(ret));
      continue;
    }

    DeviceStatistics & last_statistics = hdd_stat.second.last_statistics;

    hdd_stat.second.read_data_rate_MBs = get_increase_sysfs_device_stat_value_per_sec(
      statistics.read_sectors, last_statistics.read_sectors, duration_sec);
    hdd_stat.second.read_data_rate_MBs /= 2048;
    hdd_stat.second.write_data_rate_MBs = get_increase_sysfs_device_stat_value_per_sec(
      statistics.write_sectors, last_statistics.write_sectors, duration_sec);
    hdd_stat.second.write_data_rate_MBs /= 2048;
    hdd_stat.second.read_iops = get_increase_sysfs_device_stat_value_per_sec(
      statistics.read_ios, last_statistics.read_ios, duration_sec);
    hdd_stat.second.write_iops = get_increase_sysfs_device_stat_value_per_sec(
      statistics.write_ios, last_statistics.write_ios, duration_sec);

    hdd_stat.second.last_statistics = statistics;
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

int HddMonitor::get_device_statistics(const std::string & device, DeviceStatistics & statistics)
{
  uint64_t read_ios = 0;           // Number of read I/Os processed
  uint64_t read_merges = 0;        // Number of read I/Os merged with in-queue I/O
  uint64_t read_sectors = 0;       // Number of sectors read
  uint64_t read_ticks = 0;         // Total wait time for read requests
  uint64_t write_ios = 0;          // Number of write I/Os processed
  uint64_t write_merges = 0;       // Number of write I/Os merged with in-queue I/O
  uint64_t write_sectors = 0;      // Number of sectors written
  unsigned int write_ticks = 0;    // Total wait time for write requests
  unsigned int in_flight = 0;      // Number of I/Os currently in flight
  unsigned int io_ticks = 0;       // Total time this block device has been active
  unsigned int time_in_queue = 0;  // Total wait time for all requests
  uint64_t discard_ios = 0;        // Number of discard I/Os processed
  uint64_t discard_merges = 0;     // Number of discard I/Os merged with in-queue I/O
  uint64_t discard_sectors = 0;    // Number of sectors discarded
  uint64_t discard_ticks = 0;      // Total wait time for discard requests

  std::ifstream file(fmt::format("/sys/block/{}/stat", device), std::ios::in);
  if (file.fail()) {
    RCLCPP_ERROR(
      get_logger(), "Failed to open statistics of %s. %s", device.c_str(), strerror(errno));
    return errno;
  }

  std::string line;
  getline(file, line);
  int scanned = sscanf(
    line.data(), "%lu %lu %lu %lu %lu %lu %lu %u %u %u %u %lu %lu %lu %lu", &read_ios, &read_merges,
    &read_sectors, &read_ticks, &write_ios, &write_merges, &write_sectors, &write_ticks, &in_flight,
    &io_ticks, &time_in_queue, &discard_ios, &discard_merges, &discard_sectors, &discard_ticks);

  if (scanned < 7) {
    return EIO;
  }

  statistics.read_ios = read_ios;
  statistics.read_sectors = read_sectors;
  statistics.write_ios = write_ios;
  statistics.write_sectors = write_sectors;

  return EXIT_SUCCESS;
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
    communication_diag_.summary(DiagStatus::ERROR, "connect error");
    communication_diag_.add("connect", error_code.message().c_str());
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
    communication_diag_.summary(DiagStatus::ERROR, "write error");
    communication_diag_.add("write", error_code.message().c_str());
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
    communication_diag_.summary(DiagStatus::ERROR, "read error");
    communication_diag_.add("read", error_code.message().c_str());
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
    communication_diag_.summary(DiagStatus::ERROR, "restore error");
    communication_diag_.add("restore", error_code.message().c_str());
  }
}

void HddMonitor::close_connection()
{
  // Close socket
  socket_->close();
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(HddMonitor)
