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
 * @file cpu_monitor_base.cpp
 * @brief CPU monitor base class
 */

#include "system_monitor/cpu_monitor/cpu_monitor_base.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <boost/filesystem.hpp>
#include <boost/process.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/thread.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <regex>
#include <string>

namespace bp = boost::process;
namespace fs = boost::filesystem;
namespace pt = boost::property_tree;

CPUMonitorBase::CPUMonitorBase(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  updater_(this),
  hostname_(),
  num_cores_(0),
  temps_(),
  freqs_(),
  mpstat_exists_(false),
  usage_warn_(declare_parameter<float>("usage_warn", 0.96)),
  usage_error_(declare_parameter<float>("usage_error", 0.96)),
  usage_warn_count_(declare_parameter<int>("usage_warn_count", 1)),
  usage_error_count_(declare_parameter<int>("usage_error_count", 2)),
  usage_avg_(declare_parameter<bool>("usage_avg", true))
{
  using namespace std::literals::chrono_literals;

  gethostname(hostname_, sizeof(hostname_));
  num_cores_ = boost::thread::hardware_concurrency();
  usage_warn_check_cnt_.resize(num_cores_ + 2);   // 2 = all + dummy
  usage_error_check_cnt_.resize(num_cores_ + 2);  // 2 = all + dummy

  // Check if command exists
  fs::path p = bp::search_path("mpstat");
  mpstat_exists_ = (p.empty()) ? false : true;

  updater_.setHardwareID(hostname_);
  updater_.add("CPU Temperature", this, &CPUMonitorBase::checkTemp);
  updater_.add("CPU Usage", this, &CPUMonitorBase::checkUsage);
  updater_.add("CPU Load Average", this, &CPUMonitorBase::checkLoad);
  updater_.add("CPU Thermal Throttling", this, &CPUMonitorBase::checkThrottling);
  updater_.add("CPU Frequency", this, &CPUMonitorBase::checkFrequency);

  // Publisher
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_cpu_usage_ =
    this->create_publisher<tier4_external_api_msgs::msg::CpuUsage>("~/cpu_usage", durable_qos);

  // timer
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  temp_timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&MemMonitor::onTempTimer, this), timer_callback_group_);
  usage_timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&MemMonitor::onUsageTimer, this), timer_callback_group_);
  load_timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&MemMonitor::onLoadTimer, this), timer_callback_group_);
  throt_timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&MemMonitor::onThrottlingTimer, this), timer_callback_group_);
  freq_timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&MemMonitor::onFrequencyTimer, this), timer_callback_group_);
}

void CPUMonitorBase::update()
{
  updater_.force_update();
}

void CPUMonitorBase::checkTemp(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (temps_.empty()) {
    stat.summary(DiagStatus::ERROR, "temperature files not found");
    return;
  }

  int level = DiagStatus::OK;
  std::string error_str = "";

  for (auto itr = temps_.begin(); itr != temps_.end(); ++itr) {
    // Read temperature file
    const fs::path path(itr->path_);
    fs::ifstream ifs(path, std::ios::in);
    if (!ifs) {
      stat.add("file open error", itr->path_);
      error_str = "file open error";
      continue;
    }

    float temp;
    ifs >> temp;
    ifs.close();
    temp /= 1000;
    stat.addf(itr->label_, "%.1f DegC", temp);
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(level, temp_dict_.at(level));
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void CPUMonitorBase::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  tier4_external_api_msgs::msg::CpuUsage cpu_usage;
  using CpuStatus = tier4_external_api_msgs::msg::CpuStatus;

  std::ifstream ifs("/proc/stat");
  std::vector<cpu_usage_info> curr_usages;
  std::string line;
  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;

  if (!ifs) {
    stat.summary(DiagStatus::ERROR, "ifstream error");
    stat.add("ifstream", strerror(errno));
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }

  while (std::getline(ifs, line)) {
    if (line.compare(0, 3, "cpu") == 0) {
      std::istringstream ss(line);
      cpu_usage_info tmp_usage;
      if (!(ss >> tmp_usage.cpu_name_ >> tmp_usage.usr_ >> tmp_usage.nice_ >> tmp_usage.sys_ >>
            tmp_usage.idle_ >> tmp_usage.iowait_ >> tmp_usage.irq_ >> tmp_usage.soft_ >>
            tmp_usage.steal_)) {
        stat.summary(DiagStatus::ERROR, "parsing error");
        stat.add("istringstream", "Error parsing line in /proc/stat: " + line);
        cpu_usage.all.status = CpuStatus::STALE;
        publishCpuUsage(cpu_usage);
        return;
      }
      curr_usages.push_back(tmp_usage);
    } else {
      break;
    }
  }

  if (prev_usages_.empty()) {
    prev_usages_ = curr_usages;
    stat.summary(DiagStatus::ERROR, "cpu usages update error");
    stat.add("update error", "Error update cpu usage info from /proc/stat");
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }

  if (prev_usages_[0].usr_ < 0) {
    stat.summary(DiagStatus::ERROR, "cpu usages overflow error");
    stat.add("overflow error", "A value of /proc/stat is greater than INT_MAX. Restart ECU");
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }

  if (prev_usages_.size() != curr_usages.size()) {
    stat.summary(DiagStatus::ERROR, "cpu usages update error");
    stat.add("update error", "Error update cpu usage info from /proc/stat");
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }

  for (int i = 0; i < prev_usages_.size(); i++) {
    CpuStatus cpu_status;
    int total_diff = curr_usages[i].totalTime() - prev_usages_[i].totalTime();

    cpu_status.usr = 100.0 * (curr_usages[i].usr_ - prev_usages_[i].usr_) / total_diff;
    cpu_status.nice = 100.0 * (curr_usages[i].nice_ - prev_usages_[i].nice_) / total_diff;
    cpu_status.sys = 100.0 * (curr_usages[i].sys_ - prev_usages_[i].sys_) / total_diff;
    cpu_status.idle = 100.0 * (curr_usages[i].idle_ - prev_usages_[i].idle_) / total_diff;
    cpu_status.total =
      100.0 * (curr_usages[i].totalActiveTime() - prev_usages_[i].totalActiveTime()) / total_diff;
    level = CpuUsageToLevel(curr_usages[i].cpu_name_, cpu_status.total * 1e-2);
    cpu_status.status = level;

    if (curr_usages[i].cpu_name_ == "cpu") curr_usages[i].cpu_name_ = "cpu all";
    stat.add(fmt::format("{}: status", curr_usages[i].cpu_name_), load_dict_.at(level));
    stat.addf(fmt::format("{}: total", curr_usages[i].cpu_name_), "%.2f%%", cpu_status.total);
    stat.addf(fmt::format("{}: usr", curr_usages[i].cpu_name_), "%.2f%%", cpu_status.usr);
    stat.addf(fmt::format("{}: nice", curr_usages[i].cpu_name_), "%.2f%%", cpu_status.nice);
    stat.addf(fmt::format("{}: sys", curr_usages[i].cpu_name_), "%.2f%%", cpu_status.sys);
    stat.addf(fmt::format("{}: idle", curr_usages[i].cpu_name_), "%.2f%%", cpu_status.idle);

    if (usage_avg_ == true) {
      if (curr_usages[i].cpu_name_ == "cpu all") {
        whole_level = level;
      }
    } else {
      whole_level = std::max(whole_level, level);
    }

    if (curr_usages[i].cpu_name_ == "cpu all") {
      cpu_usage.all = cpu_status;
    } else {
      cpu_usage.cpus.push_back(cpu_status);
    }
  }

  stat.summary(whole_level, load_dict_.at(whole_level));

  // Publish msg
  publishCpuUsage(cpu_usage);

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

int CPUMonitorBase::CpuUsageToLevel(const std::string & cpu_name, float usage)
{
  // cpu name to counter index
  int idx;
  try {
    int num = std::stoi(cpu_name.substr(3));
    if (num > num_cores_ || num < 0) {
      num = num_cores_;
    }
    idx = num + 1;
  } catch (std::exception &) {
    if (cpu_name == std::string("cpu")) {  // mpstat output "all"
      idx = 0;
    } else {
      idx = num_cores_ + 1;
    }
  }

  // convert CPU usage to level
  int level = DiagStatus::OK;
  if (usage >= usage_warn_) {
    if (usage_warn_check_cnt_[idx] < usage_warn_count_) {
      usage_warn_check_cnt_[idx]++;
    }
    if (usage_warn_check_cnt_[idx] >= usage_warn_count_) {
      level = DiagStatus::WARN;
    }
  } else {
    usage_warn_check_cnt_[idx] = 0;
  }
  if (usage >= usage_error_) {
    if (usage_error_check_cnt_[idx] < usage_error_count_) {
      usage_error_check_cnt_[idx]++;
    }
    if (usage_error_check_cnt_[idx] >= usage_error_count_) {
      level = DiagStatus::ERROR;
    }
  } else {
    usage_error_check_cnt_[idx] = 0;
  }

  return level;
}

void CPUMonitorBase::checkLoad(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  double loadavg[3];

  std::ifstream ifs("/proc/loadavg", std::ios::in);

  if (!ifs) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", strerror(errno));
    return;
  }

  std::string line;

  if (!std::getline(ifs, line)) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", "format error");
    return;
  }

  if (sscanf(line.c_str(), "%lf %lf %lf", &loadavg[0], &loadavg[1], &loadavg[2]) != 3) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", "format error");
    return;
  }

  loadavg[0] /= num_cores_;
  loadavg[1] /= num_cores_;
  loadavg[2] /= num_cores_;

  stat.summary(DiagStatus::OK, "OK");
  stat.addf("1min", "%.2f%%", loadavg[0] * 1e2);
  stat.addf("5min", "%.2f%%", loadavg[1] * 1e2);
  stat.addf("15min", "%.2f%%", loadavg[2] * 1e2);

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void CPUMonitorBase::checkThrottling(
  [[maybe_unused]] diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  RCLCPP_INFO(this->get_logger(), "CPUMonitorBase::checkThrottling not implemented.");
}

void CPUMonitorBase::checkFrequency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (freqs_.empty()) {
    stat.summary(DiagStatus::ERROR, "frequency files not found");
    return;
  }

  for (auto itr = freqs_.begin(); itr != freqs_.end(); ++itr) {
    // Read scaling_cur_freq file
    const fs::path path(itr->path_);
    fs::ifstream ifs(path, std::ios::in);
    if (ifs) {
      std::string line;
      if (std::getline(ifs, line)) {
        stat.addf(fmt::format("CPU {}: clock", itr->index_), "%d MHz", std::stoi(line) / 1000);
      }
    }
    ifs.close();
  }

  stat.summary(DiagStatus::OK, "OK");

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void CPUMonitorBase::onTempTimer()
{
  // Remember start time to measure elapsed time
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("execution_time");

  if (temps_.empty()) {
    // stat.summary(DiagStatus::ERROR, "temperature files not found");
    error_str = "temperature files not found";
    return;
  }

  std::string error_str = "";
  std::vector<float> vec;

  for (auto itr = temps_.begin(); itr != temps_.end(); ++itr) {
    // Read temperature file
    const fs::path path(itr->path_);
    fs::ifstream ifs(path, std::ios::in);
    if (!ifs) {
      stat.add("file open error", itr->path_);
      error_str = "file open error";
      continue;
    }

    float temp;
    ifs >> temp;
    ifs.close();
    temp /= 1000;
    temp_vec.append(temp);
  }

  // Measure elapsed time since start time and report
  const double elapsed_ms = stop_watch.toc("execution_time");

  {
    std::lock_guard<std::mutex> lock(tmp_mutex_);
    temp_error_str = error_str;
    temp_vec_ = vec;
    tmp_elapsed_ms_ = elapsed_ms;
  }
}

void CPUMonitorBase::onUsageTimer()
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  tier4_external_api_msgs::msg::CpuUsage cpu_usage;
  using CpuStatus = tier4_external_api_msgs::msg::CpuStatus;

  std::ifstream ifs("/proc/stat");
  std::vector<cpu_usage_info> curr_usages;
  std::string line;
  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;

  if (!ifs) {
    stat.summary(DiagStatus::ERROR, "ifstream error");
    stat.add("ifstream", strerror(errno));
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }

  while (std::getline(ifs, line)) {
    if (line.compare(0, 3, "cpu") == 0) {
      std::istringstream ss(line);
      cpu_usage_info tmp_usage;
      if (!(ss >> tmp_usage.cpu_name_ >> tmp_usage.usr_ >> tmp_usage.nice_ >> tmp_usage.sys_ >>
            tmp_usage.idle_ >> tmp_usage.iowait_ >> tmp_usage.irq_ >> tmp_usage.soft_ >>
            tmp_usage.steal_)) {
        stat.summary(DiagStatus::ERROR, "parsing error");
        stat.add("istringstream", "Error parsing line in /proc/stat: " + line);
        cpu_usage.all.status = CpuStatus::STALE;
        publishCpuUsage(cpu_usage);
        return;
      }
      curr_usages.push_back(tmp_usage);
    } else {
      break;
    }
  }

  if (prev_usages_.empty()) {
    prev_usages_ = curr_usages;
    stat.summary(DiagStatus::ERROR, "cpu usages update error");
    stat.add("update error", "Error update cpu usage info from /proc/stat");
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }

  if (prev_usages_[0].usr_ < 0) {
    stat.summary(DiagStatus::ERROR, "cpu usages overflow error");
    stat.add("overflow error", "A value of /proc/stat is greater than INT_MAX. Restart ECU");
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }

  if (prev_usages_.size() != curr_usages.size()) {
    stat.summary(DiagStatus::ERROR, "cpu usages update error");
    stat.add("update error", "Error update cpu usage info from /proc/stat");
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }

  for (int i = 0; i < prev_usages_.size(); i++) {
    CpuStatus cpu_status;
    int total_diff = curr_usages[i].totalTime() - prev_usages_[i].totalTime();

    cpu_status.usr = 100.0 * (curr_usages[i].usr_ - prev_usages_[i].usr_) / total_diff;
    cpu_status.nice = 100.0 * (curr_usages[i].nice_ - prev_usages_[i].nice_) / total_diff;
    cpu_status.sys = 100.0 * (curr_usages[i].sys_ - prev_usages_[i].sys_) / total_diff;
    cpu_status.idle = 100.0 * (curr_usages[i].idle_ - prev_usages_[i].idle_) / total_diff;
    cpu_status.total =
      100.0 * (curr_usages[i].totalActiveTime() - prev_usages_[i].totalActiveTime()) / total_diff;
    level = CpuUsageToLevel(curr_usages[i].cpu_name_, cpu_status.total * 1e-2);
    cpu_status.status = level;

    if (curr_usages[i].cpu_name_ == "cpu") curr_usages[i].cpu_name_ = "cpu all";
    stat.add(fmt::format("{}: status", curr_usages[i].cpu_name_), load_dict_.at(level));
    stat.addf(fmt::format("{}: total", curr_usages[i].cpu_name_), "%.2f%%", cpu_status.total);
    stat.addf(fmt::format("{}: usr", curr_usages[i].cpu_name_), "%.2f%%", cpu_status.usr);
    stat.addf(fmt::format("{}: nice", curr_usages[i].cpu_name_), "%.2f%%", cpu_status.nice);
    stat.addf(fmt::format("{}: sys", curr_usages[i].cpu_name_), "%.2f%%", cpu_status.sys);
    stat.addf(fmt::format("{}: idle", curr_usages[i].cpu_name_), "%.2f%%", cpu_status.idle);

    if (usage_avg_ == true) {
      if (curr_usages[i].cpu_name_ == "cpu all") {
        whole_level = level;
      }
    } else {
      whole_level = std::max(whole_level, level);
    }

    if (curr_usages[i].cpu_name_ == "cpu all") {
      cpu_usage.all = cpu_status;
    } else {
      cpu_usage.cpus.push_back(cpu_status);
    }
  }

  stat.summary(whole_level, load_dict_.at(whole_level));

  // Publish msg
  publishCpuUsage(cpu_usage);

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void CPUMonitorBase::onLoadTimer()
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  double loadavg[3];

  std::ifstream ifs("/proc/loadavg", std::ios::in);

  if (!ifs) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", strerror(errno));
    return;
  }

  std::string line;

  if (!std::getline(ifs, line)) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", "format error");
    return;
  }

  if (sscanf(line.c_str(), "%lf %lf %lf", &loadavg[0], &loadavg[1], &loadavg[2]) != 3) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", "format error");
    return;
  }

  loadavg[0] /= num_cores_;
  loadavg[1] /= num_cores_;
  loadavg[2] /= num_cores_;

  stat.summary(DiagStatus::OK, "OK");
  stat.addf("1min", "%.2f%%", loadavg[0] * 1e2);
  stat.addf("5min", "%.2f%%", loadavg[1] * 1e2);
  stat.addf("15min", "%.2f%%", loadavg[2] * 1e2);

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void CPUMonitorBase::onThrottlingTimer(
  [[maybe_unused]] diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // do nothing
}

void CPUMonitorBase::onFrequencyTimer()
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (freqs_.empty()) {
    stat.summary(DiagStatus::ERROR, "frequency files not found");
    return;
  }

  for (auto itr = freqs_.begin(); itr != freqs_.end(); ++itr) {
    // Read scaling_cur_freq file
    const fs::path path(itr->path_);
    fs::ifstream ifs(path, std::ios::in);
    if (ifs) {
      std::string line;
      if (std::getline(ifs, line)) {
        stat.addf(fmt::format("CPU {}: clock", itr->index_), "%d MHz", std::stoi(line) / 1000);
      }
    }
    ifs.close();
  }

  stat.summary(DiagStatus::OK, "OK");

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void CPUMonitorBase::getTempNames()
{
  RCLCPP_INFO(this->get_logger(), "CPUMonitorBase::getTempNames not implemented.");
}

void CPUMonitorBase::getFreqNames()
{
  const fs::path root("/sys/devices/system/cpu");

  for (const fs::path & path :
       boost::make_iterator_range(fs::directory_iterator(root), fs::directory_iterator())) {
    if (!fs::is_directory(path)) {
      continue;
    }

    std::cmatch match;
    const char * cpu_dir = path.generic_string().c_str();

    // /sys/devices/system/cpu[0-9] ?
    if (!std::regex_match(cpu_dir, match, std::regex(".*cpu(\\d+)"))) {
      continue;
    }

    // /sys/devices/system/cpu[0-9]/cpufreq/scaling_cur_freq
    cpu_freq_info freq;
    const fs::path freq_path = path / "cpufreq/scaling_cur_freq";
    freq.index_ = std::stoi(match[1].str());
    freq.path_ = freq_path.generic_string();
    freqs_.push_back(freq);
  }

  std::sort(freqs_.begin(), freqs_.end(), [](const cpu_freq_info & c1, const cpu_freq_info & c2) {
    return c1.index_ < c2.index_;
  });  // NOLINT
}

void CPUMonitorBase::publishCpuUsage(tier4_external_api_msgs::msg::CpuUsage usage)
{
  // Create timestamp
  const auto stamp = this->now();

  usage.stamp = stamp;
  pub_cpu_usage_->publish(usage);
}
