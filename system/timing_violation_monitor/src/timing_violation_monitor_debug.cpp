// Copyright 2022 TIER IV, Inc.
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

#include "timing_violation_monitor/timing_violation_monitor_debug.hpp"

#include "timing_violation_monitor/timing_violation_monitor_core.hpp"

#define FMT_HEADER_ONLY
#include <fmt/format.h>

namespace timing_violation_monitor
{
// const
static constexpr char tm_command_topic[] = "timing_violation_monitor_command";
static constexpr char SHOW_INFO[] = "show info";
static constexpr char REQ_INFO[] = "req info";
static constexpr char PRINT_LOG[] = "show hist";
static constexpr char CLR_INFO[] = "clear info";
static constexpr char ENA_DETECT[] = "detection on";
static constexpr char DIS_DETECT[] = "detection off";
static constexpr char ENA_DBG[] = "dbg on";
static constexpr char DIS_DBG[] = "dbg off";
static constexpr char ENA_LOG[] = "hist on";
static constexpr char DIS_LOG[] = "hist off";
static constexpr char DISP_ON[] = "disp on";
static constexpr char DISP_OFF[] = "disp off";

PathDebugInfoMap path_debug_info_;
CbStatisticsMap cb_statistics_map_;

// register TimingViolationMonitorPathDebug
void TimingViolationMonitorDebug::registerPathDebugInfo(
  uint32_t key, const std::shared_ptr<TimingViolationMonitorPathDebug> dinfo_ptr)
{
  path_debug_info_.insert(std::make_pair(key, dinfo_ptr));
}

// TimingViolationMonitorDebug constructor
TimingViolationMonitorDebug::TimingViolationMonitorDebug(const char * version, bool debug_ctrl)
: version(version), debug_ctrl(debug_ctrl), enable_log(debug_ctrl)
{
  log_disp = false;
}

void TimingViolationMonitorDebug::registerNodeToDebug(
  const std::shared_ptr<TimingViolationMonitor> & node)
{
  this->node = node;
  rclcpp::QoS qos = rclcpp::QoS{1};
  // Publisher
  pub_tm_statistics_ = node->create_publisher<tier4_system_msgs::msg::TimingViolationMonitorInfos>(
    "~/output/timing_violation_monitor/statistics", qos);
  // command topic
  cmd_sub_ = node->create_subscription<tier4_system_msgs::msg::TimingViolationMonitorCommand>(
    tm_command_topic, qos,
    [this](tier4_system_msgs::msg::TimingViolationMonitorCommand::ConstSharedPtr msg) {
      TimingViolationMonitorDebug::onCommand(msg);
    });
}

// show stats
void TimingViolationMonitorDebug::cmdShowStatistics()
{
  std::string fs =
    fmt::format("\n----- timing violation monitor statistics ({}) start -----\n", this->version);
  for (auto pair : path_debug_info_) {
    auto dinfo_ptr = pair.second;
    auto pinfo_ptr = dinfo_ptr->pinfo_ptr;
    fs += fmt::format(
      "path_name={} path_i={} p_i={}(ms) d_i={}(ms)\n", pinfo_ptr->path_name.c_str(),
      pinfo_ptr->index, pinfo_ptr->p_i * 1000, pinfo_ptr->d_i * 1000);
    fs += fmt::format("topic={} [{}]\n", pinfo_ptr->topic.c_str(), pinfo_ptr->mtype.c_str());
    fs +=
      fmt::format("deadline detect={}\n", (dinfo_ptr->enable_detect == true) ? "true" : "false");
    fs += fmt::format(
      "topic valid={} discard={}\n", dinfo_ptr->valid_topic_count, dinfo_ptr->discard_topic_count);
    fs += fmt::format(
      "path OK={} NG={}\n", dinfo_ptr->completed_count,
      dinfo_ptr->deadline_miss_count + dinfo_ptr->false_deadline_miss_count +
        dinfo_ptr->presumed_deadline_miss_count);
    fs += fmt::format("path completed={}\n", dinfo_ptr->completed_count);
    fs += fmt::format(
      "deadline miss={} false_miss={} presumed miss={}\n", dinfo_ptr->deadline_miss_count,
      dinfo_ptr->false_deadline_miss_count, dinfo_ptr->presumed_deadline_miss_count);
    fs += fmt::format(
      "response time({}) min={:.6f} ave={:.6f} max={:.6f} (sec)\n",
      dinfo_ptr->response_time.getCnt(), dinfo_ptr->response_time.getMin(),
      dinfo_ptr->response_time.getAve(), dinfo_ptr->response_time.getMax());
    fs += fmt::format(
      "too long response time({}) min={:.6f} ave={:.6f} max={:.6f} (sec)\n",
      dinfo_ptr->too_long_response_time.getCnt(), dinfo_ptr->too_long_response_time.getMin(),
      dinfo_ptr->too_long_response_time.getAve(), dinfo_ptr->too_long_response_time.getMax());
    fs += fmt::format("cur_j={} completed_j={}\n", pinfo_ptr->cur_j, pinfo_ptr->completed_j);
    fs += fmt::format("r_i_j_1={:.6f} r_i_j={:.6f}\n", pinfo_ptr->r_i_j_1, pinfo_ptr->r_i_j);
    fs += fmt::format(
      "topic({}) HZ min={:.6f} ave={:.6f} max={:.6f} (sec) d_i over={} per limit={}\n",
      dinfo_ptr->hz.getCnt(), dinfo_ptr->hz.getMin(), dinfo_ptr->hz.getAve(),
      dinfo_ptr->hz.getMax(), dinfo_ptr->hz.getOver(), dinfo_ptr->hz.getPerLimit());
    fs += fmt::format(
      "topic({}) Sub interval min={:.6f} ave={:.6f} max={:.6f} (sec) d_i over={} per limit={}\n",
      dinfo_ptr->sub_interval.getCnt(), dinfo_ptr->sub_interval.getMin(),
      dinfo_ptr->sub_interval.getAve(), dinfo_ptr->sub_interval.getMax(),
      dinfo_ptr->sub_interval.getOver(), dinfo_ptr->sub_interval.getPerLimit());
    fs += fmt::format(
      "communication delay({}) min={:.6f} ave={:.6f} max={:.6f} (sec)\n",
      dinfo_ptr->com_delay.getCnt(), dinfo_ptr->com_delay.getMin(), dinfo_ptr->com_delay.getAve(),
      dinfo_ptr->com_delay.getMax());
    for (auto kv : pinfo_ptr->deadline_timer) {
      auto dm = kv.second;
      fs += fmt::format(
        "-- j={}[{}] valid={} start={:.6f}\n", dm.self_j, dm.uniq, dm.valid, dm.start_time);
    }
    fs += fmt::format("-------------\n", this->version);
  }
  if (debug_ctrl) {
    fs += fmt::format("--- callbacks ---\n");
    for (auto & cb : cb_statistics_map_) {
      fs += fmt::format(
        "[{}] ({}) min={:.6f} ave={:.6f} max={:.6f} (sec)", cb.first.c_str(), cb.second.getCnt(),
        cb.second.getMin(), cb.second.getAve(), cb.second.getMax());
    }
  }
  fs += fmt::format("----- timing violation monitor statistics ({}) end -----\n", this->version);
  RCLCPP_INFO(this->node->get_logger(), "%s", fs.c_str());
}

// command topic callback
void TimingViolationMonitorDebug::onCommand(
  const tier4_system_msgs::msg::TimingViolationMonitorCommand::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(tm_mutex_);
  cbStatisticsEnter(__func__);

  std::string fs;
  if (msg->command == SHOW_INFO) {
    cmdShowStatistics();
  } else if (msg->command == REQ_INFO) {
    pubCmdReqInfo();
    fs = fmt::format("\n--- req info\n");
    fs += fmt::format("debug={} log={} disp={}\n", debug_ctrl, enable_log, log_disp);
    RCLCPP_INFO(this->node->get_logger(), "%s", fs.c_str());
  } else if (msg->command == PRINT_LOG) {
    printLog();
    RCLCPP_INFO(this->node->get_logger(), "\n--- print log\n");
  } else if (msg->command == CLR_INFO) {
    clearInfo();
    RCLCPP_INFO(this->node->get_logger(), "\n--- clear info\n");
  } else if (msg->command == ENA_DETECT) {
    detectCtrl(true);
    RCLCPP_INFO(this->node->get_logger(), "\n--- enable timing violation detection\n");
  } else if (msg->command == DIS_DETECT) {
    detectCtrl(false);
    RCLCPP_INFO(this->node->get_logger(), "\n--- disable timing violation detection\n");
  } else if (msg->command == ENA_LOG) {
    enLog(true);
    fs = fmt::format("\n--- enable logging\n");
    fs += fmt::format("debug={} log={} disp={}\n", debug_ctrl, enable_log, log_disp);
    RCLCPP_INFO(this->node->get_logger(), "%s", fs.c_str());
  } else if (msg->command == DIS_LOG) {
    enLog(false);
    fs = fmt::format("\n--- disable logging\n");
    fs += fmt::format("debug={} log={} disp={}\n", debug_ctrl, enable_log, log_disp);
    RCLCPP_INFO(this->node->get_logger(), "%s", fs.c_str());
  } else if (msg->command == ENA_DBG) {
    enDbg(true);
    fs = fmt::format("\n--- enable debug\n");
    fs += fmt::format("debug={} log={} disp={}\n", debug_ctrl, enable_log, log_disp);
    RCLCPP_INFO(this->node->get_logger(), "%s", fs.c_str());
  } else if (msg->command == DIS_DBG) {
    enDbg(false);
    fs = fmt::format("\n--- disabled debug\n");
    fs += fmt::format("debug={} log={} disp={}\n", debug_ctrl, enable_log, log_disp);
    RCLCPP_INFO(this->node->get_logger(), "%s", fs.c_str());
  } else if (msg->command == DISP_ON) {
    dispLogCtrl(true);
    fs = fmt::format("\ndebug={} log={} disp={}\n", debug_ctrl, enable_log, log_disp);
    RCLCPP_INFO(this->node->get_logger(), "%s", fs.c_str());
  } else if (msg->command == DISP_OFF) {
    dispLogCtrl(false);
    fs = fmt::format("\ndebug={} log={} disp={}\n", debug_ctrl, enable_log, log_disp);
    RCLCPP_INFO(this->node->get_logger(), "%s", fs.c_str());
  } else {
    RCLCPP_WARN(
      this->node->get_logger(), "[%s]:%04d ## not supported command [%s]", __func__, __LINE__,
      msg->command.c_str());
  }

  cbStatisticsExit(__func__);
}

// publish statistics
void TimingViolationMonitorDebug::pubCmdReqInfo()
{
  if (!debug_ctrl) {
    return;
  }
  // RCLCPP_INFO(this->node->get_logger(), "--[%s]:%04d called", __func__, __LINE__);
  auto m = tier4_system_msgs::msg::TimingViolationMonitorInfos();
  for (auto & pair : path_debug_info_) {
    auto dinfo_ptr = pair.second;
    auto pinfo_ptr = dinfo_ptr->pinfo_ptr;
    auto p = tier4_system_msgs::msg::TimingViolationMonitorPathInfos();
    p.path_name = pinfo_ptr->path_name.c_str();
    p.topic = pinfo_ptr->topic;
    p.valid_topic_count = dinfo_ptr->valid_topic_count;
    p.discard_topic_count = dinfo_ptr->discard_topic_count;
    p.completed_count = dinfo_ptr->completed_count;
    p.deadline_miss_count = dinfo_ptr->deadline_miss_count;
    p.false_deadline_miss_count = dinfo_ptr->false_deadline_miss_count;
    p.presumed_deadline_miss_count = dinfo_ptr->presumed_deadline_miss_count;
    p.response_count = dinfo_ptr->response_time.getCnt();
    p.response_time_min = dinfo_ptr->response_time.getMin();
    p.response_time_max = dinfo_ptr->response_time.getMax();
    p.response_time_ave = dinfo_ptr->response_time.getAve();
    p.too_long_response_count = dinfo_ptr->too_long_response_time.getCnt();
    p.too_long_response_time_min = dinfo_ptr->too_long_response_time.getMin();
    p.too_long_response_time_max = dinfo_ptr->too_long_response_time.getMax();
    p.too_long_response_time_ave = dinfo_ptr->too_long_response_time.getAve();
    p.path_i = pinfo_ptr->index;
    p.cur_j = pinfo_ptr->cur_j;
    p.completed_j = pinfo_ptr->completed_j;
    p.r_i_j_1_stamp = pinfo_ptr->r_i_j_1_stamp;
    p.r_i_j_1_float = pinfo_ptr->r_i_j_1;
    p.r_i_j_float = pinfo_ptr->r_i_j;
    p.hz_min = dinfo_ptr->hz.getMin();
    p.hz_max = dinfo_ptr->hz.getMax();
    p.hz_ave = dinfo_ptr->hz.getAve();
    p.sub_min = dinfo_ptr->sub_interval.getMin();
    p.sub_max = dinfo_ptr->sub_interval.getMax();
    p.sub_ave = dinfo_ptr->sub_interval.getAve();
    m.path_info.push_back(p);
  }
  for (auto & cb : cb_statistics_map_) {
    auto c = tier4_system_msgs::msg::TimingViolationMonitorCbLatency();
    c.cb_name = cb.first.c_str();
    c.cb_min = cb.second.getMin();
    c.cb_max = cb.second.getMax();
    c.cb_ave = cb.second.getAve();
    c.cb_count = cb.second.getCnt();
    m.cb_latency.push_back(c);
  }
  m.header.stamp = this->node->get_clock()->now();
  pub_tm_statistics_->publish(m);
}

// statistics
bool TimingViolationMonitorDebug::topicStatistics(
  TimingViolationMonitorPathConfig & pinfo, double & pub_time, double & cur_ros,
  double & response_time)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  bool over_f = false;
  auto hz_over = dinfo_ptr->hz.getOver();
  auto sub_over = dinfo_ptr->sub_interval.getOver();
  dinfo_ptr->hz.addRate(pinfo.r_i_j_1, pinfo.d_i);
  dinfo_ptr->sub_interval.addRate(cur_ros, pinfo.d_i);
  if (hz_over != dinfo_ptr->hz.getOver() || sub_over != dinfo_ptr->sub_interval.getOver()) {
    over_f = true;
  }
  dinfo_ptr->com_delay.addData(cur_ros - pub_time, pinfo.d_i);
  dinfo_ptr->response_time.addData(response_time);
  if (response_time >= pinfo.d_i) {
    dinfo_ptr->too_long_response_time.addData(response_time);
    over_f = true;
  } else if (response_time + (cur_ros - pub_time) >= pinfo.d_i) {
    dinfo_ptr->false_deadline_miss_count++;
  }
  return over_f;
}

void TimingViolationMonitorDebug::setTopicCounter(
  TimingViolationMonitorPathConfig & pinfo, bool discard)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  if (discard) {
    dinfo_ptr->discard_topic_count++;
  } else {
    dinfo_ptr->valid_topic_count++;
  }
}

void TimingViolationMonitorDebug::setCompletedCounter(TimingViolationMonitorPathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  dinfo_ptr->completed_count++;
}

void TimingViolationMonitorDebug::setDeadlineCounter(TimingViolationMonitorPathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  dinfo_ptr->deadline_miss_count++;
}

void TimingViolationMonitorDebug::setFalseDeadlineCounter(TimingViolationMonitorPathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  dinfo_ptr->false_deadline_miss_count++;
}

void TimingViolationMonitorDebug::setPresumedDeadlineCounter(
  TimingViolationMonitorPathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  dinfo_ptr->presumed_deadline_miss_count++;
}

uint64_t TimingViolationMonitorDebug::getOkCounter(TimingViolationMonitorPathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  return dinfo_ptr->completed_count;
}

uint64_t TimingViolationMonitorDebug::getNgCounter(TimingViolationMonitorPathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  return dinfo_ptr->deadline_miss_count + dinfo_ptr->false_deadline_miss_count +
         dinfo_ptr->presumed_deadline_miss_count;
}

uint64_t TimingViolationMonitorDebug::getValidTopicCounter(TimingViolationMonitorPathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  return dinfo_ptr->valid_topic_count;
}

uint64_t TimingViolationMonitorDebug::getDiscardTopicCounter(
  TimingViolationMonitorPathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  return dinfo_ptr->discard_topic_count;
}

// measurement callback process time
void TimingViolationMonitorDebug::cbStatisticsEnter(const char * func)
{
  if (debug_ctrl) {
    std::string fn = func;
    if (cb_statistics_map_.find(fn) == cb_statistics_map_.end()) {
      ElapseMinMax tmp;
      tmp.setName(func);
      cb_statistics_map_[fn] = tmp;
    }
    auto & cs = cb_statistics_map_[fn];
    cs.setPrev();
  }
}

void TimingViolationMonitorDebug::cbStatisticsExit(const char * func)
{
  if (debug_ctrl) {
    std::string fn = func;
    auto & cs = cb_statistics_map_[fn];
    cs.addElapse();
  }
}

// logging
std::deque<std::string> log_buffer_;
void TimingViolationMonitorDebug::log(std::string fs)
{
  if (!enable_log) {
    return;
  }
  if (log_disp) {
    RCLCPP_INFO(this->node->get_logger(), fs.c_str());
  }
  double cur_ros = this->node->get_now();
  log_buffer_.push_back(fmt::format("[{:.6f}] {}", cur_ros, fs.c_str()));
  if (log_buffer_.size() >= 1000 * 100) {
    log_buffer_.erase(log_buffer_.begin(), log_buffer_.begin() + 1000);
  }
}

void TimingViolationMonitorDebug::printLog()
{
  if (!enable_log) {
    return;
  }
  for (auto & fs : log_buffer_) {
    RCLCPP_INFO(this->node->get_logger(), "%s\n", fs.c_str());
  }
}
void TimingViolationMonitorDebug::enDbg(bool ope)
{
  debug_ctrl = ope;
  enable_log = ope;
}

void TimingViolationMonitorDebug::enLog(bool ope) { enable_log = ope; }
void TimingViolationMonitorDebug::dispLogCtrl(bool ope) { log_disp = ope; }

void TimingViolationMonitorDebug::clearInfo()
{
  for (auto & pair : path_debug_info_) {
    auto dinfo_ptr = pair.second;
    dinfo_ptr->completed_count = 0lu;
    dinfo_ptr->deadline_miss_count = 0lu;
    dinfo_ptr->false_deadline_miss_count = 0lu;
    dinfo_ptr->presumed_deadline_miss_count = 0lu;
    dinfo_ptr->valid_topic_count = 0lu;
    dinfo_ptr->discard_topic_count = 0lu;
    dinfo_ptr->response_time.setName("response_time");
    dinfo_ptr->too_long_response_time.setName("too_long_response_time");
    dinfo_ptr->hz.setName("hz");
    dinfo_ptr->sub_interval.setName("sub_interval");
    dinfo_ptr->com_delay.setName("com_delay");
  }
  cb_statistics_map_.clear();
  log_buffer_.clear();
}

void TimingViolationMonitorDebug::detectCtrl(bool ope)
{
  for (auto & pair : path_debug_info_) {
    auto dinfo_ptr = pair.second;
    dinfo_ptr->enable_detect = ope;
  }
}

bool TimingViolationMonitorDebug::getEnableDetect(TimingViolationMonitorPathConfig & pinfo)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  return dinfo_ptr->enable_detect;
}

void TimingViolationMonitorDebug::setEnableDetect(
  TimingViolationMonitorPathConfig & pinfo, bool ope)
{
  auto dinfo_ptr = path_debug_info_[pinfo.index];
  dinfo_ptr->enable_detect = ope;
}

}  // namespace timing_violation_monitor
