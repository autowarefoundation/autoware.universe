// Copyright 2022 Tier IV, Inc.
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

#include "autoware/rtc_interface/rtc_interface.hpp"

#include <string>
#include <vector>

namespace
{
using tier4_rtc_msgs::msg::Module;

std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +uuid.uuid[i];
  }
  return ss.str();
}

std::string command_to_string(const uint8_t type)
{
  if (type == tier4_rtc_msgs::msg::Command::ACTIVATE) {
    return "ACTIVATE";
  }
  if (type == tier4_rtc_msgs::msg::Command::DEACTIVATE) {
    return "DEACTIVATE";
  }

  throw std::domain_error("invalid rtc command.");
}

std::string state_to_string(const uint8_t type)
{
  if (type == tier4_rtc_msgs::msg::State::WAITING_FOR_EXECUTION) {
    return "WAITING_FOR_EXECUTION";
  }
  if (type == tier4_rtc_msgs::msg::State::RUNNING) {
    return "RUNNING";
  }
  if (type == tier4_rtc_msgs::msg::State::ABORTING) {
    return "ABORTING";
  }
  if (type == tier4_rtc_msgs::msg::State::SUCCEEDED) {
    return "SUCCEEDED";
  }
  if (type == tier4_rtc_msgs::msg::State::FAILED) {
    return "FAILED";
  }

  throw std::domain_error("invalid rtc state.");
}

Module getModuleType(const std::string & module_name)
{
  Module module;
  if (module_name == "blind_spot") {
    module.type = Module::BLIND_SPOT;
  } else if (module_name == "crosswalk") {
    module.type = Module::CROSSWALK;
  } else if (module_name == "detection_area") {
    module.type = Module::DETECTION_AREA;
  } else if (module_name == "intersection") {
    module.type = Module::INTERSECTION;
  } else if (module_name == "no_stopping_area") {
    module.type = Module::NO_STOPPING_AREA;
  } else if (module_name == "occlusion_spot") {
    module.type = Module::OCCLUSION_SPOT;
  } else if (module_name == "stop_line") {
    module.type = Module::NONE;
  } else if (module_name == "traffic_light" || module_name == "virtual_traffic_light") {
    module.type = Module::TRAFFIC_LIGHT;
  } else if (module_name == "lane_change_left") {
    module.type = Module::LANE_CHANGE_LEFT;
  } else if (module_name == "lane_change_right") {
    module.type = Module::LANE_CHANGE_RIGHT;
  } else if (module_name == "external_request_lane_change_left") {
    module.type = Module::EXT_REQUEST_LANE_CHANGE_LEFT;
  } else if (module_name == "external_request_lane_change_right") {
    module.type = Module::EXT_REQUEST_LANE_CHANGE_RIGHT;
  } else if (module_name == "avoidance_by_lane_change_left") {
    module.type = Module::AVOIDANCE_BY_LC_LEFT;
  } else if (module_name == "avoidance_by_lane_change_right") {
    module.type = Module::AVOIDANCE_BY_LC_RIGHT;
  } else if (module_name == "static_obstacle_avoidance_left") {
    module.type = Module::AVOIDANCE_LEFT;
  } else if (module_name == "static_obstacle_avoidance_right") {
    module.type = Module::AVOIDANCE_RIGHT;
  } else if (module_name == "goal_planner") {
    module.type = Module::GOAL_PLANNER;
  } else if (module_name == "start_planner") {
    module.type = Module::START_PLANNER;
  } else if (module_name == "intersection_occlusion") {
    module.type = Module::INTERSECTION_OCCLUSION;
  }
  return module;
}

}  // namespace

namespace autoware::rtc_interface
{
RTCInterface::RTCInterface(rclcpp::Node * node, const std::string & name, const bool enable_rtc)
: clock_{node->get_clock()},
  logger_{node->get_logger().get_child("RTCInterface[" + name + "]")},
  is_auto_mode_enabled_{!enable_rtc},
  is_locked_{false}
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  constexpr double update_rate = 10.0;
  const auto period_ns = rclcpp::Rate(update_rate).period();
  timer_ = rclcpp::create_timer(
    node, node->get_clock(), period_ns, std::bind(&RTCInterface::onTimer, this));

  // Publisher
  pub_statuses_ =
    node->create_publisher<CooperateStatusArray>(cooperate_status_namespace_ + "/" + name, 1);

  pub_auto_mode_status_ =
    node->create_publisher<AutoModeStatus>(auto_mode_status_namespace_ + "/" + name, 1);

  // Service
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_commands_ = node->create_service<CooperateCommands>(
    cooperate_commands_namespace_ + "/" + name,
    std::bind(&RTCInterface::onCooperateCommandService, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_);
  srv_auto_mode_ = node->create_service<AutoMode>(
    enable_auto_mode_namespace_ + "/" + name,
    std::bind(&RTCInterface::onAutoModeService, this, _1, _2), rmw_qos_profile_services_default,
    callback_group_);

  // Module
  module_ = getModuleType(name);
}

void RTCInterface::publishCooperateStatus(const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  registered_status_.stamp = stamp;
  pub_statuses_->publish(registered_status_);
}

void RTCInterface::onCooperateCommandService(
  const CooperateCommands::Request::SharedPtr request,
  const CooperateCommands::Response::SharedPtr responses)
{
  std::lock_guard<std::mutex> lock(mutex_);

  responses->responses = validateCooperateCommands(request->commands);

  if (isLocked()) {
    stored_commands_ = request->commands;
    return;
  }

  updateCooperateCommandStatus(request->commands);
}

std::vector<CooperateResponse> RTCInterface::validateCooperateCommands(
  const std::vector<CooperateCommand> & commands)
{
  std::vector<CooperateResponse> responses;

  for (const auto & command : commands) {
    CooperateResponse response;
    response.uuid = command.uuid;
    response.module = command.module;

    const auto itr = std::find_if(
      registered_status_.statuses.begin(), registered_status_.statuses.end(),
      [command](const auto & s) { return s.uuid == command.uuid; });
    if (itr != registered_status_.statuses.end()) {
      if (itr->state.type == State::WAITING_FOR_EXECUTION || itr->state.type == State::RUNNING) {
        response.success = true;
      } else {
        RCLCPP_WARN_STREAM(
          getLogger(), "[validateCooperateCommands] uuid : "
                         << uuid_to_string(command.uuid)
                         << " state is not WAITING_FOR_EXECUTION or RUNNING. state : "
                         << itr->state.type << std::endl);
        response.success = false;
      }
    } else {
      RCLCPP_WARN_STREAM(
        getLogger(), "[validateCooperateCommands] uuid : " << uuid_to_string(command.uuid)
                                                           << " is not found." << std::endl);
      response.success = false;
    }
    responses.push_back(response);
  }

  return responses;
}

void RTCInterface::updateCooperateCommandStatus(const std::vector<CooperateCommand> & commands)
{
  for (const auto & command : commands) {
    const auto itr = std::find_if(
      registered_status_.statuses.begin(), registered_status_.statuses.end(),
      [command](const auto & s) { return s.uuid == command.uuid; });

    // Update command if the command has been already received
    if (itr != registered_status_.statuses.end()) {
      if (itr->state.type == State::WAITING_FOR_EXECUTION || itr->state.type == State::RUNNING) {
        itr->command_status = command.command;
        itr->auto_mode = false;
      }
    }
  }
}

void RTCInterface::onAutoModeService(
  const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_auto_mode_enabled_ = request->enable;
  for (auto & status : registered_status_.statuses) {
    status.auto_mode = request->enable;
  }
  response->success = true;
}

void RTCInterface::onTimer()
{
  AutoModeStatus auto_mode_status;
  auto_mode_status.module = module_;
  auto_mode_status.is_auto_mode = is_auto_mode_enabled_;

  pub_auto_mode_status_->publish(auto_mode_status);
}

void RTCInterface::updateCooperateStatus(
  const UUID & uuid, const bool safe, const uint8_t state, const double start_distance,
  const double finish_distance, const rclcpp::Time & stamp, const bool requested)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Find registered status which has same uuid
  auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](const auto & s) { return s.uuid == uuid; });

  // If there is no registered status, add it
  if (itr == registered_status_.statuses.end()) {
    CooperateStatus status;
    status.stamp = stamp;
    status.uuid = uuid;
    status.module = module_;
    status.safe = safe;
    status.requested = requested;
    status.command_status.type = Command::DEACTIVATE;
    status.state.type = State::WAITING_FOR_EXECUTION;
    status.start_distance = start_distance;
    status.finish_distance = finish_distance;
    status.auto_mode = is_auto_mode_enabled_;
    registered_status_.statuses.push_back(status);

    if (state != State::WAITING_FOR_EXECUTION)
      RCLCPP_WARN_STREAM(
        getLogger(), "[updateCooperateStatus]  Cannot register "
                       << state_to_string(state) << " as initial state" << std::endl);

    return;
  }

  auto update_status = [&](auto & status) {
    status.stamp = stamp;
    status.safe = safe;
    status.requested = requested;
    status.state.type = state;
    status.start_distance = start_distance;
    status.finish_distance = finish_distance;
  };

  // If the registered status is found, update status by considering the state transition
  if (
    itr->state.type == State::WAITING_FOR_EXECUTION &&
    (state == State::WAITING_FOR_EXECUTION || state == State::RUNNING || state == State::FAILED)) {
    update_status(*itr);
    return;
  }

  if (itr->state.type == State::RUNNING && state != State::WAITING_FOR_EXECUTION) {
    update_status(*itr);
    return;
  }

  if (itr->state.type == State::ABORTING && (state == State::ABORTING || state == State::FAILED)) {
    update_status(*itr);
    return;
  }

  if (itr->state.type == state) {
    update_status(*itr);
    return;
  }

  RCLCPP_WARN_STREAM(
    getLogger(), "[updateCooperateStatus] uuid : " << uuid_to_string(uuid)
                                                   << " cannot transit from "
                                                   << state_to_string(itr->state.type) << " to "
                                                   << state_to_string(state) << std::endl);
}

void RTCInterface::removeCooperateStatus(const UUID & uuid)
{
  std::lock_guard<std::mutex> lock(mutex_);
  removeStoredCommand(uuid);
  // Find registered status which has same uuid and erase it
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](const auto & s) { return s.uuid == uuid; });

  if (itr != registered_status_.statuses.end()) {
    registered_status_.statuses.erase(itr);
    return;
  }

  RCLCPP_WARN_STREAM(
    getLogger(),
    "[removeCooperateStatus] uuid : " << uuid_to_string(uuid) << " is not found." << std::endl);
}

void RTCInterface::removeStoredCommand(const UUID & uuid)
{
  // Find stored command which has same uuid and erase it
  const auto itr = std::find_if(
    stored_commands_.begin(), stored_commands_.end(),
    [uuid](const auto & s) { return s.uuid == uuid; });

  if (itr != stored_commands_.end()) {
    stored_commands_.erase(itr);
    return;
  }
}

void RTCInterface::removeExpiredCooperateStatus()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto itr = std::remove_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [this](const auto & status) { return (clock_->now() - status.stamp).seconds() > 10.0; });

  registered_status_.statuses.erase(itr, registered_status_.statuses.end());
}

void RTCInterface::clearCooperateStatus()
{
  std::lock_guard<std::mutex> lock(mutex_);
  registered_status_.statuses.clear();
  stored_commands_.clear();
}

bool RTCInterface::isActivated(const UUID & uuid) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](const auto & s) { return s.uuid == uuid; });

  if (itr != registered_status_.statuses.end()) {
    if (itr->state.type == State::FAILED || itr->state.type == State::SUCCEEDED) {
      return false;
    }
    if (itr->auto_mode && !itr->requested) {
      return itr->safe;
    }
    return itr->command_status.type == Command::ACTIVATE;
  }

  RCLCPP_WARN_STREAM(
    getLogger(), "[isActivated] uuid : " << uuid_to_string(uuid) << " is not found." << std::endl);
  return false;
}

bool RTCInterface::isForceActivated(const UUID & uuid) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](const auto & s) { return s.uuid == uuid; });

  if (itr != registered_status_.statuses.end()) {
    if (itr->state.type != State::WAITING_FOR_EXECUTION && itr->state.type != State::RUNNING) {
      return false;
    }
    if (itr->command_status.type != Command::ACTIVATE) return false;
    return !itr->safe || itr->requested;
  }

  RCLCPP_WARN_STREAM(
    getLogger(),
    "[isForceActivated] uuid : " << uuid_to_string(uuid) << " is not found" << std::endl);
  return false;
}

bool RTCInterface::isForceDeactivated(const UUID & uuid) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](const auto & s) { return s.uuid == uuid; });

  if (itr != registered_status_.statuses.end()) {
    if (itr->state.type != State::RUNNING) {
      return false;
    }
    return itr->command_status.type == Command::DEACTIVATE && itr->safe && !itr->auto_mode;
  }

  RCLCPP_WARN_STREAM(
    getLogger(),
    "[isForceDeactivated] uuid : " << uuid_to_string(uuid) << " is not found" << std::endl);
  return false;
}

bool RTCInterface::isRegistered(const UUID & uuid) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](const auto & s) { return s.uuid == uuid; });
  return itr != registered_status_.statuses.end();
}

bool RTCInterface::isRTCEnabled(const UUID & uuid) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](const auto & s) { return s.uuid == uuid; });

  if (itr != registered_status_.statuses.end()) {
    return !itr->auto_mode;
  }

  RCLCPP_WARN_STREAM(
    getLogger(), "[isRTCEnabled] uuid : " << uuid_to_string(uuid) << " is not found." << std::endl);
  return is_auto_mode_enabled_;
}

bool RTCInterface::isTerminated(const UUID & uuid) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](const auto & s) { return s.uuid == uuid; });

  if (itr != registered_status_.statuses.end()) {
    return itr->state.type == State::SUCCEEDED || itr->state.type == State::FAILED;
  }

  RCLCPP_WARN_STREAM(
    getLogger(), "[isTerminated] uuid : " << uuid_to_string(uuid) << " is not found." << std::endl);
  return is_auto_mode_enabled_;
}

void RTCInterface::lockCommandUpdate()
{
  is_locked_ = true;
}

void RTCInterface::unlockCommandUpdate()
{
  is_locked_ = false;
  updateCooperateCommandStatus(stored_commands_);
}

rclcpp::Logger RTCInterface::getLogger() const
{
  return logger_;
}

bool RTCInterface::isLocked() const
{
  return is_locked_;
}

void RTCInterface::print() const
{
  RCLCPP_INFO_STREAM(getLogger(), "---print rtc cooperate statuses---" << std::endl);
  for (const auto status : registered_status_.statuses) {
    RCLCPP_INFO_STREAM(
      getLogger(), "uuid:" << uuid_to_string(status.uuid)
                           << " command:" << command_to_string(status.command_status.type)
                           << std::boolalpha << " safe:" << status.safe
                           << " state:" << state_to_string(status.state.type) << std::endl);
  }
}

}  // namespace autoware::rtc_interface
