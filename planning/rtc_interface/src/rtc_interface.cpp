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

#include "rtc_interface/rtc_interface.hpp"

namespace
{
std::string to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +uuid.uuid[i];
  }
  return ss.str();
}
}  // namespace

namespace rtc_interface
{
RTCInterface::RTCInterface(rclcpp::Node & node, const std::string & name, const Module & module)
: clock_{*node.get_clock()},
  logger_{node.get_logger().get_child("RTCInterface[" + name + "]")},
  module_{module}
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Publisher
  pub_statuses_ = node.create_publisher<CooperateStatusArray>("~/" + name + "/cooperate_status", 1);

  // Service
  srv_commands_ = node.create_service<CooperateCommands>(
    "~/" + name + "/cooperate_commands",
    std::bind(&RTCInterface::onCooperateCommandService, this, _1, _2));
}

void RTCInterface::publishCooperateStatus()
{
  registered_status_.stamp = clock_.now();
  pub_statuses_->publish(registered_status_);
}

void RTCInterface::onCooperateCommandService(
  const CooperateCommands::Request::SharedPtr request,
  const CooperateCommands::Response::SharedPtr responses)
{
  for (const auto & command : request->commands) {
    CooperateResponse response;
    response.uuid = command.uuid;
    response.module = command.module;

    const auto itr = std::find_if(
      registered_status_.statuses.begin(), registered_status_.statuses.end(),
      [command](auto & s) { return s.uuid == command.uuid; });

    // Update command if the command has been already received
    if (itr != registered_status_.statuses.end()) {
      itr->command_status = command.command;
      response.success = true;
    } else {
      RCLCPP_WARN_STREAM(
        getLogger(), "[onCooperateCommandService] uuid : " << to_string(command.uuid)
                                                           << " is not found." << std::endl);
      response.success = false;
    }
    responses->responses.push_back(response);
  }
}

void RTCInterface::updateCooperateStatus(const UUID & uuid, const bool safe, const double distance)
{
  // Find registered status which has same uuid
  auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](auto & s) { return s.uuid == uuid; });

  // If there is no registered status, add it
  if (itr == registered_status_.statuses.end()) {
    CooperateStatus status;
    status.stamp = clock_.now();
    status.uuid = uuid;
    status.module = module_;
    status.safe = safe;
    status.command_status.type = Command::DEACTIVATE;
    status.distance = distance;
    registered_status_.statuses.push_back(status);
    return;
  }

  // If the registered status is found, update status
  itr->stamp = clock_.now();
  itr->safe = safe;
  itr->distance = distance;
}

void RTCInterface::removeCooperateStatus(const UUID & uuid)
{
  // Find registered status which has same uuid and erase it
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](auto & s) { return s.uuid == uuid; });

  if (itr != registered_status_.statuses.end()) {
    registered_status_.statuses.erase(itr);
    return;
  }

  RCLCPP_WARN_STREAM(
    getLogger(),
    "[removeCooperateStatus] uuid : " << to_string(uuid) << " is not found." << std::endl);
}

bool RTCInterface::isActivated(const UUID & uuid) const
{
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](auto & s) { return s.uuid == uuid; });

  if (itr != registered_status_.statuses.end()) {
    return itr->command_status.type == Command::ACTIVATE;
  }

  RCLCPP_WARN_STREAM(
    getLogger(), "[isActivated] uuid : " << to_string(uuid) << " is not found." << std::endl);
  return false;
}

rclcpp::Logger RTCInterface::getLogger() const { return logger_; }

}  // namespace rtc_interface
