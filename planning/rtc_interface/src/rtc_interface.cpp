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

namespace rtc_interface
{
RTCInterface::RTCInterface(rclcpp::Node & node, const std::string & name, const Module & module)
: clock_{*node.get_clock()}, module_(module)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Publisher
  pub_status_ = node.create_publisher<CooperateStatusArray>(name + "/status", 1);

  // Service
  srv_command_ = node.create_service<CooperateCommand>(
    name + "/command", std::bind(&RTCInterface::onCooperateCommandService, this, _1, _2));
}

void RTCInterface::publishCooperateStatus()
{
  registered_status_.stamp = clock_.now();
  pub_status_->publish(registered_status_);
}

void RTCInterface::onCooperateCommandService(
  const CooperateCommand::Request::SharedPtr request,
  const CooperateCommand::Response::SharedPtr response)
{
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [request](auto & s) { return s.uuid == request->uuid; });

  // Update command if the command has been already received
  if (itr != registered_status_.statuses.end()) {
    itr->command_status = request->command;
    response->success = true;
    return;
  }

  // Status with same uuid is not registered
  response->success = false;

  return;
}

void RTCInterface::updateCooperateStatus(const UUID & uuid, const bool safe, const double distance)
{
  // Find registered status which has same uuid
  auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](auto & s) { return s.uuid == uuid; });

  // If there is no registered status, add it.
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

  std::cout << "Something wrong" << std::endl;
}

bool RTCInterface::isActivated(const UUID & uuid) const
{
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](auto & s) { return s.uuid == uuid; });

  if (itr != registered_status_.statuses.end()) {
    return itr->command_status.type == Command::ACTIVATE;
  }

  std::cout << "Something wrong" << std::endl;
  return false;
}

}  // namespace rtc_interface
