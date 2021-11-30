// Copyright 2020 Tier IV, Inc.
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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "vehicle_info_param_server/vehicle_info_param_server_core.hpp"

VehicleInfoParamServer::VehicleInfoParamServer()
: Node("vehicle_info_param_server")
{
  const double wheel_radius_m =
    this->declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE).get<double>();
  const double wheel_width_m =
    this->declare_parameter("wheel_width", rclcpp::PARAMETER_DOUBLE).get<double>();
  const double wheel_base_m =
    this->declare_parameter("wheel_base", rclcpp::PARAMETER_DOUBLE).get<double>();
  const double wheel_tread_m =
    this->declare_parameter("wheel_tread", rclcpp::PARAMETER_DOUBLE).get<double>();
  const double front_overhang_m =
    this->declare_parameter("front_overhang", rclcpp::PARAMETER_DOUBLE).get<double>();
  const double rear_overhang_m =
    this->declare_parameter("rear_overhang", rclcpp::PARAMETER_DOUBLE).get<double>();
  const double left_overhang_m =
    this->declare_parameter("left_overhang", rclcpp::PARAMETER_DOUBLE).get<double>();
  const double right_overhang_m =
    this->declare_parameter("right_overhang", rclcpp::PARAMETER_DOUBLE).get<double>();
  const double vehicle_height_m =
    this->declare_parameter("vehicle_height", rclcpp::PARAMETER_DOUBLE).get<double>();

  // create vehicle_info_params
  vehicle_info_params.emplace_back(rclcpp::Parameter("wheel_radius", wheel_radius_m));
  vehicle_info_params.emplace_back(rclcpp::Parameter("wheel_width", wheel_width_m));
  vehicle_info_params.emplace_back(rclcpp::Parameter("wheel_base", wheel_base_m));
  vehicle_info_params.emplace_back(rclcpp::Parameter("wheel_tread", wheel_tread_m));
  vehicle_info_params.emplace_back(rclcpp::Parameter("front_overhang", front_overhang_m));
  vehicle_info_params.emplace_back(rclcpp::Parameter("rear_overhang", rear_overhang_m));
  vehicle_info_params.emplace_back(rclcpp::Parameter("left_overhang", left_overhang_m));
  vehicle_info_params.emplace_back(rclcpp::Parameter("right_overhang", right_overhang_m));
  vehicle_info_params.emplace_back(rclcpp::Parameter("vehicle_height", vehicle_height_m));
  vehicle_info_params.emplace_back(rclcpp::Parameter("ready_vehicle_info_param", true));

  // timer
  while (rclcpp::ok()) {
    setVehicleInfoParameters();
    rclcpp::Rate(1.0).sleep();
  }
}

void VehicleInfoParamServer::setVehicleInfoParameters()
{
  std::vector<std::string> node_names = get_node_names();

  // remove self node
  node_names.erase(
    std::remove_if(
      node_names.begin(), node_names.end(),
      [](std::string s) {
        return s.find(std::string("vehicle_info_param_server")) != std::string::npos;
      }),
    node_names.end());

  for (const auto & n : node_names) {
    // remove nodes that already checked
    if (std::find(set_node_list_.begin(), set_node_list_.end(), n) != set_node_list_.end()) {
      continue;
    }

    if(n.find("transform_listener_impl") != std::string::npos) {
      continue;
    }

    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      break;
    }

    // wait for parameter service
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, n);
    using namespace std::chrono_literals;
    if (!parameters_client->wait_for_service(100ms)) {
      // cannot access to parameter service
      continue;
    }

    // Check that the node has vehicle_info client or not
    bool has_param = true;
    if (
      !hasParameter(
        parameters_client, "ready_vehicle_info_param", request_timeout_sec_, &has_param) ||
      !has_param)
    {
      // No need to set vehicle parameter.
      if (!has_param) {set_node_list_.emplace_back(n);}
      continue;
    }

    // Check that the vehicle_info_params are already set to the node or not.
    bool ready_vehicle_info_param = false;
    if (
      !getParameter<bool>(
        parameters_client, "ready_vehicle_info_param", request_timeout_sec_,
        &ready_vehicle_info_param) ||
      ready_vehicle_info_param)
    {
      // Already vehicle_info_params are already set.
      if (ready_vehicle_info_param) {set_node_list_.emplace_back(n);}
      continue;
    }

    // Set Parameter
    while (!setParameter(parameters_client, request_timeout_sec_, vehicle_info_params)) {
      rclcpp::Rate(100.0).sleep();
    }
    set_node_list_.emplace_back(n);
    RCLCPP_INFO_STREAM(this->get_logger(), "Set vehicle info parameter: " << n);
  }
}
