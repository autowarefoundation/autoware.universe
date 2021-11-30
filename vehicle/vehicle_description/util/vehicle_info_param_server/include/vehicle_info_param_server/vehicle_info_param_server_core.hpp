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

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#ifndef VEHICLE_INFO_PARAM_SERVER__VEHICLE_INFO_PARAM_SERVER_CORE_HPP_
#define VEHICLE_INFO_PARAM_SERVER__VEHICLE_INFO_PARAM_SERVER_CORE_HPP_

class VehicleInfoParamServer : public rclcpp::Node
{
public:
  VehicleInfoParamServer();

private:
  void setVehicleInfoParameters();
  bool hasParameter(
    const rclcpp::AsyncParametersClient::SharedPtr client, const std::string & param_name,
    const double timeout, bool * result)
  {
    std::vector<std::string> params;
    params.emplace_back(param_name);
    auto list_param = client->list_parameters(params, 1);

    const auto timeout_chrono = std::chrono::duration<double>(timeout);
    using rclcpp::spin_until_future_complete;
    if (
      spin_until_future_complete(this->get_node_base_interface(), list_param, timeout_chrono) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto vars = list_param.get();
      *result = vars.names.size() > 0;
      return true;
    }
    // timeout
    RCLCPP_DEBUG(get_logger(), "Timeout: hasParameter()");
    return false;
  }

  bool setParameter(
    const rclcpp::AsyncParametersClient::SharedPtr client, const double timeout,
    std::vector<rclcpp::Parameter> params)
  {
    auto set_parameters_results = client->set_parameters(params);
    const auto timeout_chrono = std::chrono::duration<double>(timeout);
    using rclcpp::spin_until_future_complete;
    return spin_until_future_complete(
      this->get_node_base_interface(), set_parameters_results, timeout_chrono) ==
           rclcpp::FutureReturnCode::SUCCESS ||
           set_parameters_results.get().front().successful;
  }

  template<class T>
  bool getParameter(
    const rclcpp::AsyncParametersClient::SharedPtr client, const std::string & param_name,
    const double timeout, T * result)
  {
    std::vector<std::string> params;
    params.emplace_back(param_name);
    auto get_param = client->get_parameters(params);

    const auto timeout_chrono = std::chrono::duration<double>(timeout);
    using rclcpp::spin_until_future_complete;
    if (
      spin_until_future_complete(this->get_node_base_interface(), get_param, timeout_chrono) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto vars = get_param.get();
      *result = vars.front().get_value<T>();
      return true;
    }
    // timeout
    RCLCPP_DEBUG(get_logger(), "Timeout: getParameter()");
    return false;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Parameter> vehicle_info_params;
  std::vector<std::string> set_node_list_;
  double request_timeout_sec_ = 0.2;
};

#endif  // VEHICLE_INFO_PARAM_SERVER__VEHICLE_INFO_PARAM_SERVER_CORE_HPP_
