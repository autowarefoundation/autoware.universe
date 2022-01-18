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

#ifndef PLANNING_MANAGER__MODULE_CORE_CPP_
#define PLANNING_MANAGER__MODULE_CORE_CPP_

#include "planning_manager/msg/planning_data.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

enum class Status : int {
  Ready = 0,
  Executing,
  JustFinished,
  Finished,
};

namespace
{
template <typename T>
void waitForService(
  const typename rclcpp::Client<T>::SharedPtr client, const std::string & service_name)
{
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("planning_manager"),
      "waiting for " << service_name << " service connection...");
  }
}
}  // namespace

namespace planning_manager
{
using planning_manager::msg::PlanningData;

template <typename PlanService, typename ValidateService>
class Module
{
public:
  Module(
    rclcpp::Node * node_ptr, const std::string & service_name, const bool is_showing_debug_info)
  : service_name_(service_name), is_showing_debug_info_(is_showing_debug_info)
  {
    callback_group_service_ =
      node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // client for planning
    const std::string plan_service_name = service_name_ + "plan";
    client_plan_ = node_ptr->create_client<PlanService>(
      plan_service_name, rmw_qos_profile_services_default, callback_group_service_);
    waitForService<PlanService>(client_plan_, plan_service_name);

    // client for validation
    const std::string validate_service_name = service_name_ + "validate";
    client_validate_ = node_ptr->create_client<ValidateService>(
      validate_service_name + "validate", rmw_qos_profile_services_default,
      callback_group_service_);
    waitForService<ValidateService>(client_validate_, validate_service_name);
  }

  template <typename T>
  void plan(const int id, const T pre_module_result, const PlanningData & planning_data)
  {
    // RCLCPP_INFO_EXPRESSION(
    //   get_logger(), is_showing_debug_info_, "%d / %ld: start BehaviorVelocityPlannerPlan", id,
    //   modules_result_map_.size());
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("planning_manager"), is_showing_debug_info_, "start %s plan",
      service_name_);

    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      plan_status_ = Status::Executing;
    }

    auto request = std::make_shared<PlanService>();
    request->input = pre_module_result;
    request->planning_data = planning_data;

    client_plan_->async_send_request(
      // request, [this, id](rclcpp::Client<Module::PlanService>::SharedFuture future) {
      request, [this, id](auto future) {
        // RCLCPP_INFO_EXPRESSION(
        //  get_logger(), is_showing_debug_info_, "%d / %ld: get BehaviorPathPlannerPlan", id,
        //  modules_result_map_.size());

        RCLCPP_INFO_EXPRESSION(
          rclcpp::get_logger("planning_manager"), is_showing_debug_info_, "get %s plan",
          service_name_);

        const auto response = future.get();

        std::lock_guard<std::mutex> lock(plan_mutex_);
        plan_result_map_[id] = response;
        plan_status_ = Status::Finished;
      });
  }

  void validate() {}

  void removeResultMap(const int id)
  {
    plan_result_map_.erase(id);
    validate_result_map_.erase(id);
  }

  // getter
  PlanService getPlanResult(const int id) const
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    return plan_result_map_[id];
  }
  Status getPlanStatus() const
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    return plan_status_;
  }
  ValidateService getValidateResult(const int id) const
  {
    std::lock_guard<std::mutex> lock(validate_mutex_);
    return validate_result_map_[id];
  }
  Status getValidateStatus() const
  {
    std::lock_guard<std::mutex> lock(validate_mutex_);
    return validate_status_;
  }

  // setter
  void setPlanStatus(const Status status)
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    plan_status_ = status;
  }
  void setValidateStatus(const Status status)
  {
    std::lock_guard<std::mutex> lock(validate_mutex_);
    validate_status_ = status;
  }

private:
  const std::string service_name_;
  const bool is_showing_debug_info_;

  rclcpp::CallbackGroup::SharedPtr callback_group_service_;
  typename rclcpp::Client<PlanService>::SharedPtr client_plan_;
  typename rclcpp::Client<ValidateService>::SharedPtr client_validate_;

  std::mutex plan_mutex_;
  std::unordered_map<int, typename PlanService::Response> plan_result_map_;
  Status plan_status_;

  std::mutex validate_mutex_;
  std::unordered_map<int, typename ValidateService::Response> validate_result_map_;
  Status validate_status_;
};
}  // namespace planning_manager

#endif  // PLANNING_MANAGER__MODULE_CORE_CPP_
