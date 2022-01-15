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

#include "planning_manager/planning_manager_node.hpp"

#include <string>

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
      "Waiting for " << service_name << " service connection...");
  }
}
}  // namespace

namespace planning_manager
{
PlanningManagerNode::PlanningManagerNode(const rclcpp::NodeOptions & node_options)
: Node("planning_manager", node_options)
{
  // subscriber
  using std::placeholders::_1;
  route_sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::HADMapRoute>(
    "~/input/route", 1, std::bind(&PlanningManagerNode::onRoute, this, _1));

  {  // client
    const auto callback_group_services =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    client_behavior_path_planner_plan_ =
      this->create_client<planning_manager::srv::BehaviorPathPlannerPlan>(
        "~/srv/behavior_path_planner/plan", rmw_qos_profile_services_default,
        callback_group_services);
    waitForService<planning_manager::srv::BehaviorPathPlannerPlan>(
      client_behavior_path_planner_plan_, "behavior_path_planner/plan");

    client_behavior_path_planner_validate_ =
      this->create_client<planning_manager::srv::BehaviorPathPlannerValidate>(
        "~/srv/behavior_path_planner/validate", rmw_qos_profile_services_default,
        callback_group_services);
    waitForService<planning_manager::srv::BehaviorPathPlannerValidate>(
      client_behavior_path_planner_validate_, "behavior_path_planner/validate");

    // TODO(murooka) add other clients
  }

  {  // timer
    const double planning_hz =
      declare_parameter<double>("planning_hz", 10.0);  // TODO(murooka) remove default parameter
    const auto period = rclcpp::Rate(planning_hz).period();
    auto on_timer = std::bind(&PlanningManagerNode::run, this);
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer)>>(
      this->get_clock(), period, std::move(on_timer),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, nullptr);
  }
}

void PlanningManagerNode::onRoute(
  const autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr msg)
{
  route_ = msg;
}

void PlanningManagerNode::run()
{
  if (!route_) {
    return;
  }

  // TODO(murooka) prepare planning data
  planning_data_.header.stamp = this->now();

  planTrajectory();

  optimizeVelocity();

  validateTrajectory();
}

void PlanningManagerNode::planTrajectory()
{
  {  // behavior path planner
    auto request = std::make_shared<planning_manager::srv::BehaviorPathPlannerPlan::Request>();
    request->route = *route_;
    request->planning_data = planning_data_;

    client_behavior_path_planner_plan_->async_send_request(
      request,
      [this](rclcpp::Client<planning_manager::srv::BehaviorPathPlannerPlan>::SharedFuture result) {
        const auto response = result.get();
        path_with_lane_id_ = response->path_with_lane_id;
      });
    // TODO(murooka) add wait function here?
  }

  // TODO(murooka) add other services
}

// TODO(murooka) optimize velocity
void PlanningManagerNode::optimizeVelocity() {}

// TODO(murooka) validate
void PlanningManagerNode::validateTrajectory()
{
  {  // behavior path planner
    auto request = std::make_shared<planning_manager::srv::BehaviorPathPlannerValidate::Request>();
    request->trajectory = motion_trajectory_;

    client_behavior_path_planner_validate_->async_send_request(
      request,
      [this](
        rclcpp::Client<planning_manager::srv::BehaviorPathPlannerValidate>::SharedFuture result) {
        const auto response = result.get();
        [[maybe_unused]] const bool status = response->status.data;
      });
  }
}

}  // namespace planning_manager
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_manager::PlanningManagerNode)
