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

#include <chrono>
#include <random>
#include <string>

namespace
{
rclcpp::SubscriptionOptions createSubscriptionOptions(
  rclcpp::Node * node_ptr,
  const rclcpp::CallbackGroupType & group_type = rclcpp::CallbackGroupType::MutuallyExclusive)
{
  rclcpp::CallbackGroup::SharedPtr callback_group = node_ptr->create_callback_group(group_type);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}

rclcpp::CallbackGroup::SharedPtr createNewCallbackGroup(
  rclcpp::Node * node_ptr, std::vector<rclcpp::CallbackGroup::SharedPtr> & group_ptr_vec,
  const rclcpp::CallbackGroupType & group_type = rclcpp::CallbackGroupType::MutuallyExclusive)
{
  group_ptr_vec.push_back(node_ptr->create_callback_group(group_type));
  return group_ptr_vec.back();
}

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

template <typename T>
typename T::Response::SharedPtr sendRequest(
  typename rclcpp::Client<T>::SharedPtr client, typename T::Request::SharedPtr request,
  const int timeout_s = 3)
{
  const auto response = client->async_send_request(request);
  if (response.wait_for(std::chrono::seconds(timeout_s)) != std::future_status::ready) {
    return nullptr;
  }
  return response.get();
}
}  // namespace

namespace planning_manager
{
PlanningManagerNode::PlanningManagerNode(const rclcpp::NodeOptions & node_options)
: Node("planning_manager", node_options)
{
  using std::placeholders::_1;

  // parameter
  is_showing_debug_info_ = true;
  planning_hz_ =
    declare_parameter<double>("planning_hz", 10.0);  // TODO(murooka) remove default parameter
  const double main_hz =
    declare_parameter<double>("main_hz", 1000.0);  // TODO(murooka) remove default parameter

  // publisher
  traj_pub_ = this->create_publisher<Trajectory>("~/output/trajectory", 1);

  // subscriber
  route_sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::HADMapRoute>(
    "~/input/route", 1, std::bind(&PlanningManagerNode::onRoute, this, _1),
    createSubscriptionOptions(this));

  {  // getter subscriber for planning data
    vector_map_sub_ = create_subscription<HADMapBin>(
      "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
      std::bind(&PlanningManagerNode::onMap, this, _1), createSubscriptionOptions(this));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odometry", 1, std::bind(&PlanningManagerNode::onOdometry, this, _1),
      createSubscriptionOptions(this));
    predicted_objects_sub_ =
      create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
        "~/input/predicted_objects", 1,
        std::bind(&PlanningManagerNode::onPredictedObjects, this, _1),
        createSubscriptionOptions(this));
    external_approval_sub_ = create_subscription<Approval>(
      "~/input/external_approval", 1, std::bind(&PlanningManagerNode::onExternalApproval, this, _1),
      createSubscriptionOptions(this));
    force_approval_sub_ = create_subscription<PathChangeModule>(
      "~/input/force_approval", 1, std::bind(&PlanningManagerNode::onForceApproval, this, _1),
      createSubscriptionOptions(this));
  }

  {  // service client
    // behavior path planner
    client_behavior_path_planner_plan_ =
      this->create_client<planning_manager::srv::BehaviorPathPlannerPlan>(
        "~/srv/behavior_path_planner/plan", rmw_qos_profile_services_default,
        createNewCallbackGroup(this, callback_group_service_vec_));
    waitForService<planning_manager::srv::BehaviorPathPlannerPlan>(
      client_behavior_path_planner_plan_, "behavior_path_planner/plan");

    client_behavior_path_planner_validate_ =
      this->create_client<planning_manager::srv::BehaviorPathPlannerValidate>(
        "~/srv/behavior_path_planner/validate", rmw_qos_profile_services_default,
        createNewCallbackGroup(this, callback_group_service_vec_));
    waitForService<planning_manager::srv::BehaviorPathPlannerValidate>(
      client_behavior_path_planner_validate_, "behavior_path_planner/validate");

    // behavior velocity planner
    client_behavior_velocity_planner_plan_ =
      this->create_client<planning_manager::srv::BehaviorVelocityPlannerPlan>(
        "~/srv/behavior_velocity_planner/plan", rmw_qos_profile_services_default,
        createNewCallbackGroup(this, callback_group_service_vec_));
    waitForService<planning_manager::srv::BehaviorVelocityPlannerPlan>(
      client_behavior_velocity_planner_plan_, "behavior_velocity_planner/plan");

    client_behavior_velocity_planner_validate_ =
      this->create_client<planning_manager::srv::BehaviorVelocityPlannerValidate>(
        "~/srv/behavior_velocity_planner/validate", rmw_qos_profile_services_default,
        createNewCallbackGroup(this, callback_group_service_vec_));
    waitForService<planning_manager::srv::BehaviorVelocityPlannerValidate>(
      client_behavior_velocity_planner_validate_, "behavior_velocity_planner/validate");
  }

  {  // timer
    callback_group_timer_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    const auto period = rclcpp::Rate(main_hz).period();
    auto on_timer = std::bind(&PlanningManagerNode::run, this);
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer)>>(
      this->get_clock(), period, std::move(on_timer),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, callback_group_timer_);
  }
}

void PlanningManagerNode::onRoute(
  const autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr msg)
{
  route_ = msg;
}

void PlanningManagerNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(planning_data_mutex_);
  planning_data_.ego_odom = *msg;
}

void PlanningManagerNode::onPredictedObjects(const PredictedObjects::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(planning_data_mutex_);
  planning_data_.predicted_objects = *msg;
}

void PlanningManagerNode::onExternalApproval(const Approval::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(planning_data_mutex_);
  planning_data_.external_approval = *msg;
}

void PlanningManagerNode::onForceApproval(const PathChangeModule::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(planning_data_mutex_);
  planning_data_.force_approval = *msg;
}

void PlanningManagerNode::onMap(const HADMapBin::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(planning_data_mutex_);
  planning_data_.had_map_bin = *msg;
}

void PlanningManagerNode::run()
{
  if (!route_) {
    RCLCPP_INFO_EXPRESSION(get_logger(), is_showing_debug_info_, "waiting for route");
    return;
  }

  planning_data_mutex_.lock();
  // TODO(murooka) wait for planning data to be ready in a reasonable way
  planning_data_.header.stamp = this->now();
  while (planning_data_.had_map_bin.data.empty() && rclcpp::ok()) {
    rclcpp::Rate(100).sleep();
  }

  // NOTE: planning_data must not be referenced for multithreading
  const auto planning_data = planning_data_;
  planning_data_mutex_.unlock();

  planTrajectory(*route_, planning_data);
  optimizeVelocity(planning_data);

  // validateTrajectory(traj_with_optimal_vel, planning_data);

  publishTrajectory();
  publishDiagnostics();

  removeFinishedMap();
}

void PlanningManagerNode::planTrajectory(
  const HADMapRoute & route, const PlanningData & planning_data)
{
  // RCLCPP_INFO_EXPRESSION(get_logger(), is_showing_debug_info_, "start planTrajectory");
  if ((rclcpp::Clock().now() - prev_plan_time_).seconds() > 1.0 / planning_hz_) {
    prev_plan_time_ = rclcpp::Clock().now();

    // TODO(murooka)
    current_id_++;
    const int unique_id = current_id_;
    {
      // std::lock_guard<std::mutex> lock(map_mutex_);
      modules_result_map_[unique_id] = ModulesResult();
      modules_result_map_[unique_id].behavior_path_planner.status = Status::WAITING;
    }

    RCLCPP_INFO_EXPRESSION(
      get_logger(), is_showing_debug_info_, "%d / %ld: start new planning", unique_id,
      modules_result_map_.size());

    if (current_id_ == 10000) {
      current_id_ = 0;
    }
  }

  for (auto itr = modules_result_map_.begin(); itr != modules_result_map_.end(); ++itr) {
    const int id = itr->first;
    // RCLCPP_INFO_EXPRESSION(
    //   get_logger(), is_showing_debug_info_, "%d / %ld", id, modules_result_map_.size());

    // behavior path planner
    if (itr->second.behavior_path_planner.status == Status::WAITING) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), is_showing_debug_info_, "%d / %ld: start BehaviorPathPlannerPlan", id,
        modules_result_map_.size());
      itr->second.behavior_path_planner.status = Status::EXECUTING;

      auto behavior_path_request =
        std::make_shared<planning_manager::srv::BehaviorPathPlannerPlan::Request>();
      behavior_path_request->route = route;
      behavior_path_request->planning_data = planning_data;

      client_behavior_path_planner_plan_->async_send_request(
        behavior_path_request,
        [this, itr, id](rclcpp::Client<BehaviorPathPlannerPlan>::SharedFuture future) {
          // std::lock_guard<std::mutex> lock(mutex_);
          RCLCPP_INFO_EXPRESSION(
            get_logger(), is_showing_debug_info_, "%d / %ld: get BehaviorPathPlannerPlan", id,
            modules_result_map_.size());
          const auto response = future.get();
          itr->second.behavior_path_planner.result = response->path_with_lane_id;
          itr->second.behavior_path_planner.status = Status::FINISHED;
          itr->second.behavior_velocity_planner.status = Status::WAITING;
        });
    }

    // behavior velocity planner
    if (
      itr->second.behavior_path_planner.status == Status::FINISHED &&
      itr->second.behavior_velocity_planner.status == Status::WAITING) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), is_showing_debug_info_, "%d / %ld: start BehaviorVelocityPlannerPlan", id,
        modules_result_map_.size());
      itr->second.behavior_velocity_planner.status = Status::EXECUTING;

      auto behavior_velocity_request =
        std::make_shared<planning_manager::srv::BehaviorVelocityPlannerPlan::Request>();
      behavior_velocity_request->path_with_lane_id = itr->second.behavior_path_planner.result;
      behavior_velocity_request->planning_data = planning_data;

      client_behavior_velocity_planner_plan_->async_send_request(
        behavior_velocity_request,
        [this, itr, id](rclcpp::Client<BehaviorVelocityPlannerPlan>::SharedFuture future) {
          // std::lock_guard<std::mutex> lock(mutex_);
          RCLCPP_INFO_EXPRESSION(
            get_logger(), is_showing_debug_info_, "%d / %ld: get BehaviorVelocityPlannerPlan", id,
            modules_result_map_.size());
          const auto response = future.get();
          itr->second.behavior_velocity_planner.result = response->path;
          itr->second.behavior_velocity_planner.status = Status::FINISHED;
        });
    }
  }
}

void PlanningManagerNode::optimizeVelocity([[maybe_unused]] const PlanningData & planning_data)
{
  // TODO(murooka) implement the same loop as planTrajectory
  // RCLCPP_INFO_EXPRESSION(get_logger(), is_showing_debug_info_, "start optimizeVelocity");
}

void PlanningManagerNode::validateTrajectory(
  [[maybe_unused]] const Trajectory & traj, [[maybe_unused]] const PlanningData & planning_data)
{
  // RCLCPP_INFO_EXPRESSION(get_logger(), is_showing_debug_info_, "start validateTrajectory");

  // behavior path planner
  // RCLCPP_INFO_EXPRESSION(get_logger(), is_showing_debug_info_, "start
  // BehaviorPathPlannerValidate");

  auto behavior_path_request =
    std::make_shared<planning_manager::srv::BehaviorPathPlannerValidate::Request>();
  behavior_path_request->trajectory = traj;

  const auto behavior_path_planner_validate_result = sendRequest<BehaviorPathPlannerValidate>(
    client_behavior_path_planner_validate_, behavior_path_request);

  // behavior velocity planner
  // RCLCPP_INFO_EXPRESSION(
  //   get_logger(), is_showing_debug_info_, "start BehaviorVelocityPlannerValidate");

  auto behavior_velocity_request =
    std::make_shared<planning_manager::srv::BehaviorVelocityPlannerValidate::Request>();
  behavior_velocity_request->trajectory = traj;

  const auto behavior_velocity_planner_validate_result =
    sendRequest<BehaviorVelocityPlannerValidate>(
      client_behavior_velocity_planner_validate_, behavior_velocity_request);
}

void PlanningManagerNode::publishTrajectory()
{
  // RCLCPP_INFO_EXPRESSION(get_logger(), is_showing_debug_info_, "start publishTrajectory");

  // TODO(murooka) publish trajectory where all validates are finished, and remove the id from map
  // traj_pub_->publish(traj);
}
void PlanningManagerNode::publishDiagnostics() {}

void PlanningManagerNode::removeFinishedMap()
{
  // TODO(murooka) use remove_if
  // TODO(murooka) finally move this part to publishTrajectory or create removeFinishedMap
  auto itr = modules_result_map_.begin();
  while (itr != modules_result_map_.end()) {
    if (itr->second.behavior_velocity_planner.status == Status::FINISHED) {
      const int id = itr->first;

      RCLCPP_INFO_EXPRESSION(
        get_logger(), is_showing_debug_info_, "%d / %ld: remove planning", id,
        modules_result_map_.size());
      itr = modules_result_map_.erase(itr);
    } else {
      itr++;
    }
  }
}

}  // namespace planning_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_manager::PlanningManagerNode)
