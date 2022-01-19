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
}  // namespace

namespace planning_manager
{
PlanningManagerNode::PlanningManagerNode(const rclcpp::NodeOptions & node_options)
: Node("planning_manager", node_options)
{
  using std::placeholders::_1;

  // parameter
  is_showing_debug_info_ = declare_parameter<double>(
    "is_showing_debug_info", true);  // TODO(murooka) remove default parameter
  planning_hz_ =
    declare_parameter<double>("planning_hz", 10.0);  // TODO(murooka) remove default parameter
  const double main_hz =
    declare_parameter<double>("main_hz", 100.0);  // TODO(murooka) remove default parameter

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
    modules_ = {
      new Module<BehaviorPathPlannerPlan, BehaviorPathPlannerValidate>(
        this, "behavior_path_module", is_showing_debug_info_),
      new Module<BehaviorVelocityPlannerPlan, BehaviorVelocityPlannerValidate>(
        this, "behavior_velocity_module", is_showing_debug_info_)
    };
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
  validateTrajectory(planning_data);

  publishTrajectory();
  publishDiagnostics();

  removeFinishedMap();
}

void PlanningManagerNode::planTrajectory(
  const HADMapRoute & route, const PlanningData & planning_data)
{
  // RCLCPP_INFO_EXPRESSION(get_logger(), is_showing_debug_info_, "start planTrajectory");

  // spawn new planning in a certain hz (= planning_hz)
  if ((rclcpp::Clock().now() - prev_plan_time_).seconds() > 1.0 / planning_hz_) {
    prev_plan_time_ = rclcpp::Clock().now();

    // TODO(murooka)
    current_id_++;
    const int unique_id = current_id_;
    {
      planning_num_.push_back(unique_id);
      // static_cast<decltype(modules_.front())>(modules_.front()).setPlanStatus(Status::Ready);
      // static_cast<decltype(modules_.front())>(modules_.front()).setPlanStatus(Status::Ready);
      // static_cast<Module<BehaviorPathPlannerPlan, BehaviorPathPlannerValidate>>(modules_.front()).setPlanStatus(Status::Ready);
      dynamic_cast<Module<BehaviorPathPlannerPlan, BehaviorPathPlannerValidate>*>(modules_.front())->setPlanStatus(Status::Ready);
      // dynamic_cast<decltype(*modules_.front())*>(modules_.front())->setPlanStatus(Status::Ready);

      // auto & module = std::any_cast<Module<BehaviorPathPlannerPlan, BehaviorPathPlannerValidate>>(modules_.front());
      // Module<std::any_cast<module>, std::any_cast<module>>(module)
      // const auto module = std::any_cast<Module<BehaviorPathPlannerPlan, BehaviorPathPlannerValidate>>(modules_.front());
      // module.setPlanStatus(Status::Ready);
    }

    RCLCPP_INFO_EXPRESSION(
      get_logger(), is_showing_debug_info_, "%d / %ld: start new planning", unique_id,
      planning_num_.size());

    if (current_id_ == 10000) {
      current_id_ = 0;
    }
  }

  for (const int id : planning_num_) {
    // plan
    for (auto m_itr = modules_.begin(); m_itr != modules_.end(); ++m_itr) {
      if ((*m_itr)->getPlanStatus() == Status::Ready) {
        const auto pre_mdule_result =
          (m_itr == modules_.begin()) ? route : *(m_itr - 1)->getPlanRresult(id)->output;
        *m_itr->plan(id, pre_module_result, planning_data);
      }
    }

    // check planning
    for (auto m_itr = modules_.begin(); m_itr != modules_.end(); ++m_itr) {
      if (*m_itr->getPlanStatus() == Status::JustFinished) {
        *m_itr->setPlanStatus(Status::Finished);

        if (m_itr == modules_.end() - 1) {  // last module
          modules_.begin()->setValidateStatus(Status::Ready);
        } else {  // not last module
          *(m_itr + 1)->setPlanStatus(Status::Ready);
        }
      }
    }

    // TODO(murooka) validate
  }
}

void PlanningManagerNode::optimizeVelocity([[maybe_unused]] const PlanningData & planning_data)
{
  // TODO(murooka) implement the same loop as planTrajectory
  // RCLCPP_INFO_EXPRESSION(get_logger(), is_showing_debug_info_, "start optimizeVelocity");
}

void PlanningManagerNode::validateTrajectory([[maybe_unused]] const PlanningData & planning_data)
{
  /*
    std::lock_guard<std::mutex> lock(map_mutex_);
    // RCLCPP_INFO_EXPRESSION(get_logger(), is_showing_debug_info_, "start validateTrajectory");

    for (auto itr = modules_result_map_.begin(); itr != modules_result_map_.end(); ++itr) {
      const int id = itr->first;

      // if (itr->second.motion_velocity_smoother_plan.status == Status::FINISHED) { //
    TODO(murooka) if (itr->second.behavior_velocity_planner_plan.status == Status::FINISHED) {
        // behavior path planner
        if (itr->second.behavior_velocity_planner_validate.status == Status::WAITING) {
          RCLCPP_INFO_EXPRESSION(
            get_logger(), is_showing_debug_info_, "%d / %ld: start BehaviorPathPlannerValidate", id,
            modules_result_map_.size());

          itr->second.behavior_path_planner_validate.status = Status::EXECUTING;

          auto request =
            std::make_shared<planning_manager::srv::BehaviorPathPlannerValidate::Request>();
          request->trajectory = itr->second.motion_velocity_smoother_plan.result;

          client_behavior_path_planner_validate_->async_send_request(
            request,
            [this, itr, id](rclcpp::Client<BehaviorPathPlannerValidate>::SharedFuture future) {
              std::lock_guard<std::mutex> lock(map_mutex_);
              RCLCPP_INFO_EXPRESSION(
                get_logger(), is_showing_debug_info_, "%d / %ld: get BehaviorPathPlannerValidate",
    id, modules_result_map_.size()); const auto response = future.get();
              itr->second.behavior_path_planner_validate.status = Status::FINISHED;
            });
        }

        // behavior velocity planner
        if (itr->second.behavior_velocity_planner_validate.status == Status::WAITING) {
          RCLCPP_INFO_EXPRESSION(
            get_logger(), is_showing_debug_info_, "%d / %ld: start BehaviorVelocityPlannerValidate",
            id, modules_result_map_.size());

          itr->second.behavior_velocity_planner_validate.status = Status::EXECUTING;

          auto request =
            std::make_shared<planning_manager::srv::BehaviorVelocityPlannerValidate::Request>();
          request->trajectory = itr->second.motion_velocity_smoother_plan.result;

          client_behavior_velocity_planner_validate_->async_send_request(
            request,
            [this, itr, id](rclcpp::Client<BehaviorVelocityPlannerValidate>::SharedFuture future) {
              std::lock_guard<std::mutex> lock(map_mutex_);
              RCLCPP_INFO_EXPRESSION(
                get_logger(), is_showing_debug_info_, "%d / %ld: get
    BehaviorVelocityPlannerValidate", id, modules_result_map_.size()); const auto response =
    future.get(); itr->second.behavior_velocity_planner_validate.status = Status::FINISHED;
            });
        }
      }
    }
  */
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
  // std::lock_guard<std::mutex> lock(map_mutex_);
  // TODO(murooka) use remove_if
  // TODO(murooka) finally move this part to publishTrajectory or create removeFinishedMap

  for (const int id : planning_num_) {
    if (modules_.back().validate_result_map_->result) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), is_showing_debug_info_, "%d / %ld: remove planning", id,
        planning_num_.size());
      for (auto & module : modules_) {
        module->removeResultMap(id);
      }
    }
  }
}

}  // namespace planning_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_manager::PlanningManagerNode)
