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

#ifndef PLANNING_MANAGER__PLANNING_MANAGER_CORE_CPP_
#define PLANNING_MANAGER__PLANNING_MANAGER_CORE_CPP_

#include "planning_manager/module.hpp"
#include "planning_manager/srv/behavior_path_planner_plan.hpp"
#include "planning_manager/srv/behavior_path_planner_validate.hpp"
#include "planning_manager/srv/behavior_velocity_planner_plan.hpp"
#include "planning_manager/srv/behavior_velocity_planner_validate.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_planning_msgs/msg/had_map_route.hpp"
#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/approval.hpp>
#include <tier4_planning_msgs/msg/path_change_module.hpp>

#include <mutex>
#include <unordered_map>
#include <vector>

namespace planning_manager
{
using autoware_auto_planning_msgs::msg::HADMapRoute;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_planning_msgs::msg::Trajectory;
using planning_manager::msg::PlanningData;
using planning_manager::srv::BehaviorPathPlannerPlan;
using planning_manager::srv::BehaviorPathPlannerValidate;
using planning_manager::srv::BehaviorVelocityPlannerPlan;
using planning_manager::srv::BehaviorVelocityPlannerValidate;

// MEMO(murooka) behavior path
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::HADMapRoute;
using nav_msgs::msg::Odometry;
using tier4_planning_msgs::msg::Approval;
using tier4_planning_msgs::msg::PathChangeModule;

class PlanningManagerNode : public rclcpp::Node
{
public:
  explicit PlanningManagerNode(const rclcpp::NodeOptions & node_options);
  void run();

private:
  /*
  template <typename T>
  struct ResultWithStatus
  {
    T result;
  };

  struct ModulesResult
  {
    // plan
    ResultWithStatus<PathWithLaneId> behavior_path_planner_plan;
    ResultWithStatus<Path> behavior_velocity_planner_plan;

    ResultWithStatus<Trajectory> motion_velocity_smoother_plan;

    // validate
    ResultWithStatus<PathWithLaneId> behavior_path_planner_validate;
    ResultWithStatus<Path> behavior_velocity_planner_validate;

    ResultWithStatus<Trajectory> motion_velocity_smoother_validate;

    bool isValidateFinished()
    {
      if (
        behavior_path_planner_validate.status != Status::FINISHED ||
        behavior_velocity_planner_validate.status != Status::FINISHED) {
        return false;
      }
      return true;
    }
  };
  */

  // TODO(murooka) remove this
  int current_id_ = 0;

  // std::mutex mutex_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // publisher
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;

  // subscriber
  rclcpp::Subscription<HADMapRoute>::SharedPtr route_sub_;
  void onRoute(const HADMapRoute::ConstSharedPtr msg);

  // MEMO(murooka) subscriber for behavior path
  rclcpp::Subscription<HADMapBin>::SharedPtr vector_map_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr predicted_objects_sub_;
  rclcpp::Subscription<Approval>::SharedPtr external_approval_sub_;
  rclcpp::Subscription<PathChangeModule>::SharedPtr force_approval_sub_;

  void onMap(const HADMapBin::ConstSharedPtr map_msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onPredictedObjects(const PredictedObjects::ConstSharedPtr msg);
  void onExternalApproval(const Approval::ConstSharedPtr msg);
  void onForceApproval(const PathChangeModule::ConstSharedPtr msg);

  // NOTE: callback group for client must be member variable
  // std::vector<rclcpp::CallbackGroup::SharedPtr> callback_group_service_vec_;
  rclcpp::CallbackGroup::SharedPtr callback_group_timer_;

  std::vector<Module> modules_;

  /*
  // clients
  rclcpp::Client<BehaviorPathPlannerPlan>::SharedPtr client_behavior_path_planner_plan_;
  rclcpp::Client<BehaviorPathPlannerValidate>::SharedPtr client_behavior_path_planner_validate_;
  rclcpp::Client<BehaviorVelocityPlannerPlan>::SharedPtr client_behavior_velocity_planner_plan_;
  rclcpp::Client<BehaviorVelocityPlannerValidate>::SharedPtr
    client_behavior_velocity_planner_validate_;
  */

  bool is_showing_debug_info_;
  rclcpp::Time prev_plan_time_;
  double planning_hz_;

  autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr route_;
  planning_manager::msg::PlanningData planning_data_;
  std::mutex planning_data_mutex_;
  // std::unordered_map<int, ModulesResult> modules_result_map_ = {};
  // std::mutex map_mutex_;

  // parallel planning num
  std::vector<int> planning_num_;

  // autoware_auto_planning_msgs::msg::PathWithLaneId path_with_lane_id_;
  // autoware_auto_planning_msgs::msg::Path path_;
  // autoware_auto_planning_msgs::msg::Trajectory behavior_trajectory_;
  // autoware_auto_planning_msgs::msg::Trajectory motion_trajectory_;

  void planTrajectory(const HADMapRoute & route, const PlanningData & planning_data);
  void optimizeVelocity(const PlanningData & planning_data);
  void validateTrajectory(const PlanningData & planning_data);
  void publishTrajectory();
  void publishDiagnostics();
  void removeFinishedMap();
};
}  // namespace planning_manager

#endif  // PLANNING_MANAGER__PLANNING_MANAGER_CORE_CPP_
