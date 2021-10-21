// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SIDE_SHIFT__SIDE_SHIFT_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SIDE_SHIFT__SIDE_SHIFT_MODULE_HPP_

#include <memory>
#include <string>

#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_planning_msgs/msg/lateral_offset.hpp"

#include "rclcpp/rclcpp.hpp"

#include "behavior_path_planner/route_handler.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"

namespace behavior_path_planner
{
using autoware_planning_msgs::msg::LateralOffset;
using autoware_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;

struct SideShiftParameters
{
  double min_fix_distance;
  double start_avoid_sec;
  double drivable_area_resolution;
  double drivable_area_width;
  double drivable_area_height;
  double inside_outside_judge_margin;
};

class SideShiftModule : public SceneModuleInterface
{
public:
  SideShiftModule(
    const std::string & name, rclcpp::Node & node,
    const SideShiftParameters & parameters);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  void updateData() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  PathWithLaneId planCandidate() const override;
  void onEntry() override;
  void onExit() override;

  void setParameters(const SideShiftParameters & parameters);

private:
  rclcpp::Subscription<LateralOffset>::SharedPtr lateral_offset_subscriber_;

  void initVariables();

  void onLateralOffset(const LateralOffset::ConstSharedPtr lateral_offset_msg);

  // non-const methods
  // TODO(Horibe) cleanup args
  PathWithLaneId refinePath(
    const PathWithLaneId & input_path, double lateral_offset,
    bool & start_pose_reset_request, bool & drivable_area_shrink_request,
    double & drivable_area_extention_width, Pose & start_avoid_pose) const;

  // const methods
  void extendDrivableArea(
    PathWithLaneId & path, double lateral_offset,
    const RouteHandler & route_handler) const;
  bool isVehicleInDrivableArea(const OccupancyGrid & drivable_area) const;
  void publishPath(const PathWithLaneId & path) const;

  // member
  PathWithLaneId refined_path_{};
  std::shared_ptr<PathWithLaneId> reference_path_{std::make_shared<PathWithLaneId>()};
  lanelet::ConstLanelets current_lanelets_;
  SideShiftParameters parameters_;

  // Current lateral offset to shift the reference path.
  double lateral_offset_{0.0};

  // lateral offset used for previous planned path
  double prev_planned_lateral_offset_{0.0};

  // Triggered when offset is changed, released when start pose is refound.
  bool start_pose_reset_request_{false};

  // To keep the consistency in each planning.
  Pose start_avoid_pose_{};

  // Triggered when received smaller lateral offset, released when the drivable area is shrunk
  bool drivable_area_shrink_request_{false};

  // For the drivable area extension.
  double drivable_area_extention_width_{0.0};

  //
  bool is_approval_needed_{true};  // needs approval at first
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SIDE_SHIFT__SIDE_SHIFT_MODULE_HPP_
