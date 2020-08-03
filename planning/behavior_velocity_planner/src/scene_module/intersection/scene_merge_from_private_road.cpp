/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <scene_module/intersection/scene_merge_from_private_road.h>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_extension/regulatory_elements/road_marking.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>

#include "scene_module/intersection/util.h"
#include "utilization/boost_geometry_helper.h"
#include "utilization/interpolate.h"
#include "utilization/util.h"

namespace bg = boost::geometry;

MergeFromPrivateRoadModule::MergeFromPrivateRoadModule(
  const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
  const IntersectionModule::PlannerParam & planner_param)
: SceneModuleInterface(module_id), lane_id_(lane_id)
{
  planner_param_ = planner_param;
  const auto & assigned_lanelet = planner_data->lanelet_map->laneletLayer.get(lane_id);
  state_machine_.setState(State::STOP);
}

bool MergeFromPrivateRoadModule::modifyPathVelocity(
  autoware_planning_msgs::PathWithLaneId * path,
  autoware_planning_msgs::StopReason * stop_reason)
{
  debug_data_ = {};
  *stop_reason = planning_utils::initializeStopReason(
    autoware_planning_msgs::StopReason::MERGE_FROM_PRIVATE_ROAD);

  const auto input_path = *path;
  debug_data_.path_raw = input_path;

  State current_state = state_machine_.getState();
  ROS_DEBUG(
    "[MergeFromPrivateRoad] lane_id = %ld, state = %d", lane_id_, static_cast<int>(current_state));

  /* get current pose */
  geometry_msgs::PoseStamped current_pose = planner_data_->current_pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  /* get detection area */
  std::vector<lanelet::CompoundPolygon3d> detection_areas;
  util::getObjectivePolygons(
    lanelet_map_ptr, routing_graph_ptr, lane_id_, planner_param_, &detection_areas);
  if (detection_areas.empty()) {
    ROS_DEBUG("[MergeFromPrivateRoad] no detection area. skip computation.");
    return true;
  }
  debug_data_.detection_area = detection_areas;

  /* set stop-line and stop-judgement-line for base_link */
  int stop_line_idx = -1;
  int judge_line_idx = -1;
  int first_idx_inside_lane = -1;
  if (!util::generateStopLine(
        lane_id_, detection_areas, planner_data_, planner_param_, path, &stop_line_idx,
        &judge_line_idx, &first_idx_inside_lane)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MergeFromPrivateRoadModule::run] setStopLineIdx fail");
    return false;
  }

  if (stop_line_idx <= 0 || judge_line_idx <= 0) {
    ROS_DEBUG("[MergeFromPrivateRoad] stop line or judge line is at path[0], ignore planning.");
    return true;
  }

  debug_data_.virtual_wall_pose =
    util::getAheadPose(stop_line_idx, planner_data_->base_link2front, *path);
  debug_data_.stop_point_pose = path->points.at(stop_line_idx).point.pose;
  debug_data_.first_collision_point = path->points.at(first_idx_inside_lane).point.pose.position;

  /* set stop speed */
  if (state_machine_.getState() == State::STOP) {
    constexpr double stop_vel = 0.0;
    const double decel_vel = planner_param_.decel_velocoity;
    double v = (has_traffic_light_ && turn_direction_ == "straight") ? decel_vel : stop_vel;
    util::setVelocityFrom(stop_line_idx, v, path);

    /* get stop point and stop factor */
    if (v == stop_vel) {
      autoware_planning_msgs::StopFactor stop_factor;
      stop_factor.stop_pose = debug_data_.stop_point_pose;
      stop_factor.stop_factor_points.emplace_back(debug_data_.first_collision_point);
      planning_utils::appendStopReason(stop_factor, stop_reason);
    }

    const double distance =
      planning_utils::calcDist2d(current_pose.pose, path->points.at(stop_line_idx).point.pose);
    constexpr double distance_threshold = 2.0;
    if (distance < distance_threshold && planner_data_->isVehicleStopping())
      state_machine_.setState(State::GO);

    return true;
  }

  return true;
}

void MergeFromPrivateRoadModule::StateMachine::setStateWithMarginTime(State state)
{
  /* same state request */
  if (state_ == state) {
    start_time_ = nullptr;  // reset timer
    return;
  }

  /* GO -> STOP */
  if (state == State::STOP) {
    state_ = State::STOP;
    start_time_ = nullptr;  // reset timer
    return;
  }

  /* STOP -> GO */
  if (state == State::GO) {
    if (start_time_ == nullptr) {
      start_time_ = std::make_shared<ros::Time>(ros::Time::now());
    } else {
      const double duration = (ros::Time::now() - *start_time_).toSec();
      if (duration > margin_time_) {
        state_ = State::GO;
        start_time_ = nullptr;  // reset timer
      }
    }
    return;
  }

  ROS_ERROR("[StateMachine] : Unsuitable state. ignore request.");
  return;
}

void MergeFromPrivateRoadModule::StateMachine::setState(State state) { state_ = state; }

void MergeFromPrivateRoadModule::StateMachine::setMarginTime(const double t) { margin_time_ = t; }

MergeFromPrivateRoadModule::State MergeFromPrivateRoadModule::StateMachine::getState()
{
  return state_;
}
