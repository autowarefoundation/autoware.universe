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

#include "scene_module/intersection/scene_merge_from_private_road.hpp"

#include <memory>
#include <vector>

#include "lanelet2_core/geometry/Polygon.h"
#include "lanelet2_core/primitives/BasicRegulatoryElements.h"
#include "lanelet2_extension/regulatory_elements/road_marking.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/utility/utilities.hpp"

#include "scene_module/intersection/util.hpp"
#include "utilization/boost_geometry_helper.hpp"
#include "utilization/interpolate.hpp"
#include "utilization/util.hpp"

namespace bg = boost::geometry;

MergeFromPrivateRoadModule::MergeFromPrivateRoadModule(
  const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
  const IntersectionModule::PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock), lane_id_(lane_id)
{
  planner_param_ = planner_param;
  const auto & assigned_lanelet = planner_data->lanelet_map->laneletLayer.get(lane_id);
  state_machine_.setState(State::STOP);
}

bool MergeFromPrivateRoadModule::modifyPathVelocity(
  autoware_planning_msgs::msg::PathWithLaneId * path,
  autoware_planning_msgs::msg::StopReason * stop_reason)
{
  debug_data_ = DebugData();
  *stop_reason = planning_utils::initializeStopReason(
    autoware_planning_msgs::msg::StopReason::MERGE_FROM_PRIVATE_ROAD);

  const auto input_path = *path;
  debug_data_.path_raw = input_path;

  State current_state = state_machine_.getState();
  RCLCPP_DEBUG(logger_, "lane_id = %ld, state = %s", lane_id_, toString(current_state).c_str());

  /* get current pose */
  geometry_msgs::msg::PoseStamped current_pose = planner_data_->current_pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  /* get detection area */
  std::vector<lanelet::CompoundPolygon3d> detection_areas;
  std::vector<lanelet::CompoundPolygon3d> conflicting_areas;

  util::getObjectivePolygons(
    lanelet_map_ptr, routing_graph_ptr, lane_id_, planner_param_, &conflicting_areas,
    &detection_areas, logger_);
  if (detection_areas.empty()) {
    RCLCPP_DEBUG(logger_, "no detection area. skip computation.");
    return true;
  }
  debug_data_.detection_area = conflicting_areas;

  /* set stop-line and stop-judgement-line for base_link */
  int stop_line_idx = -1;
  int judge_line_idx = -1;
  int first_idx_inside_lane = -1;
  if (!util::generateStopLine(
      lane_id_, conflicting_areas, planner_data_, planner_param_, path, *path, &stop_line_idx,
      &judge_line_idx, &first_idx_inside_lane, logger_.get_child("util")))
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "setStopLineIdx fail");
    return false;
  }

  if (stop_line_idx <= 0 || judge_line_idx <= 0) {
    RCLCPP_DEBUG(logger_, "stop line or judge line is at path[0], ignore planning.");
    return true;
  }

  debug_data_.virtual_wall_pose = util::getAheadPose(
    stop_line_idx, planner_data_->vehicle_info_.max_longitudinal_offset_m, *path);
  debug_data_.stop_point_pose = path->points.at(stop_line_idx).point.pose;
  if (first_idx_inside_lane != -1) {
    debug_data_.first_collision_point = path->points.at(first_idx_inside_lane).point.pose.position;
  }

  /* set stop speed */
  if (state_machine_.getState() == State::STOP) {
    constexpr double stop_vel = 0.0;
    const double decel_vel = planner_param_.decel_velocity;
    double v = (has_traffic_light_ && turn_direction_ == "straight") ? decel_vel : stop_vel;
    util::setVelocityFrom(stop_line_idx, v, path);

    /* get stop point and stop factor */
    if (v == stop_vel) {
      autoware_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = debug_data_.stop_point_pose;
      stop_factor.stop_factor_points.emplace_back(debug_data_.first_collision_point);
      planning_utils::appendStopReason(stop_factor, stop_reason);
    }

    const double distance =
      planning_utils::calcDist2d(current_pose.pose, path->points.at(stop_line_idx).point.pose);
    constexpr double distance_threshold = 2.0;
    if (distance < distance_threshold && planner_data_->isVehicleStopped()) {
      state_machine_.setState(State::GO);
    }

    return true;
  }

  return true;
}

void MergeFromPrivateRoadModule::StateMachine::setState(State state) {state_ = state;}

void MergeFromPrivateRoadModule::StateMachine::setMarginTime(const double t) {margin_time_ = t;}

MergeFromPrivateRoadModule::State MergeFromPrivateRoadModule::StateMachine::getState()
{
  return state_;
}
