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

#include "scene_walkway.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <cmath>
#include <memory>
#include <utility>

namespace autoware::behavior_velocity_planner
{
namespace bg = boost::geometry;
using autoware::motion_utils::calcLongitudinalOffsetPose;
using autoware::motion_utils::calcSignedArcLength;
using autoware::motion_utils::findNearestSegmentIndex;
using autoware::universe_utils::createPoint;
using autoware::universe_utils::getPose;

WalkwayModule::WalkwayModule(
  const int64_t module_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const PlannerParam & planner_param, const bool use_regulatory_element,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<universe_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  module_id_(module_id),
  state_(State::APPROACH),
  planner_param_(planner_param),
  use_regulatory_element_(use_regulatory_element)
{
  if (use_regulatory_element_) {
    const auto reg_elem_ptr = std::dynamic_pointer_cast<const lanelet::autoware::Crosswalk>(
      lanelet_map_ptr->regulatoryElementLayer.get(module_id));
    stop_lines_ = reg_elem_ptr->stopLines();
    walkway_ = reg_elem_ptr->crosswalkLanelet();
  } else {
    const auto stop_line = getStopLineFromMap(module_id_, lanelet_map_ptr, "crosswalk_id");
    if (!!stop_line) {
      stop_lines_.push_back(*stop_line);
    }
    walkway_ = lanelet_map_ptr->laneletLayer.get(module_id);
  }
}

std::pair<double, geometry_msgs::msg::Point> WalkwayModule::getStopLine(
  const PathWithLaneId & ego_path, bool & exist_stopline_in_map,
  const geometry_msgs::msg::Point & first_path_point_on_walkway) const
{
  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  for (const auto & stop_line : stop_lines_) {
    const auto p_stop_lines =
      getLinestringIntersects(ego_path, lanelet::utils::to2D(stop_line).basicLineString(), ego_pos);
    if (p_stop_lines.empty()) {
      continue;
    }

    exist_stopline_in_map = true;

    const auto dist_ego_to_stop =
      calcSignedArcLength(ego_path.points, ego_pos, p_stop_lines.front());
    return std::make_pair(dist_ego_to_stop, p_stop_lines.front());
  }

  exist_stopline_in_map = false;

  const auto p_stop_line = first_path_point_on_walkway;
  const auto dist_ego_to_stop = calcSignedArcLength(ego_path.points, ego_pos, p_stop_line) -
                                planner_param_.stop_distance_from_crosswalk;
  return std::make_pair(dist_ego_to_stop, p_stop_line);
}

bool WalkwayModule::modifyPathVelocity(PathWithLaneId * path)
{
  const auto & base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  debug_data_ = DebugData(planner_data_);

  const auto input = *path;

  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto path_end_points_on_walkway =
    getPathEndPointsOnCrosswalk(input, walkway_.polygon2d().basicPolygon(), ego_pos);
  if (!path_end_points_on_walkway) {
    return false;
  }

  const auto & first_path_point_on_walkway = path_end_points_on_walkway->first;

  if (state_ == State::APPROACH) {
    bool exist_stopline_in_map;
    const auto p_stop_line = getStopLine(input, exist_stopline_in_map, first_path_point_on_walkway);

    const auto & p_stop = p_stop_line.second;
    const auto stop_distance_from_crosswalk =
      exist_stopline_in_map ? 0.0 : planner_param_.stop_distance_from_crosswalk;
    const auto margin = stop_distance_from_crosswalk + base_link2front;
    const auto stop_pose = calcLongitudinalOffsetPose(input.points, p_stop, -margin);

    if (!stop_pose) {
      return false;
    }

    const auto inserted_pose = planning_utils::insertStopPoint(stop_pose->position, *path);
    if (inserted_pose) {
      debug_data_.stop_poses.push_back(inserted_pose.value());
    }

    /* get stop point and stop factor */
    planning_factor_interface_->add(
      path->points, planner_data_->current_odometry->pose, stop_pose.value(), stop_pose.value(),
      tier4_planning_msgs::msg::PlanningFactor::STOP, tier4_planning_msgs::msg::SafetyFactorArray{},
      true /*is_driving_forward*/, 0.0 /*velocity*/, 0.0 /*shift distance*/, "walkway_stop");

    // use arc length to identify if ego vehicle is in front of walkway stop or not.
    const double signed_arc_dist_to_stop_point =
      calcSignedArcLength(input.points, ego_pos, stop_pose->position);

    const double distance_threshold = 1.0;
    debug_data_.stop_judge_range = distance_threshold;

    const auto stop_at_stop_point = signed_arc_dist_to_stop_point < distance_threshold &&
                                    planner_data_->isVehicleStopped(planner_param_.stop_duration);

    if (stop_at_stop_point) {
      // If ego vehicle is after walkway stop and stopped then move to stop state
      state_ = State::STOP;
      if (signed_arc_dist_to_stop_point < -distance_threshold) {
        RCLCPP_ERROR(
          logger_, "Failed to stop near walkway but ego stopped. Change state to STOPPED");
      }
    }

    return true;
  }

  if (state_ == State::STOP) {
    if (planner_data_->isVehicleStopped()) {
      state_ = State::SURPASSED;
    }
  }

  return true;
}
}  // namespace autoware::behavior_velocity_planner
