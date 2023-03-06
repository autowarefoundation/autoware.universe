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

#include "scene_module/invalid_lanelet/scene.hpp"

#include "utilization/arc_lane_util.hpp"
#include "utilization/path_utilization.hpp"
#include "utilization/util.hpp"

#include <interpolation/spline_interpolation.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <lanelet2_core/utility/Optional.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using tier4_autoware_utils::createPoint;
namespace bg = boost::geometry;

InvalidLaneletModule::InvalidLaneletModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::InvalidLanelet & invalid_lanelet_reg_elem,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
  invalid_lanelet_reg_elem_(invalid_lanelet_reg_elem),
  planner_param_(planner_param),
  state_(State::INIT)
{
  velocity_factor_.init(VelocityFactor::INVALID_LANELET);
}

bool InvalidLaneletModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  if (path->points.empty()) {
    return false;
  }

  const auto & current_pose = planner_data_->current_odometry;
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  *stop_reason = planning_utils::initializeStopReason(StopReason::INVALID_LANELET);

  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto & invalid_lanelet = invalid_lanelet_reg_elem_.invalidLanelet();
  const auto & invalid_lanelet_polygon = lanelet::utils::to2D(invalid_lanelet).basicPolygon();

  const auto & ego_path = *path;
  const auto & path_polygon_intersection_status =
    getPathIntersectionWithInvalidLaneletPolygon(ego_path, invalid_lanelet_polygon, ego_pos, 2);

  double distance = 0.0;
  const double distance_threshold = 1e-3;
  geometry_msgs::msg::Point first_intersection_point;
  if (path_polygon_intersection_status.first_intersection_point) {
    first_intersection_point = path_polygon_intersection_status.first_intersection_point.get();
    distance = motion_utils::calcSignedArcLength(
      path->points, current_pose->pose.position, first_intersection_point);
  }

  debug_data_.path_polygon_intersection_status = path_polygon_intersection_status;

  for (const auto & p : invalid_lanelet_reg_elem_.invalidLanelet().basicPolygon()) {
    debug_data_.invalid_lanelet_polygon.push_back(createPoint(p.x(), p.y(), ego_pos.z));
  }

  switch (state_) {
    case State::INIT: {
      if (path_polygon_intersection_status.is_path_inside_of_polygon) {
        state_ = State::INSIDE_INVALID_LANELET;
      } else if (distance > distance_threshold) {
        state_ = State::APPROACH;
      } else if (distance <= distance_threshold) {
        state_ = State::INSIDE_INVALID_LANELET;
      } else {
        state_ = State::INIT;
      }
      break;
    }

    case State::APPROACH: {
      const auto intersection_point_idx =
        motion_utils::findNearestIndex(path->points, first_intersection_point);
      const size_t intersection_point_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
        path->points, first_intersection_point, intersection_point_idx);
      // Insert stop point
      planning_utils::insertStopPoint(first_intersection_point, intersection_point_seg_idx, *path);

      // Get stop point and stop factor
      {
        tier4_planning_msgs::msg::StopFactor stop_factor;
        const auto stop_pose =
          tier4_autoware_utils::getPose(path->points.at(intersection_point_idx));
        stop_factor.stop_pose = stop_pose;
        stop_factor.stop_factor_points.push_back(first_intersection_point);
        planning_utils::appendStopReason(stop_factor, stop_reason);
        velocity_factor_.set(
          path->points, planner_data_->current_odometry->pose, stop_pose,
          VelocityFactor::APPROACHING);

        debug_data_.stop_pose = stop_pose;
        debug_data_.stop_wall_pose =
          planning_utils::getAheadPose(intersection_point_idx, debug_data_.base_link2front, *path);
      }

      // Move to stopped state if stopped
      const size_t current_seg_idx = findEgoSegmentIndex(path->points);
      const double signed_arc_dist_to_stop_point = motion_utils::calcSignedArcLength(
        path->points, planner_data_->current_odometry->pose.position, current_seg_idx,
        first_intersection_point, intersection_point_seg_idx);
      if (
        signed_arc_dist_to_stop_point < planner_param_.stop_margin &&
        planner_data_->isVehicleStopped()) {
        RCLCPP_INFO(logger_, "APPROACH -> STOPPED");

        state_ = State::STOPPED;
        stopped_time_ = std::make_shared<const rclcpp::Time>(clock_->now());

        if (signed_arc_dist_to_stop_point < -planner_param_.stop_margin) {
          RCLCPP_ERROR(
            logger_,
            "Failed to stop near invalid lanelet but ego stopped. Change state to STOPPED");
        }
      }

      break;
    }

    case State::INSIDE_INVALID_LANELET: {
      const auto current_point = path->points.at(0).point.pose.position;
      const size_t current_seg_idx = findEgoSegmentIndex(path->points);
      // Insert stop point
      planning_utils::insertStopPoint(current_point, current_seg_idx, *path);

      // Get stop point and stop factor
      {
        tier4_planning_msgs::msg::StopFactor stop_factor;
        const auto stop_pose = tier4_autoware_utils::getPose(path->points.at(0));
        stop_factor.stop_pose = stop_pose;
        stop_factor.stop_factor_points.push_back(current_point);
        planning_utils::appendStopReason(stop_factor, stop_reason);
        velocity_factor_.set(
          path->points, planner_data_->current_odometry->pose, stop_pose,
          VelocityFactor::INVALID_LANELET);

        debug_data_.stop_pose = stop_pose;
        debug_data_.stop_wall_pose =
          planning_utils::getAheadPose(0, debug_data_.base_link2front, *path);
      }

      // Move to stopped state if stopped
      if (planner_data_->isVehicleStopped()) {
        RCLCPP_INFO(logger_, "APPROACH -> STOPPED");

        state_ = State::STOPPED;
        stopped_time_ = std::make_shared<const rclcpp::Time>(clock_->now());
      }

      break;
    }

    case State::STOPPED: {
      // TODO(ahmeddesokyebrahim): Driver RTC to take over responsibility
      break;
    }

      // case default: {

      // }
  }
  return true;
}
}  // namespace behavior_velocity_planner
