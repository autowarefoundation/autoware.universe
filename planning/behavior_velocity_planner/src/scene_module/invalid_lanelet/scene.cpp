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
  const int64_t module_id, const int64_t lane_id, const PlannerParam & planner_param,
  const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
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
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto invalid_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const auto invalid_lanelet_polygon =
    lanelet::utils::to2D(invalid_lanelet).polygon2d().basicPolygon();
    
    const auto & ego_path = *path;
    const auto path_invalid_lanelet_polygon_intersection =
      getPathIntersectionWithInvalidLaneletPolygon(ego_path, invalid_lanelet_polygon, ego_pos, 2);


  double distance_ego_first_intersection = 0.0;

  geometry_msgs::msg::Point first_intersection_point;

  if (path_invalid_lanelet_polygon_intersection.first_intersection_point) {
    first_intersection_point =
      path_invalid_lanelet_polygon_intersection.first_intersection_point.get();
    distance_ego_first_intersection = motion_utils::calcSignedArcLength(
      path->points, current_pose->pose.position, first_intersection_point);
  }

  debug_data_.path_polygon_intersection = path_invalid_lanelet_polygon_intersection;

  for (const auto & p : invalid_lanelet.polygon2d().basicPolygon()) {
    debug_data_.invalid_lanelet_polygon.push_back(createPoint(p.x(), p.y(), ego_pos.z));
  }

  switch (state_) {
    case State::INIT: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "Init");
      }
      if ((path_invalid_lanelet_polygon_intersection.is_first_path_point_inside_polygon)) {
        state_ = State::INSIDE_INVALID_LANELET;
      } else if (path_invalid_lanelet_polygon_intersection.first_intersection_point) {
        if (distance_ego_first_intersection > planner_param_.stop_margin) {
          state_ = State::APPROACH;
        } else {
          state_ = State::INSIDE_INVALID_LANELET;
        }
      } else {
        state_ = State::INIT;
      }
      setSafe(true);
      setDistance(std::numeric_limits<double>::lowest());
      setActivation(false);
      break;
    }

    case State::APPROACH: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "Approach ");
      }

      const double longitudinal_offset =
        -1.0 *
        (planner_param_.stop_margin + planner_data_->vehicle_info_.max_longitudinal_offset_m);
      const auto op_target_point = motion_utils::calcLongitudinalOffsetPoint(
        path->points, first_intersection_point, longitudinal_offset);
      geometry_msgs::msg::Point target_point;

      if (op_target_point) {
        target_point = op_target_point.get();
      }

      const auto target_segment_idx =
        motion_utils::findNearestSegmentIndex(path->points, target_point);

      const auto op_target_point_idx =
        motion_utils::insertTargetPoint(target_segment_idx, target_point, path->points, 5e-2);
      size_t target_point_idx;
      if (op_target_point_idx) {
        target_point_idx = op_target_point_idx.get();
      }

      path->points.at(target_point_idx).point.longitudinal_velocity_mps = 0.0;
      const auto stop_pose = tier4_autoware_utils::getPose(path->points.at(target_point_idx).point);

      // const auto & op_stop_point = motion_utils::calcLongitudinalOffsetPoint(
      //   path->points, intersection_pose.position,
      //   );

      //   const auto & target_segment_idx = motion_utils::findNearestSegmentIndex(path,
      //   *op_stop_point); const auto & stop_pose = planning_utils::insertStopPoint(*op_stop_point,
      //   target_segment_idx, *path);

      // auto stop_point = first_intersection_point;
      // auto stop_point_idx = intersection_point_idx;

      // if(op_stop_point){
      //   RCLCPP_INFO(logger_, "TRUEEEE");
      //   stop_point = op_stop_point.get();
      //   stop_point_idx = motion_utils::findNearestIndex(path->points, stop_point);
      // }
      // else{
      //   RCLCPP_INFO(logger_, "FALSEEEE");
      // }

      // const size_t stop_point_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
      //   path->points, stop_point, stop_point_idx);

      // // Insert stop point
      // planning_utils::insertStopPoint(stop_point, stop_point_seg_idx, *path);

      // Get stop point and stop factor
      {
        // tier4_planning_msgs::msg::StopFactor stop_factor;
        // const auto stop_pose = *stop_pose;
        //   //tier4_autoware_utils::getPose(path->points.at(stop_point_idx));
        // stop_factor.stop_pose = stop_pose;
        // stop_factor.stop_factor_points.push_back(stop_point);
        // planning_utils::appendStopReason(stop_factor, stop_reason);
        // velocity_factor_.set(
        //   path->points, planner_data_->current_odometry->pose, stop_pose,
        //   VelocityFactor::APPROACHING);

        const auto virtual_wall_pose = motion_utils::calcLongitudinalOffsetPose(
          path->points, stop_pose.position, debug_data_.base_link2front);

        debug_data_.stop_pose = virtual_wall_pose.get();
      }
      const size_t current_seg_idx = findEgoSegmentIndex(path->points);
      const auto intersection_segment_idx =
        motion_utils::findNearestSegmentIndex(path->points, first_intersection_point);
      const double signed_arc_dist_to_intersection_point =
        motion_utils::calcSignedArcLength(
          path->points, planner_data_->current_odometry->pose.position, current_seg_idx,
          first_intersection_point, intersection_segment_idx) -
        planner_data_->vehicle_info_.max_longitudinal_offset_m;

      // Move to stopped state if stopped
      if (
        (signed_arc_dist_to_intersection_point <= planner_param_.stop_margin) &&
        (planner_data_->isVehicleStopped())) {
          
          if (planner_param_.print_debug_info) {
            RCLCPP_INFO(logger_, "APPROACH -> STOPPED");
            RCLCPP_INFO_STREAM(logger_, "signed_arc_dist_to_stop_point = " << signed_arc_dist_to_intersection_point);
          }
            
          if (signed_arc_dist_to_intersection_point < 0.0) {
            RCLCPP_ERROR(logger_,"Failed to stop before invalid lanelet but ego stopped. Change state to STOPPED");
          }
          
          state_ = State::STOPPED;
          stopped_time_ = std::make_shared<const rclcpp::Time>(clock_->now());
      }

      setSafe(true);
      setDistance(std::numeric_limits<double>::lowest());
      setActivation(false);
      break;
    }

    case State::INSIDE_INVALID_LANELET: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "INSIDE_INVALID_LANELET");
      }

      const auto current_point =
        planner_data_->current_odometry->pose.position;  // path->points.at(0).point.pose.position;
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

        const auto virtual_wall_pose = motion_utils::calcLongitudinalOffsetPose(
          path->points, stop_pose.position, debug_data_.base_link2front);

        debug_data_.stop_pose =
          virtual_wall_pose.get();  /// planning_utils::getAheadPose(0,
                                    /// -1*debug_data_.base_link2front, *path);//stop_pose;
        // debug_data_.stop_wall_pose =
        //   planning_utils::getAheadPose(0, debug_data_.base_link2front, *path);
      }

      // Move to stopped state if stopped
      if (planner_data_->isVehicleStopped()) {
        if (planner_param_.print_debug_info) {
          RCLCPP_INFO(logger_, "APPROACH -> STOPPED");
        }

        state_ = State::STOPPED;
        stopped_time_ = std::make_shared<const rclcpp::Time>(clock_->now());
      }

      setSafe(true);
      setDistance(std::numeric_limits<double>::lowest());
      setActivation(false);
      break;
    }

    case State::STOPPED: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "STOPPED");
      }

      // Change state after vehicle departure
      const auto stopped_pose = motion_utils::calcLongitudinalOffsetPose(
        path->points, planner_data_->current_odometry->pose.position, 0.0);

      if (!stopped_pose) {
        state_ = State::INIT;
        break;
      }

      SegmentIndexWithPose ego_pos_on_path;
      ego_pos_on_path.pose = stopped_pose.get();
      ego_pos_on_path.index = findEgoSegmentIndex(path->points);

      // Insert stop pose
      planning_utils::insertStopPoint(ego_pos_on_path.pose.position, ego_pos_on_path.index, *path);

      const auto virtual_wall_pose = motion_utils::calcLongitudinalOffsetPose(
        path->points, stopped_pose.get().position, debug_data_.base_link2front);

      debug_data_.stop_pose =
        virtual_wall_pose.get();  // planning_utils::getAheadPose(0, -1*debug_data_.base_link2front,
                                  // *path);;//stopped_pose.get();

      // Get stop point and stop factor
      {
        tier4_planning_msgs::msg::StopFactor stop_factor;
        stop_factor.stop_pose = ego_pos_on_path.pose;
        stop_factor.stop_factor_points.push_back(ego_pos_on_path.pose.position);
        planning_utils::appendStopReason(stop_factor, stop_reason);
        velocity_factor_.set(
          path->points, planner_data_->current_odometry->pose, ego_pos_on_path.pose,
          VelocityFactor::STOPPED);
      }

      // const auto elapsed_time = (clock_->now() - *stopped_time_).seconds();

      // TODO(ahmeddesokyebrahim): Driver RTC to take over responsibility
      setSafe(false);
      setDistance(0);
      setActivation(true);
      break;
    }

      // case default: {

      // }
  }
  return true;
}
}  // namespace behavior_velocity_planner
