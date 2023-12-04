// Copyright 2023 TIER IV, Inc. All rights reserved.
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

#include "scene_dynamic_obstacle_stop.hpp"

#include "debug.hpp"
#include "footprint.hpp"
#include "types.hpp"

#include <behavior_velocity_planner_common/utilization/debug.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_utils/distance/distance.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <boost/geometry/algorithms/intersection.hpp>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

DynamicObstacleStopModule::DynamicObstacleStopModule(
  const int64_t module_id, PlannerParam planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock), params_(std::move(planner_param))
{
  prev_stop_decision_time_ = rclcpp::Time(int64_t{0}, clock->get_clock_type());
  velocity_factor_.init(VelocityFactor::UNKNOWN);
}

std::vector<autoware_auto_perception_msgs::msg::PredictedObject> filter_predicted_objects(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const PlannerParam & params,
  const double hysteresis)
{
  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> filtered_objects;
  for (const auto & object : objects.objects) {
    const auto vel = object.kinematics.initial_twist_with_covariance.twist.linear.x;
    const auto width = object.shape.dimensions.y + params.extra_object_width;
    const auto front_position = tier4_autoware_utils::transformPoint(
      tier4_autoware_utils::createPoint(object.shape.dimensions.x / 2.0, 0.0, 0.0),
      object.kinematics.initial_pose_with_covariance.pose);
    const auto lateral_offset_limit = width / 2 + params.ego_lateral_offset + hysteresis;
    const auto is_close_to_path = [&](const auto & p) {
      return std::abs(motion_utils::calcLateralOffset(path.points, p)) < lateral_offset_limit;
    };
    const auto is_driving_toward_ego = [&]() {
      return motion_utils::calcSignedArcLength(
               path.points, object.kinematics.initial_pose_with_covariance.pose.position,
               front_position) < 0.0;
    };
    if (
      vel >= params.minimum_object_velocity && is_driving_toward_ego() &&
      is_close_to_path(object.kinematics.initial_pose_with_covariance.pose.position) &&
      is_close_to_path(front_position))
      filtered_objects.push_back(object);
  }
  return filtered_objects;
}

std::optional<geometry_msgs::msg::Point> find_earliest_collision(
  const EgoData & ego_data,
  const tier4_autoware_utils::MultiPolygon2d & obstacle_forward_footprints, DebugData & debug_data)
{
  std::optional<geometry_msgs::msg::Point> earliest_collision;
  tier4_autoware_utils::LineString2d path_ls;
  for (auto i = ego_data.first_path_idx + 1; i < ego_data.path.points.size(); ++i) {
    const auto & p = ego_data.path.points[i].point.pose.position;
    path_ls.emplace_back(p.x, p.y);
  }
  if (!path_ls.empty() && boost::geometry::within(path_ls.front(), obstacle_forward_footprints))
    return geometry_msgs::msg::Point().set__x(path_ls.front().x()).set__y(path_ls.front().y());
  // TODO(Maxime): if need better perf, check collisions for each segment of the path_ls
  tier4_autoware_utils::MultiPoint2d collision_points;
  boost::geometry::intersection(path_ls, obstacle_forward_footprints, collision_points);
  debug_data.collisions = collision_points;
  auto min_l = std::numeric_limits<double>::max();
  for (const auto & p : collision_points) {
    const auto collision = geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y());
    const auto l = motion_utils::calcSignedArcLength(
      ego_data.path.points, ego_data.path.points.front().point.pose.position, collision);
    if (l < min_l) {
      min_l = l;
      earliest_collision = collision;
    }
  }
  return earliest_collision;
}

bool DynamicObstacleStopModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  debug_data_.reset_data();
  *stop_reason = planning_utils::initializeStopReason(StopReason::OBSTACLE_STOP);
  if (!path || path->points.size() < 2) return true;
  tier4_autoware_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();

  stopwatch.tic("preprocessing");
  EgoData ego_data;
  ego_data.pose = planner_data_->current_odometry->pose;
  ego_data.path.points = path->points;
  motion_utils::removeOverlapPoints(ego_data.path.points);
  ego_data.first_path_idx =
    motion_utils::findNearestSegmentIndex(ego_data.path.points, ego_data.pose.position);
  ego_data.longitudinal_offset_to_first_path_idx = motion_utils::calcLongitudinalOffsetToSegment(
    ego_data.path.points, ego_data.first_path_idx, ego_data.pose.position);
  const auto preprocessing_duration_us = stopwatch.toc("preprocessing");
  const auto has_decided_to_stop =
    (clock_->now() - prev_stop_decision_time_).seconds() < params_.decision_duration_buffer;
  double hysteresis = has_decided_to_stop ? params_.hysteresis : 0.0;
  if (!has_decided_to_stop) current_stop_pose_.reset();

  stopwatch.tic("filter");
  const auto dynamic_obstacles =
    filter_predicted_objects(*planner_data_->predicted_objects, ego_data.path, params_, hysteresis);
  const auto filter_duration_us = stopwatch.toc("filter");
  stopwatch.tic("footprints");
  const auto obstacle_forward_footprints =
    make_forward_footprints(dynamic_obstacles, params_, hysteresis);
  const auto footprints_duration_us = stopwatch.toc("footprints");
  const auto min_stop_distance =
    motion_utils::calcDecelDistWithJerkAndAccConstraints(
      planner_data_->current_velocity->twist.linear.x, 0.0,
      planner_data_->current_acceleration->accel.accel.linear.x,
      planner_data_->max_stop_acceleration_threshold, -planner_data_->max_stop_jerk_threshold,
      planner_data_->max_stop_jerk_threshold)
      .get_value_or(0.0);
  stopwatch.tic("collisions");
  const auto collision =
    find_earliest_collision(ego_data, obstacle_forward_footprints, debug_data_);
  const auto collisions_duration_us = stopwatch.toc("collisions");
  if (collision) {
    const auto arc_length_diff =
      motion_utils::calcSignedArcLength(ego_data.path.points, *collision, ego_data.pose.position);
    const auto can_stop_before_limit = arc_length_diff < min_stop_distance -
                                                           params_.ego_longitudinal_offset -
                                                           params_.stop_distance_buffer;
    const auto stop_pose = can_stop_before_limit
                             ? motion_utils::calcLongitudinalOffsetPose(
                                 ego_data.path.points, *collision,
                                 -params_.stop_distance_buffer - params_.ego_longitudinal_offset)
                             : motion_utils::calcLongitudinalOffsetPose(
                                 ego_data.path.points, ego_data.pose.position, min_stop_distance);
    if (stop_pose) {
      const auto use_new_stop_pose = !has_decided_to_stop || motion_utils::calcSignedArcLength(
                                                               path->points, stop_pose->position,
                                                               current_stop_pose_->position) > 0.0;
      if (use_new_stop_pose) current_stop_pose_ = *stop_pose;
      prev_stop_decision_time_ = clock_->now();
    }
  }

  if (current_stop_pose_) motion_utils::insertStopPoint(*current_stop_pose_, 0.0, path->points);

  const auto total_time_us = stopwatch.toc();
  // TODO(Maxime): set to DEBUG
  RCLCPP_WARN(
    logger_,
    "Total time = %2.2fus\n\tpreprocessing = %2.2fus\n\tfilter = %2.2fus\n\tfootprints = "
    "%2.2fus\n\tcollisions = %2.2fus\n",
    total_time_us, preprocessing_duration_us, filter_duration_us, footprints_duration_us,
    collisions_duration_us);
  debug_data_.dynamic_obstacles = dynamic_obstacles;
  debug_data_.obstacle_footprints = obstacle_forward_footprints;
  return true;
}

MarkerArray DynamicObstacleStopModule::createDebugMarkerArray()
{
  constexpr auto z = 0.0;
  MarkerArray debug_marker_array;
  // dynamic obstacles
  const auto obstacle_markers = debug::make_dynamic_obstacle_markers(debug_data_.dynamic_obstacles);
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), obstacle_markers.begin(), obstacle_markers.end());
  const auto delete_obstacle_markers = debug::make_delete_markers(
    obstacle_markers.size(), debug_data_.prev_dynamic_obstacles_nb, "dynamic_obstacles");
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), delete_obstacle_markers.begin(),
    delete_obstacle_markers.end());
  // dynamic obstacles footprints
  const auto obstacle_footprint_markers =
    debug::make_polygon_markers(debug_data_.obstacle_footprints, "dynamic_obstacles_footprints", z);
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), obstacle_footprint_markers.begin(),
    obstacle_footprint_markers.end());
  const auto delete_footprint_markers = debug::make_delete_markers(
    obstacle_footprint_markers.size(), debug_data_.prev_dynamic_obstacles_nb,
    "dynamic_obstacles_footprints");
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), delete_footprint_markers.begin(),
    delete_footprint_markers.end());
  // collisions
  const auto collision_markers = debug::make_collision_markers(debug_data_.collisions, z);
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), collision_markers.begin(), collision_markers.end());

  debug_data_.prev_dynamic_obstacles_nb = obstacle_markers.size();
  return debug_marker_array;
}

motion_utils::VirtualWalls DynamicObstacleStopModule::createVirtualWalls()
{
  motion_utils::VirtualWalls virtual_walls{};
  if (current_stop_pose_) {
    motion_utils::VirtualWall virtual_wall;
    virtual_wall.text = "dynamic_obstacle_stop";
    virtual_wall.longitudinal_offset = params_.ego_longitudinal_offset;
    virtual_wall.style = motion_utils::VirtualWallType::stop;
    virtual_wall.pose = *current_stop_pose_;
    virtual_walls.push_back(virtual_wall);
  }
  return virtual_walls;
}

}  // namespace behavior_velocity_planner::dynamic_obstacle_stop
