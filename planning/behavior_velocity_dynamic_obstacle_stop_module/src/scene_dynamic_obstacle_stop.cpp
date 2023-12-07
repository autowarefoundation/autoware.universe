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
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <boost/geometry.hpp>
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
  velocity_factor_.init(PlanningBehavior::UNKNOWN);
}

std::vector<autoware_auto_perception_msgs::msg::PredictedObject> filter_predicted_objects(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects, const EgoData & ego_data,
  const PlannerParam & params, const double hysteresis)
{
  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> filtered_objects;
  const auto is_vehicle = [](const auto & o) {
    return std::find_if(o.classification.begin(), o.classification.end(), [](const auto & c) {
             return c.label == autoware_auto_perception_msgs::msg::ObjectClassification::CAR ||
                    c.label == autoware_auto_perception_msgs::msg::ObjectClassification::BUS ||
                    c.label == autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK ||
                    c.label == autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER ||
                    c.label ==
                      autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE ||
                    c.label == autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE;
           }) != o.classification.end();
  };
  const auto is_in_range = [&](const auto & o) {
    for (const auto & p : ego_data.path.points) {
      const auto distance = tier4_autoware_utils::calcDistance2d(
        o.kinematics.initial_pose_with_covariance.pose, p.point.pose);
      if (
        distance <= params.minimum_object_distance_from_ego_path + params.ego_lateral_offset +
                      o.shape.dimensions.y / 2.0 + hysteresis)
        return true;
    }
    return false;
  };
  const auto is_not_too_close = [&](const auto & o) {
    const auto obj_arc_length = motion_utils::calcSignedArcLength(
      ego_data.path.points, ego_data.pose.position,
      o.kinematics.initial_pose_with_covariance.pose.position);
    std::cout << obj_arc_length << " > "
              << ego_data.longitudinal_offset_to_first_path_idx + params.ego_longitudinal_offset +
                   o.shape.dimensions.x / 2.0
              << std::endl;
    return obj_arc_length > ego_data.longitudinal_offset_to_first_path_idx +
                              params.ego_longitudinal_offset + o.shape.dimensions.x / 2.0;
  };
  for (const auto & object : objects.objects)
    if (
      is_vehicle(object) &&
      object.kinematics.initial_twist_with_covariance.twist.linear.x >=
        params.minimum_object_velocity &&
      is_in_range(object) && is_not_too_close(object))
      filtered_objects.push_back(object);
  return filtered_objects;
}

std::optional<geometry_msgs::msg::Point> find_earliest_collision(
  const EgoData & ego_data,
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
  const tier4_autoware_utils::MultiPolygon2d & obstacle_forward_footprints, DebugData & debug_data)
{
  auto earliest_idx = ego_data.path_footprints.size();
  auto earliest_arc_length = motion_utils::calcArcLength(ego_data.path.points);
  std::optional<geometry_msgs::msg::Point> earliest_collision_point;
  debug_data.collisions.clear();
  std::vector<BoxIndexPair> rough_collisions;
  for (auto obstacle_idx = 0UL; obstacle_idx < objects.size(); ++obstacle_idx) {
    rough_collisions.clear();
    const auto & obstacle_pose = objects[obstacle_idx].kinematics.initial_pose_with_covariance.pose;
    const auto & obstacle_footprint = obstacle_forward_footprints[obstacle_idx];
    ego_data.rtree.query(
      boost::geometry::index::intersects(obstacle_footprint), std::back_inserter(rough_collisions));
    for (const auto & rough_collision : rough_collisions) {
      const auto path_idx = rough_collision.second;
      const auto & ego_footprint = ego_data.path_footprints[path_idx];
      const auto & ego_pose = ego_data.path.points[ego_data.first_path_idx + path_idx].point.pose;
      const auto angle_diff = tier4_autoware_utils::normalizeRadian(
        tf2::getYaw(ego_pose.orientation) - tf2::getYaw(obstacle_pose.orientation));
      if (path_idx <= earliest_idx && std::abs(angle_diff) > (M_PI_2 + M_PI_4)) {
        tier4_autoware_utils::MultiPoint2d collision_points;
        boost::geometry::intersection(
          obstacle_footprint.outer(), ego_footprint.outer(), collision_points);
        earliest_idx = path_idx;
        for (const auto & coll_p : collision_points) {
          auto p = geometry_msgs::msg::Point().set__x(coll_p.x()).set__y(coll_p.y());
          const auto arc_length =
            motion_utils::calcSignedArcLength(ego_data.path.points, ego_data.first_path_idx, p);
          if (arc_length < earliest_arc_length) {
            earliest_arc_length = arc_length;
            debug_data.collisions = {obstacle_footprint, ego_footprint};
            earliest_collision_point = p;
          }
        }
      }
    }
  }
  return earliest_collision_point;
}

void make_ego_footprint_rtree(EgoData & ego_data, const PlannerParam & params)
{
  for (const auto & p : ego_data.path.points)
    ego_data.path_footprints.push_back(tier4_autoware_utils::toFootprint(
      p.point.pose, params.ego_longitudinal_offset, 0.0, params.ego_lateral_offset * 2.0));
  for (auto i = 0UL; i < ego_data.path_footprints.size(); ++i) {
    const auto box =
      boost::geometry::return_envelope<tier4_autoware_utils::Box2d>(ego_data.path_footprints[i]);
    ego_data.rtree.insert(std::make_pair(box, i));
  }
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
  make_ego_footprint_rtree(ego_data, params_);
  const auto has_decided_to_stop =
    (clock_->now() - prev_stop_decision_time_).seconds() < params_.decision_duration_buffer;
  if (!has_decided_to_stop) current_stop_pose_.reset();
  double hysteresis = has_decided_to_stop ? params_.hysteresis : 0.0;
  const auto dynamic_obstacles =
    filter_predicted_objects(*planner_data_->predicted_objects, ego_data, params_, hysteresis);

  const auto preprocessing_duration_us = stopwatch.toc("preprocessing");

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
      .value_or(0.0);
  stopwatch.tic("collisions");
  const auto collision =
    find_earliest_collision(ego_data, dynamic_obstacles, obstacle_forward_footprints, debug_data_);
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
    "Total time = %2.2fus\n\tpreprocessing = %2.2fus\n\tfootprints = "
    "%2.2fus\n\tcollisions = %2.2fus\n",
    total_time_us, preprocessing_duration_us, footprints_duration_us, collisions_duration_us);
  debug_data_.ego_footprints = ego_data.path_footprints;
  debug_data_.obstacle_footprints = obstacle_forward_footprints;
  return true;
}

MarkerArray DynamicObstacleStopModule::createDebugMarkerArray()
{
  constexpr auto z = 0.0;
  MarkerArray debug_marker_array;
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
  // ego path footprints
  const auto ego_footprint_markers =
    debug::make_polygon_markers(debug_data_.ego_footprints, "ego_footprints", z);
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), ego_footprint_markers.begin(), ego_footprint_markers.end());
  const auto delete_ego_footprint_markers = debug::make_delete_markers(
    ego_footprint_markers.size(), debug_data_.prev_ego_footprints_nb, "ego_footprints");
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), delete_ego_footprint_markers.begin(),
    delete_ego_footprint_markers.end());
  // collisions
  auto collision_markers = debug::make_polygon_markers(debug_data_.collisions, "collisions", z);
  for (auto & m : collision_markers) m.color.r = 1.0;
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), collision_markers.begin(), collision_markers.end());
  const auto delete_collision_markers = debug::make_delete_markers(
    collision_markers.size(), debug_data_.prev_collisions_nb, "collisions");
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), delete_collision_markers.begin(),
    delete_collision_markers.end());

  debug_data_.prev_dynamic_obstacles_nb = obstacle_footprint_markers.size();
  debug_data_.prev_collisions_nb = collision_markers.size();
  debug_data_.prev_ego_footprints_nb = ego_footprint_markers.size();
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
