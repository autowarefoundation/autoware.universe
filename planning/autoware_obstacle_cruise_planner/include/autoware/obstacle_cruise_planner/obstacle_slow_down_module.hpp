// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_SLOW_DOWN_MODULE_HPP_
#define AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_SLOW_DOWN_MODULE_HPP_

#include "autoware/obstacle_cruise_planner/common_structs.hpp"
#include "autoware/obstacle_cruise_planner/obstacle_velocity_module_interface.hpp"
#include "autoware/obstacle_cruise_planner/type_alias.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
bool isLowerConsideringHysteresis(
  const double current_val, const bool was_low, const double high_val, const double low_val)
{
  if (was_low) {
    if (high_val < current_val) {
      return false;
    }
    return true;
  }
  if (current_val < low_val) {
    return true;
  }
  return false;
}

geometry_msgs::msg::Point toGeomPoint(const autoware::universe_utils::Point2d & point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point.x();
  geom_point.y = point.y();
  return geom_point;
}
}  // namespace

namespace autoware::motion_planning
{
class ObstacleSlowDownModule : public ObstacleVelocityModuleInterface
{
public:
  ObstacleSlowDownModule(rclcpp::Node * node, const VehicleInfo & vehicle_info)
  : ObstacleVelocityModuleInterface(node, vehicle_info)
  {
    enable_slow_down_planning_ = node->declare_parameter<bool>("common.enable_slow_down_planning");
    slow_down_obstacle_types_ =
      obstacle_cruise_utils::getTargetObjectType(*node, "common.slow_down_obstacle_type.");
    use_pointcloud_for_slow_down_ =
      node->declare_parameter<bool>("common.slow_down_obstacle_type.pointcloud");
  }

  std::vector<SlowDownObstacle> determineEgoBehaviorAgainstPredictedObjectObstacles(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<Obstacle> & obstacles,
    const BehaviorDeterminationParam & behavior_determination_param)
  {
    behavior_determination_param_ = behavior_determination_param;

    // slow down
    slow_down_condition_counter_.resetCurrentUuids();
    std::vector<SlowDownObstacle> slow_down_obstacles;
    for (const auto & obstacle : obstacles) {
      const auto slow_down_obstacle = createSlowDownObstacleForPredictedObject(
        odometry, decimated_traj_points, obstacle, obstacle.precise_lat_dist);
      if (slow_down_obstacle) {
        slow_down_obstacles.push_back(*slow_down_obstacle);
        continue;
      }
    }
    slow_down_condition_counter_.removeCounterUnlessUpdated();
    prev_slow_down_object_obstacles_ = slow_down_obstacles;

    return slow_down_obstacles;
  }

private:
  bool use_pointcloud_for_slow_down_;
  bool enable_slow_down_planning_{false};
  std::vector<SlowDownObstacle> prev_slow_down_object_obstacles_;
  std::vector<int> slow_down_obstacle_types_;
  struct SlowDownConditionCounter
  {
    void resetCurrentUuids() { current_uuids_.clear(); }
    void addCurrentUuid(const std::string & uuid) { current_uuids_.push_back(uuid); }
    void removeCounterUnlessUpdated()
    {
      std::vector<std::string> obsolete_uuids;
      for (const auto & key_and_value : counter_) {
        if (
          std::find(current_uuids_.begin(), current_uuids_.end(), key_and_value.first) ==
          current_uuids_.end()) {
          obsolete_uuids.push_back(key_and_value.first);
        }
      }

      for (const auto & obsolete_uuid : obsolete_uuids) {
        counter_.erase(obsolete_uuid);
      }
    }

    int increaseCounter(const std::string & uuid)
    {
      if (counter_.count(uuid) != 0) {
        counter_.at(uuid) = std::max(1, counter_.at(uuid) + 1);
      } else {
        counter_.emplace(uuid, 1);
      }
      return counter_.at(uuid);
    }
    int decreaseCounter(const std::string & uuid)
    {
      if (counter_.count(uuid) != 0) {
        counter_.at(uuid) = std::min(-1, counter_.at(uuid) - 1);
      } else {
        counter_.emplace(uuid, -1);
      }
      return counter_.at(uuid);
    }
    void reset(const std::string & uuid) { counter_.emplace(uuid, 0); }

    // NOTE: positive is for meeting entering condition, and negative is for exiting.
    std::unordered_map<std::string, int> counter_;
    std::vector<std::string> current_uuids_;
  };
  SlowDownConditionCounter slow_down_condition_counter_;

  bool isSlowDownObstacle(const uint8_t label) const
  {
    const auto & types = slow_down_obstacle_types_;
    return std::find(types.begin(), types.end(), label) != types.end();
  }

  std::optional<SlowDownObstacle> createSlowDownObstacleForPredictedObject(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const Obstacle & obstacle, const double precise_lat_dist)
  {
    const auto & object_id = obstacle.uuid.substr(0, 4);
    const auto & p = behavior_determination_param_;
    slow_down_condition_counter_.addCurrentUuid(obstacle.uuid);

    const bool is_prev_obstacle_slow_down =
      obstacle_cruise_utils::getObstacleFromUuid(prev_slow_down_object_obstacles_, obstacle.uuid)
        .has_value();

    if (!enable_slow_down_planning_ || !isSlowDownObstacle(obstacle.classification.label)) {
      return std::nullopt;
    }

    // check lateral distance considering hysteresis
    const bool is_lat_dist_low = isLowerConsideringHysteresis(
      precise_lat_dist, is_prev_obstacle_slow_down,
      p.max_lat_margin_for_slow_down + p.lat_hysteresis_margin_for_slow_down / 2.0,
      p.max_lat_margin_for_slow_down - p.lat_hysteresis_margin_for_slow_down / 2.0);

    const bool is_slow_down_required = [&]() {
      if (is_prev_obstacle_slow_down) {
        // check if exiting slow down
        if (!is_lat_dist_low) {
          const int count = slow_down_condition_counter_.decreaseCounter(obstacle.uuid);
          if (count <= -p.successive_num_to_exit_slow_down_condition) {
            slow_down_condition_counter_.reset(obstacle.uuid);
            return false;
          }
        }
        return true;
      }
      // check if entering slow down
      if (is_lat_dist_low) {
        const int count = slow_down_condition_counter_.increaseCounter(obstacle.uuid);
        if (p.successive_num_to_entry_slow_down_condition <= count) {
          slow_down_condition_counter_.reset(obstacle.uuid);
          return true;
        }
      }
      return false;
    }();
    if (!is_slow_down_required) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[SlowDown] Ignore obstacle (%s) since it's far from trajectory. (%f [m])",
      //   object_id.c_str(), precise_lat_dist);
      return std::nullopt;
    }

    const auto obstacle_poly = autoware::universe_utils::toPolygon2d(obstacle.pose, obstacle.shape);

    // calculate collision points with trajectory with lateral stop margin
    // NOTE: For additional margin, hysteresis is not divided by two.
    const auto traj_polys_with_lat_margin = obstacle_cruise_utils::createOneStepPolygons(
      traj_points, vehicle_info_, odometry.pose.pose,
      p.max_lat_margin_for_slow_down + p.lat_hysteresis_margin_for_slow_down,
      behavior_determination_param_);

    std::vector<Polygon2d> front_collision_polygons;
    size_t front_seg_idx = 0;
    std::vector<Polygon2d> back_collision_polygons;
    size_t back_seg_idx = 0;
    for (size_t i = 0; i < traj_polys_with_lat_margin.size(); ++i) {
      std::vector<Polygon2d> collision_polygons;
      bg::intersection(traj_polys_with_lat_margin.at(i), obstacle_poly, collision_polygons);

      if (!collision_polygons.empty()) {
        if (front_collision_polygons.empty()) {
          front_collision_polygons = collision_polygons;
          front_seg_idx = i == 0 ? i : i - 1;
        }
        back_collision_polygons = collision_polygons;
        back_seg_idx = i == 0 ? i : i - 1;
      } else {
        if (!back_collision_polygons.empty()) {
          break;  // for efficient calculation
        }
      }
    }

    if (front_collision_polygons.empty() || back_collision_polygons.empty()) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[SlowDown] Ignore obstacle (%s) since there is no collision point", object_id.c_str());
      return std::nullopt;
    }

    // calculate front collision point
    double front_min_dist = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point front_collision_point;
    for (const auto & collision_poly : front_collision_polygons) {
      for (const auto & collision_point : collision_poly.outer()) {
        const auto collision_geom_point = toGeomPoint(collision_point);
        const double dist = autoware::motion_utils::calcLongitudinalOffsetToSegment(
          traj_points, front_seg_idx, collision_geom_point);
        if (dist < front_min_dist) {
          front_min_dist = dist;
          front_collision_point = collision_geom_point;
        }
      }
    }

    // calculate back collision point
    double back_max_dist = -std::numeric_limits<double>::max();
    geometry_msgs::msg::Point back_collision_point = front_collision_point;
    for (const auto & collision_poly : back_collision_polygons) {
      for (const auto & collision_point : collision_poly.outer()) {
        const auto collision_geom_point = toGeomPoint(collision_point);
        const double dist = autoware::motion_utils::calcLongitudinalOffsetToSegment(
          traj_points, back_seg_idx, collision_geom_point);
        if (back_max_dist < dist) {
          back_max_dist = dist;
          back_collision_point = collision_geom_point;
        }
      }
    }

    return SlowDownObstacle{
      obstacle.uuid,
      obstacle.stamp,
      obstacle.classification,
      obstacle.pose,
      obstacle.longitudinal_velocity,
      obstacle.approach_velocity,
      precise_lat_dist,
      front_collision_point,
      back_collision_point};
  }

  std::optional<SlowDownObstacle> createSlowDownObstacleForPointCloud(
    const Obstacle & obstacle, const double precise_lat_dist)
  {
    if (!enable_slow_down_planning_ || !use_pointcloud_for_slow_down_) {
      return std::nullopt;
    }

    if (!obstacle.slow_down_front_collision_point) {
      return std::nullopt;
    }

    auto front_collision_point = *obstacle.slow_down_front_collision_point;
    auto back_collision_point =
      obstacle.slow_down_back_collision_point.value_or(front_collision_point);

    return SlowDownObstacle{
      obstacle.uuid,    obstacle.stamp,        obstacle.classification, obstacle.pose, {}, {},
      precise_lat_dist, front_collision_point, back_collision_point};
  }
};
}  // namespace autoware::motion_planning

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_SLOW_DOWN_MODULE_HPP_
