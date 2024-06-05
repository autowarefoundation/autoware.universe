// Copyright 2023-2024 TIER IV, Inc.
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

#include "collision.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <boost/geometry.hpp>

#include <limits>
#include <optional>
#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop
{
std::optional<geometry_msgs::msg::Point> find_closest_collision_point(
  const EgoData & ego_data, const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const tier4_autoware_utils::MultiPolygon2d & object_footprints, const PlannerParam & params)
{
  std::optional<geometry_msgs::msg::Point> closest_collision_point;
  auto closest_dist = std::numeric_limits<double>::max();
  std::vector<BoxIndexPair> rough_collisions;
  for (const auto & object_footprint : object_footprints) {
    ego_data.rtree.query(
      boost::geometry::index::intersects(object_footprint), std::back_inserter(rough_collisions));
    for (const auto & rough_collision : rough_collisions) {
      const auto path_idx = rough_collision.second;
      const auto & ego_footprint = ego_data.path_footprints[path_idx];
      const auto & ego_pose = ego_data.path.points[path_idx].point.pose;
      const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
      const auto angle_diff = tier4_autoware_utils::normalizeRadian(
        tf2::getYaw(ego_pose.orientation) - tf2::getYaw(object_pose.orientation));
      if (
        (!params.ignore_objects_behind_ego &&
         std::abs(angle_diff) < params.yaw_threshold_behind_object) ||
        std::abs(angle_diff) > params.yaw_threshold) {
        tier4_autoware_utils::MultiPoint2d collision_points;
        boost::geometry::intersection(object_footprints, ego_footprint.outer(), collision_points);
        for (const auto & coll_p : collision_points) {
          auto p = geometry_msgs::msg::Point().set__x(coll_p.x()).set__y(coll_p.y());
          const auto dist_ego_to_coll =
            motion_utils::calcSignedArcLength(ego_data.path.points, ego_pose.position, p);
          const auto dist_obj_to_coll = tier4_autoware_utils::calcDistance2d(object_pose.position, p);
          const auto tta_cp_npc = abs(dist_obj_to_coll) / object.kinematics.initial_twist_with_covariance.twist.linear.x;
          const auto tta_cp_ego = dist_ego_to_coll / ego_data.path.points[path_idx].point.longitudinal_velocity_mps;
          if (abs(dist_ego_to_coll) < closest_dist && std::abs(tta_cp_npc - tta_cp_ego) < params.time_horizon) {
            closest_dist = dist_ego_to_coll;
            closest_collision_point = p;
          }
        }
      }
    }
  }
  return closest_collision_point;
}

std::vector<Collision> find_collisions(
  const EgoData & ego_data,
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
  const std::vector<tier4_autoware_utils::MultiPolygon2d> & object_forward_footprints,
  const PlannerParam & params)
{
  std::vector<Collision> collisions;
  std::optional<geometry_msgs::msg::Point> collision;
  for (const auto & object : objects) {
    tier4_autoware_utils::MultiPolygon2d object_footprint;
    for (const auto & polygon : object_forward_footprints) {
      collision = find_closest_collision_point(ego_data, object, polygon, params);
      if (collision) {
        Collision c;
        c.object_uuid = tier4_autoware_utils::toHexString(object.object_id);
        c.point = *collision;
        collisions.push_back(c);
      }
    }
  }
  return collisions;
}

}  // namespace behavior_velocity_planner::dynamic_obstacle_stop
