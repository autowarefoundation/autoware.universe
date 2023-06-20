// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/utils/safety_checker/safety_checker.hpp"

#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/safety_check.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <interpolation/spline_interpolation.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
namespace safety_checker
{

void SafetyChecker::isPathSafe(const PathWithLaneId & path, const Odometry ego_odometry)
{
  path_to_safety_check_ = std::make_unique<PathWithLaneId>(path);
  ego_odometry_ = std::make_unique<Odometry>(ego_odometry);

  const auto ego_predicted_path = createPredictedPath();

  return;
}

PredictedPath SafetyChecker::createPredictedPath() const
{
  const auto following_trajectory_points = path_to_safety_check_->points;
  const double current_velocity = ego_odometry_->twist.twist.linear.x;
  const double target_velocity = safety_check_params_->target_velocity;
  const double acc_till_target_velocity = safety_check_params_->acc_till_target_velocity;
  const auto ego_pose = ego_odometry_->pose.pose;
  const double resolution = safety_check_params_->prediction_time_resolution;
  const double stopping_time = safety_check_params_->stopping_time;

  const auto ego_predicted_path = utils::createPredictedPathFromTargetVelocity(
    following_trajectory_points, current_velocity, target_velocity, acc_till_target_velocity,
    ego_pose, resolution, stopping_time);

  return ego_predicted_path;
}

lanelet::ConstLanelets SafetyChecker::getBackwardLanelets() const
{
  const auto & route_handler = safety_check_params_->route_handler;
  const auto target_lanes = safety_check_params_->target_lanes;
  const auto ego_pose = ego_odometry_->pose.pose;
  const double backward_length = safety_check_params_->backward_lane_length;

  lanelet::ConstLanelets backward_lanes{};
  // TODO(Sugahara): move this function from utils in lane change to common utils
  // copied from utils in lane change
  {
    if (target_lanes.empty()) {
      return {};
    }

    const auto arc_length = lanelet::utils::getArcCoordinates(target_lanes, ego_pose);

    if (arc_length.length >= backward_length) {
      return {};
    }

    const auto & front_lane = target_lanes.front();
    const auto preceding_lanes = route_handler->getPrecedingLaneletSequence(
      front_lane, std::abs(backward_length - arc_length.length), {front_lane});

    const auto num_of_lanes = std::invoke([&preceding_lanes]() {
      size_t sum{0};
      for (const auto & lanes : preceding_lanes) {
        sum += lanes.size();
      }
      return sum;
    });

    backward_lanes.reserve(num_of_lanes);

    for (const auto & lanes : preceding_lanes) {
      backward_lanes.insert(backward_lanes.end(), lanes.begin(), lanes.end());
    }
  }

  return backward_lanes;
}

TargetObjectIndices SafetyChecker::filterObjectIndices() const
{
  const auto path_to_safety_check = *path_to_safety_check_;
  const auto current_lanes = safety_check_params_->current_lanes;
  const auto target_lanes = safety_check_params_->target_lanes;
  const auto objects = safety_check_params_->dynamic_objects;
  const auto target_backward_lanes = getBackwardLanelets();
  const auto ego_pose = ego_odometry_->pose.pose;
  const double forward_path_length = safety_check_params_->forward_path_length;
  const auto object_types_to_check = safety_check_params_->object_types_to_check;
  const double filter_width =
    safety_check_params_->vehicle_width / 2 + safety_check_params_->lateral_buffer;

  std::vector<size_t> current_lane_obj_indices{};
  std::vector<size_t> target_lane_obj_indices{};
  std::vector<size_t> others_obj_indices{};
  // TODO(Sugahara): move this function from utils in lane change to common utils
  // copied from utils in lane change
  {
    auto isTargetObjectType =
      [](const PredictedObject & object, const ObjectTypesToCheck & object_types_to_check) {
        using autoware_auto_perception_msgs::msg::ObjectClassification;
        const auto t = utils::getHighestProbLabel(object.classification);
        const auto is_object_type =
          ((t == ObjectClassification::CAR && object_types_to_check.check_car) ||
           (t == ObjectClassification::TRUCK && object_types_to_check.check_truck) ||
           (t == ObjectClassification::BUS && object_types_to_check.check_bus) ||
           (t == ObjectClassification::TRAILER && object_types_to_check.check_trailer) ||
           (t == ObjectClassification::UNKNOWN && object_types_to_check.check_unknown) ||
           (t == ObjectClassification::BICYCLE && object_types_to_check.check_bicycle) ||
           (t == ObjectClassification::MOTORCYCLE && object_types_to_check.check_motorcycle) ||
           (t == ObjectClassification::PEDESTRIAN && object_types_to_check.check_pedestrian));
        return is_object_type;
      };
    const auto get_basic_polygon =
      [](const lanelet::ConstLanelets & lanes, const double start_dist, const double end_dist) {
        const auto polygon_3d =
          lanelet::utils::getPolygonFromArcLength(lanes, start_dist, end_dist);
        return lanelet::utils::to2D(polygon_3d).basicPolygon();
      };
    const auto arc = lanelet::utils::getArcCoordinates(current_lanes, ego_pose);
    const auto current_polygon =
      get_basic_polygon(current_lanes, arc.length, arc.length + forward_path_length);
    const auto target_polygon =
      get_basic_polygon(target_lanes, 0.0, std::numeric_limits<double>::max());
    LineString2d ego_path_linestring;
    ego_path_linestring.reserve(path_to_safety_check.points.size());
    for (const auto & pt : path_to_safety_check.points) {
      const auto & position = pt.point.pose.position;
      boost::geometry::append(ego_path_linestring, Point2d(position.x, position.y));
    }

    for (size_t i = 0; i < objects->objects.size(); ++i) {
      const auto & obj = objects->objects.at(i);

      if (!isTargetObjectType(obj, object_types_to_check)) {
        continue;
      }

      const auto obj_polygon = tier4_autoware_utils::toPolygon2d(obj);
      if (boost::geometry::intersects(current_polygon, obj_polygon)) {
        const double distance = boost::geometry::distance(obj_polygon, ego_path_linestring);

        if (distance < filter_width) {
          current_lane_obj_indices.push_back(i);
          continue;
        }
      }

      const bool is_intersect_with_target =
        boost::geometry::intersects(target_polygon, obj_polygon);
      if (is_intersect_with_target) {
        target_lane_obj_indices.push_back(i);
        continue;
      }

      const bool is_intersect_with_backward = std::invoke([&]() {
        for (const auto & ll : target_backward_lanes) {
          const bool is_intersect_with_backward =
            boost::geometry::intersects(ll.polygon2d().basicPolygon(), obj_polygon);
          if (is_intersect_with_backward) {
            target_lane_obj_indices.push_back(i);
            return true;
          }
        }
        return false;
      });

      if (!is_intersect_with_backward) {
        others_obj_indices.push_back(i);
      }
    }
  }
  return {current_lane_obj_indices, target_lane_obj_indices, others_obj_indices};
}

std::vector<std::pair<Pose, tier4_autoware_utils::Polygon2d>>
SafetyChecker::getEgoExpectedPoseAndConvertToPolygon() const
{
  const auto ego_predicted_path = createPredictedPath();
  const double check_start_time = safety_check_params_->check_start_time;
  const double check_end_time = safety_check_params_->check_end_time;
  const double time_resolution = safety_check_params_->prediction_time_resolution;
  const auto vehicle_info = safety_check_params_->vehicle_info;

  const auto reserve_size =
    static_cast<size_t>((check_end_time - check_start_time) / time_resolution);
  std::vector<double> check_durations{};
  std::vector<std::pair<Pose, tier4_autoware_utils::Polygon2d>> interpolated_ego{};
  check_durations.reserve(reserve_size);
  interpolated_ego.reserve(reserve_size);

  for (double t = check_start_time; t < check_end_time; t += time_resolution) {
    tier4_autoware_utils::Polygon2d ego_polygon;
    const auto result =
      utils::getEgoExpectedPoseAndConvertToPolygon(ego_predicted_path, t, vehicle_info);
    if (!result) {
      continue;
    }
    check_durations.push_back(t);
    interpolated_ego.emplace_back(result->first, result->second);
  }

  return interpolated_ego;
}

bool SafetyChecker::isSafeInLaneletCollisionCheck() const
{
  const auto path_to_safety_check = *path_to_safety_check_;
  const auto predicted_ego_poses = getEgoExpectedPoseAndConvertToPolygon();
  const double current_velocity = ego_odometry_->twist.twist.linear.x;
  const std::vector<double> check_duration;
  // TODO(Sugahara): implement funciton
  // const auto target_objects = getTargetObjects();
  // const auto target_objects_paths = getTargetObjectsPaths();
  // const auto rss_params = safety_check_params_->rss_params;
  std::vector<PredictedObject> target_objects;
  std::vector<PredictedPath> target_objects_paths;
  BehaviorPathPlannerParameters common_params;
  const double front_object_deceleration = safety_check_params_->expected_front_deceleration;
  const double rear_object_deceleration = safety_check_params_->expected_rear_deceleration;
  marker_utils::CollisionCheckDebug debug_info = safety_check_params_->collision_check_debug;

  for (size_t i = 0; i < target_objects.size(); i++) {
    const auto & target_object = target_objects[i];
    const auto & target_object_path = target_objects_paths[i];
    if (!utils::safety_check::isSafeInLaneletCollisionCheck(
          path_to_safety_check, predicted_ego_poses, current_velocity, check_duration,
          target_object, target_object_path, common_params, front_object_deceleration,
          rear_object_deceleration, debug_info)) {
      return false;
    }
  }

  return true;
}

bool SafetyChecker::isObjectIndexIncluded(
  const size_t & index, const std::vector<size_t> & dynamic_objects_indices) const
{
  return std::count(dynamic_objects_indices.begin(), dynamic_objects_indices.end(), index) != 0;
}

// TODO(Sugahara): move this function from utils in lane change to common utils
bool SafetyChecker::isTargetObjectFront(const Polygon2d & obj_polygon) const
{
  const auto following_trajectory_points = path_to_safety_check_->points;
  const auto ego_pose = ego_odometry_->pose.pose;
  const auto base_to_front = safety_check_params_->vehicle_info.max_longitudinal_offset_m;
  const auto ego_point =
    tier4_autoware_utils::calcOffsetPose(ego_pose, base_to_front, 0.0, 0.0).position;

  // check all edges in the polygon
  for (const auto & obj_edge : obj_polygon.outer()) {
    const auto obj_point = tier4_autoware_utils::createPoint(obj_edge.x(), obj_edge.y(), 0.0);
    if (motion_utils::isTargetPointFront(following_trajectory_points, ego_point, obj_point)) {
      return true;
    }
  }

  return false;
}

}  // namespace safety_checker
}  // namespace behavior_path_planner
