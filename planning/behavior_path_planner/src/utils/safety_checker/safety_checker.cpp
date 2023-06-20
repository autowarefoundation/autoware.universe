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

boost::optional<std::pair<Pose, Polygon2d>> SafetyChecker::getEgoExpectedPoseAndConvertToPolygon()
  const
{
  // Implement the function to get the ego's expected pose and convert it to a polygon based on the
  // safety_check_params_ You can use the member variables and functions defined in the
  // SafetyChecker class
  return boost::none;
}

bool SafetyChecker::isSafeInLaneletCollisionCheck() const
{
  // Implement the function to check the safety in lanelet collision based on the
  // safety_check_params_ You can use the member variables and functions defined in the
  // SafetyChecker class
  return false;
}

bool SafetyChecker::isObjectIndexIncluded() const
{
  // Implement the function to check if the object index is included based on the
  // safety_check_params_ You can use the member variables and functions defined in the
  // SafetyChecker class
  return false;
}

bool SafetyChecker::isTargetObjectFront() const
{
  // Implement the function to check if the target object is in front based on the
  // safety_check_params_ You can use the member variables and functions defined in the
  // SafetyChecker class
  return false;
}

}  // namespace safety_checker
}  // namespace behavior_path_planner
