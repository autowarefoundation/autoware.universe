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

#include "behavior_path_racing_overtake_module/util.hpp"

#include "behavior_path_planner_common/utils/path_utils.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"
#include "motion_utils/trajectory/interpolation.hpp"

#include <tf2/utils.h>

#include <cmath>

namespace behavior_path_planner::racing_overtake::util
{

using motion_utils::calcInterpolatedPose;

void addLateralOffset(PathWithLaneId * path, double lateral_offset)
{
  for (auto & point : path->points) {
    double yaw = tf2::getYaw(point.point.pose.orientation);
    point.point.pose.position.x -= lateral_offset * std::sin(yaw);
    point.point.pose.position.y += lateral_offset * std::cos(yaw);
  }
}

std::optional<RivalVehicle> detectRivalVehicleInEgoCourse(
  const Pose & ego_pose, const PathWithLaneId & centerline_path,
  const std::vector<PredictedObject> & objects, double ego_course_width)
{
  auto self_odom_frenet = utils::convertToFrenetPoint(centerline_path.points, ego_pose.position, 0);

  std::optional<PredictedObject> closest_front_object_ = std::nullopt;
  double closest_front_object_length = std::numeric_limits<double>::max();
  double closest_front_object_distance;

  for (const auto & object : objects) {
    auto obj_frenet = utils::convertToFrenetPoint(
      centerline_path.points, object.kinematics.initial_pose_with_covariance.pose.position, 0);

    if (
      obj_frenet.length - self_odom_frenet.length > 0.0 &&
      obj_frenet.length - self_odom_frenet.length < closest_front_object_length &&
      std::abs(obj_frenet.distance - self_odom_frenet.distance) < ego_course_width) {
      closest_front_object_ = object;
      closest_front_object_length = obj_frenet.length - self_odom_frenet.length;
      closest_front_object_distance = obj_frenet.distance - self_odom_frenet.distance;
    }
  }

  if (!closest_front_object_.has_value()) {
    return std::nullopt;
  }

  RivalVehicle rival_vehicle;
  rival_vehicle.object = closest_front_object_.value();
  rival_vehicle.longitudinal_from_ego = closest_front_object_length;
  rival_vehicle.lateral_from_ego = closest_front_object_distance;
  return rival_vehicle;
}

std::tuple<PathWithLaneId, Pose, double> calcOvertakePath(
  const PathWithLaneId & reference_path, const PredictedObject & object,
  double current_course_shift_length)
{
  auto current_coure_path = reference_path;

  addLateralOffset(&current_coure_path, current_course_shift_length);

  utils::FrenetPoint object_frenet_point = utils::convertToFrenetPoint(
    current_coure_path.points, object.kinematics.initial_pose_with_covariance.pose.position, 0);
  Pose object_front_pose =
    calcInterpolatedPose(current_coure_path.points, object_frenet_point.length);
  Pose object_backward_pose =
    calcInterpolatedPose(current_coure_path.points, object_frenet_point.length - 20.0);

  double shift_length_candidate1 = object_frenet_point.distance + 4.0;
  double shift_length_candidate2 = object_frenet_point.distance - 4.0;
  double shift_length = std::abs(shift_length_candidate1 + current_course_shift_length) <
                            std::abs(shift_length_candidate2 + current_course_shift_length)
                          ? shift_length_candidate1
                          : shift_length_candidate2;
  ShiftLine shift_line;
  shift_line.start = object_backward_pose;
  shift_line.end = object_front_pose;
  shift_line.end_shift_length = shift_length;

  PathShifter path_shifter;
  path_shifter.setPath(current_coure_path);
  path_shifter.setShiftLines({shift_line});
  ShiftedPath shifted_path;
  path_shifter.generate(&shifted_path);
  return {shifted_path.path, object_front_pose, shift_length + current_course_shift_length};
}

std::pair<PathWithLaneId, Pose> calcBackToCenterPath(
  const PathWithLaneId & reference_path, const Pose & ego_pose, double shift_length,
  double shift_start_length, double shift_end_length)
{
  auto ego_frenet_point = utils::convertToFrenetPoint(reference_path.points, ego_pose.position, 0);

  ShiftLine shift_line;
  shift_line.start =
    calcInterpolatedPose(reference_path.points, shift_start_length + ego_frenet_point.length);
  shift_line.end =
    calcInterpolatedPose(reference_path.points, shift_end_length + ego_frenet_point.length);
  shift_line.end_shift_length = -shift_length;

  PathShifter path_shifter;
  path_shifter.setPath(reference_path);
  path_shifter.setShiftLines({shift_line});
  ShiftedPath shifted_path;
  path_shifter.generate(&shifted_path);

  addLateralOffset(&shifted_path.path, shift_length);
  return {shifted_path.path, shift_line.end};
}

}  // namespace behavior_path_planner::racing_overtake::util
