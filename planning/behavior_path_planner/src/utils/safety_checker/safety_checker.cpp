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
  // Implement the function to filter the target object indices based on the safety_check_params_
  // You can use the member variables and functions defined in the SafetyChecker class
  return TargetObjectIndices{};
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
