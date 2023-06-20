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

void SafetyChecker::isPathSafe([[maybe_unused]] const PathWithLaneId & path)
{
  // Implement the function to check the safety of the path against dynamic obstacles
  // You can use the member variables and functions defined in the SafetyChecker class
  return;
}

PredictedPath SafetyChecker::createPredictedPath() const
{
  // Implement the function to create a predicted path based on the safety_check_params_
  // You can use the member variables and functions defined in the SafetyChecker class
  // utils::createPredictedPathFromTargetVelocity(
  //   following_trajectory_points, current_velocity, target_velocity, acc_till_target_velocity,
  //   pose, resolution, stopping_time);

  return PredictedPath{};
}

lanelet::ConstLanelets SafetyChecker::getBackwardLanelets() const
{
  // Implement the function to get the backward lanelets based on the safety_check_params_
  // You can use the member variables and functions defined in the SafetyChecker class
  return lanelet::ConstLanelets{};
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
