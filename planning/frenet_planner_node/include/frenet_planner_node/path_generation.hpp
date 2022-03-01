// Copyright 2022 Tier IV, Inc.
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

#ifndef FRENET_PLANNER_NODE__TRAJECTORY_GENERATION_HPP_
#define FRENET_PLANNER_NODE__TRAJECTORY_GENERATION_HPP_

#include "frenet_planner/structures.hpp"
#include "sampler_common/constraints/hard_constraint.hpp"
#include "sampler_common/transform/spline_transform.hpp"

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>

#include <vector>

namespace frenet_planner_node
{
/**
 * @brief generate candidate paths for the given problem inputs
 * @param [in] pose current ego pose
 * @param [in] twist current ego velocities
 * @param [in] path reference path to follow
 * @param [in] path_spline spline of the reference path
 * @param [in] previous_path previous path followed by ego
 * @param [in] constraints hard and soft constraints of the problem
 * @param [out] nb_violations number of constraint violations
 * @return generated candidate paths
 */
std::vector<sampler_common::Path> generateCandidatePaths(
  const sampler_common::Point & pose, const geometry_msgs::msg::Twist & twist,
  const autoware_auto_planning_msgs::msg::Path & path,
  const sampler_common::transform::Spline2D & path_spline,
  const sampler_common::Path & previous_path,
  const sampler_common::Constraints & constraints, sampler_common::constraints::NumberOfViolations & nb_violations);

std::vector<sampler_common::Path> generateFrenetPaths(
  const frenet_planner::FrenetState & initial_state,
  const frenet_planner::Trajectory & base_path,
  const autoware_auto_planning_msgs::msg::Path & path_msg,
  const sampler_common::transform::Spline2D & path_spline,
  const sampler_common::Constraints & constraints, frenet_planner::Debug & debug);

// generateCartesianTrajectories()
// generateSigmoidTrajectories()
}  // namespace frenet_planner_node

#endif  // FRENET_PLANNER_NODE__TRAJECTORY_GENERATION_HPP_
