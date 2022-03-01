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

#include "frenet_planner_node/path_generation.hpp"
#include "frenet_planner/structures.hpp"
#include "sampler_common/structures.hpp"

#include <frenet_planner/frenet_planner.hpp>
#include <frenet_planner_node/prepare_inputs.hpp>
#include <sampler_common/path_reuse.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <vector>

namespace frenet_planner_node
{
  /*
std::vector<sampler_common::Path> generateCandidatePaths(
  const sampler_common::Point & pose, const geometry_msgs::msg::Twist & twist,
  const autoware_auto_planning_msgs::msg::Path & path,
  const sampler_common::transform::Spline2D & path_spline,
  const sampler_common::Path & previous_path,
  const sampler_common::Constraints & constraints)
{
  frenet_planner::FrenetState initial_state;
  initial_state.position = path_spline.frenet(pose);
  initial_state.longitudinal_velocity = twist.linear.x;
  initial_state.lateral_velocity = twist.linear.y;
  sampler_common::Path base_path;
  auto paths = generateFrenetPaths(initial_state, base_path, path, path_spline, constraints);

  constexpr auto reuse_max_duration = 5.0;
  constexpr auto reuse_max_length_max = 50.0;  // TODO(Maxime CLEMENT): use length of previous_path
  const auto reuse_length_step = reuse_max_length_max / 4.0;
  constexpr auto reuse_max_deviation = 1.0;
  for (auto reuse_max_length = reuse_length_step; reuse_max_length <= reuse_max_length_max;
       reuse_max_length += reuse_length_step) {
    if (sampler_common::tryToReusePath(
          previous_path, pose, reuse_max_duration, reuse_max_length, reuse_max_deviation,
          constraints, base_path)) {
      frenet_planner::FrenetState end_of_reused_path;
      // TODO(Maxime CLEMENT): find solution for this
      // end_of_reused_traj.position = base_trajectory.frenet_points.back();
      // end_of_reused_traj.longitudinal_velocity = base_trajectory.longitudinal_velocities.back();
      // end_of_reused_traj.lateral_velocity = base_trajectory.lateral_velocities.back();
      const auto trajs_from_prev_traj = generateFrenetPaths(
        end_of_reused_path, base_path, path, path_spline, constraints);
      for (const auto & path: trajs_from_prev_traj) {
        paths.push_back(base_path.extend(path));
        const auto cost_mult = 1.0 - 0.3 * reuse_length_step / reuse_max_length_max;
        paths.back().cost *= cost_mult;
      }
    } else {
      break;
    }
  }
  return paths;
}

std::vector<sampler_common::Path> generateFrenetPaths(
  const frenet_planner::FrenetState & initial_state,
  const sampler_common::Path & base_path,
  const autoware_auto_planning_msgs::msg::Path & path,
  const sampler_common::transform::Spline2D & path_spline,
  const sampler_common::Constraints & constraints)
{
  const auto sampling_parameters =
    prepareSamplingParameters(initial_state, path, base_path, path_spline);

  return frenet_planner::generatePaths(
    path_spline, initial_state, sampling_parameters, constraints);
}

*/
}  // namespace frenet_planner_node
