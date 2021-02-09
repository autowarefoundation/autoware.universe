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

#include "frenet_planner_node/trajectory_generation.hpp"

#include <frenet_planner/frenet_planner.hpp>
#include <frenet_planner/trajectory_reuse.hpp>  // TODO(Maxime CLEMENT): move to node package
#include <frenet_planner_node/prepare_inputs.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <vector>

namespace frenet_planner_node
{
std::vector<frenet_planner::Trajectory> generateCandidateTrajectories(
  const frenet_planner::Point & pose, const geometry_msgs::msg::Twist & twist,
  const autoware_auto_planning_msgs::msg::Path & path,
  const frenet_planner::transform::Spline2D & path_spline,
  const frenet_planner::Trajectory & previous_trajectory,
  const frenet_planner::Constraints & constraints, frenet_planner::Debug & debug)
{
  frenet_planner::FrenetState initial_state;
  initial_state.position = path_spline.frenet(pose);
  // TODO(Maxime CLEMENT): velocities must be transformed to frenet frame
  initial_state.longitudinal_velocity = twist.linear.x;
  initial_state.lateral_velocity = twist.linear.y;
  frenet_planner::Trajectory base_trajectory;
  auto trajectories = generateFrenetTrajectories(
    initial_state, base_trajectory, path, path_spline, constraints, debug);

  const auto reused_prev_trajectory = preparePreviousTrajectory(previous_trajectory, path_spline);

  constexpr auto reuse_max_duration = 5.0;
  // constexpr auto reuse_max_length = 50.0;
  const auto reuse_max_length_max =
    previous_trajectory.frenet_points.empty() ? 0.0 : previous_trajectory.frenet_points.back().s;
  const auto reuse_length_step = reuse_max_length_max / 4.0;
  constexpr auto reuse_max_deviation = 1.0;
  for (auto reuse_max_length = reuse_length_step; reuse_max_length <= reuse_max_length_max;
       reuse_max_length += reuse_length_step) {
    if (frenet_planner::tryToReuseTrajectory(
          reused_prev_trajectory, pose, reuse_max_duration, reuse_max_length, reuse_max_deviation,
          constraints, base_trajectory)) {
      frenet_planner::FrenetState end_of_reused_traj;
      end_of_reused_traj.position = base_trajectory.frenet_points.back();
      end_of_reused_traj.longitudinal_velocity = base_trajectory.longitudinal_velocities.back();
      end_of_reused_traj.lateral_velocity = base_trajectory.lateral_velocities.back();
      const auto trajs_from_prev_traj = generateFrenetTrajectories(
        end_of_reused_traj, base_trajectory, path, path_spline, constraints, debug);
      for (const auto & traj : trajs_from_prev_traj) {
        trajectories.push_back(base_trajectory.extend(traj));
        const auto cost_mult = 1.0 - 0.3 * reuse_length_step / reuse_max_length_max;
        trajectories.back().cost *= cost_mult;
      }
    } else {
      break;
    }
  }
  return trajectories;
}

std::vector<frenet_planner::Trajectory> generateFrenetTrajectories(
  const frenet_planner::FrenetState & initial_state,
  const frenet_planner::Trajectory & base_trajectory,
  const autoware_auto_planning_msgs::msg::Path & path,
  const frenet_planner::transform::Spline2D & path_spline,
  const frenet_planner::Constraints & constraints, frenet_planner::Debug & debug)
{
  const auto sampling_parameters =
    prepareSamplingParameters(initial_state, path, base_trajectory.duration, path_spline);

  return frenet_planner::generateTrajectories(
    path_spline, initial_state, sampling_parameters, constraints, debug);
}

}  // namespace frenet_planner_node
