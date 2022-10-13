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

#include "sampler_node/trajectory_generation.hpp"

#include "frenet_planner/frenet_planner.hpp"
#include "frenet_planner/structures.hpp"
#include "sampler_common/trajectory_reuse.hpp"
#include "sampler_node/prepare_inputs.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <numeric>

namespace sampler_node
{
std::vector<sampler_common::Trajectory> generateCandidateTrajectories(
  const sampler_common::Configuration & initial_configuration,
  const sampler_common::Trajectory & previous_trajectory,
  const sampler_common::transform::Spline2D & path_spline,
  const autoware_auto_planning_msgs::msg::Path & path_msg, gui::GUI & gui,
  const Parameters & params)
{
  const auto reuse_length_step =
    params.sampling.reuse_max_length_max / params.sampling.reuse_samples;

  std::vector<sampler_common::Trajectory> trajectories;
  sampler_common::Trajectory base_trajectory;
  const auto move_to_trajectories = [&](auto & trajs_to_move) {
    trajectories.insert(
      trajectories.end(), std::make_move_iterator(trajs_to_move.begin()),
      std::make_move_iterator(trajs_to_move.end()));
  };

  if (params.sampling.enable_frenet) {
    auto frenet_trajs = generateFrenetTrajectories(
      initial_configuration, base_trajectory, path_msg, path_spline, params);
    gui.setFrenetTrajectories(frenet_trajs);
    move_to_trajectories(frenet_trajs);
  }
  if (params.sampling.enable_bezier) {
    // TODO(Maxime CLEMENT): add frenet velocity profiles to the bezier paths
    // const auto bezier_paths =
    //   generateBezierPaths(initial_configuration, base_path, path_msg, path_spline, params);
    // move_to_paths(bezier_paths);
  }

  for (auto reuse_max_length = reuse_length_step;
       reuse_max_length <= params.sampling.reuse_max_length_max;
       reuse_max_length += reuse_length_step) {
    // TODO(Maxime CLEMENT): change to a clearer flow: 1-generate base_traj 2-check valid base traj
    // 3-calc initial config
    if (sampler_common::tryToReuseTrajectory(
          previous_trajectory, initial_configuration.pose, std::numeric_limits<double>::max(),
          reuse_max_length, params.sampling.reuse_max_deviation, params.constraints,
          base_trajectory)) {
      const auto cost_mult = 1.0 - 0.3 * reuse_length_step / params.sampling.reuse_max_length_max;
      sampler_common::Configuration end_of_reused_trajectory;
      end_of_reused_trajectory.pose = base_trajectory.points.back();
      end_of_reused_trajectory.heading = base_trajectory.yaws.back();
      end_of_reused_trajectory.curvature = base_trajectory.curvatures.back();
      end_of_reused_trajectory.velocity = base_trajectory.longitudinal_velocities.back();
      end_of_reused_trajectory.acceleration = base_trajectory.longitudinal_accelerations.back();
      if (params.sampling.enable_frenet) {
        const auto trajectories_from_prev_trajectory = generateFrenetTrajectories(
          end_of_reused_trajectory, base_trajectory, path_msg, path_spline, params);
        gui.setFrenetTrajectories(trajectories_from_prev_trajectory, base_trajectory);
        for (const auto & trajectory : trajectories_from_prev_trajectory) {
          trajectories.push_back(base_trajectory.extend(trajectory));
          trajectories.back().cost *= cost_mult;
        }
      }
      if (params.sampling.enable_bezier) {
        //  const auto trajectories_from_prev_trajectory =
        //    generateBeziertrajectories(end_of_reused_trajectory, base_trajectory, path_msg,
        //    path_spline, params);
        //  for (const auto & trajectory : trajectories_from_prev_trajectory) {
        //    trajectories.push_back(base_trajectory.extend(trajectory));
        //    trajectories.back().cost *= cost_mult;
        //  }
      }
    } else {
      // If we fail to reuse length L from the previous path, all lengths > L will also fail.
      break;
    }
  }
  return trajectories;
}

std::vector<frenet_planner::Trajectory> generateFrenetTrajectories(
  const sampler_common::Configuration & initial_configuration,
  const sampler_common::Trajectory & base_traj, const autoware_auto_planning_msgs::msg::Path & path,
  const sampler_common::transform::Spline2D & path_spline, const Parameters & params)
{
  const auto base_length =
    std::accumulate(base_traj.intervals.begin(), base_traj.intervals.end(), 0.0);
  const auto base_duration = base_traj.times.empty() ? 0.0 : base_traj.times.back();
  const auto sampling_parameters = prepareSamplingParameters(
    initial_configuration, path, base_length, base_duration, path_spline, params);

  frenet_planner::FrenetState initial_frenet_state;
  const auto final_s =
    path_spline.frenet({path.points.back().pose.position.x, path.points.back().pose.position.y}).s;
  initial_frenet_state.position = path_spline.frenet(initial_configuration.pose);
  initial_frenet_state.longitudinal_velocity = initial_configuration.velocity;
  initial_frenet_state.longitudinal_acceleration = initial_configuration.acceleration;
  auto trajectories =
    frenet_planner::generateTrajectories(path_spline, initial_frenet_state, sampling_parameters);

  frenet_planner::SamplingParameters stopping_sampling_parameters;
  stopping_sampling_parameters.time_resolution = params.sampling.resolution;
  stopping_sampling_parameters.target_lateral_velocities = {0.0};
  stopping_sampling_parameters.target_lateral_accelerations = {0.0};
  stopping_sampling_parameters.target_lateral_positions =
    sampling_parameters.target_lateral_positions;
  stopping_sampling_parameters.target_longitudinal_velocities = {0.0};
  stopping_sampling_parameters.target_longitudinal_accelerations = {
    0.0, params.constraints.hard.min_acceleration / 2, params.constraints.hard.min_acceleration};
  bool close_to_end;
  for (const auto target_const_decel :
       stopping_sampling_parameters.target_longitudinal_accelerations) {
    const auto duration = -initial_frenet_state.longitudinal_velocity / target_const_decel;
    const auto displacement = 0.5 * initial_frenet_state.longitudinal_velocity * duration;
    const auto target_s = initial_frenet_state.position.s + displacement;
    if (target_s < final_s) {
      stopping_sampling_parameters.target_longitudinal_positions.push_back(target_s);
      stopping_sampling_parameters.target_durations.push_back(duration);
    } else {
      close_to_end = true;
    }
  }
  if (close_to_end) stopping_sampling_parameters.target_longitudinal_positions.push_back(final_s);
  auto stopping_trajectories = frenet_planner::generateTrajectories(
    path_spline, initial_frenet_state, stopping_sampling_parameters);
  for (auto & traj : stopping_trajectories) traj.cost += 1000.0;
  if (close_to_end) return trajectories;
  trajectories.insert(
    trajectories.end(), stopping_trajectories.begin(), stopping_trajectories.end());
  return trajectories;
}
}  // namespace sampler_node
