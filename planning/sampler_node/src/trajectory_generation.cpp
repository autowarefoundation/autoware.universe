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

#include "sampler_common/structures.hpp"
#include "sampler_node/plot/plotter.hpp"

#include <bezier_sampler/bezier_sampling.hpp>
#include <frenet_planner/frenet_planner.hpp>
#include <sampler_common/trajectory_reuse.hpp>
#include <sampler_node/prepare_inputs.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <iterator>
#include <limits>
#include <numeric>
#include <vector>

namespace sampler_node
{
std::vector<sampler_common::Trajectory> generateCandidateTrajectories(
  const sampler_common::Configuration & initial_configuration,
  const sampler_common::Trajectory & previous_trajectory,
  const sampler_common::transform::Spline2D & path_spline,
  const autoware_auto_planning_msgs::msg::Path & path_msg, plot::Plotter & plotter,
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
    plotter.plotFrenetTrajectories(frenet_trajs);
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
      plotter.plotCommittedPath(base_trajectory);
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
        plotter.plotFrenetTrajectories(trajectories_from_prev_trajectory);
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
  const sampler_common::Path & base_path, const autoware_auto_planning_msgs::msg::Path & path,
  const sampler_common::transform::Spline2D & path_spline, const Parameters & params)
{
  const auto sampling_parameters = prepareSamplingParameters(
    initial_configuration, path,
    std::accumulate(base_path.intervals.begin(), base_path.intervals.end(), 0.0), path_spline,
    params);

  frenet_planner::FrenetState initial_frenet_state;
  initial_frenet_state.position = path_spline.frenet(initial_configuration.pose);
  initial_frenet_state.longitudinal_velocity = initial_configuration.velocity;
  initial_frenet_state.longitudinal_acceleration = initial_configuration.acceleration;
  return frenet_planner::generateTrajectories(
    path_spline, initial_frenet_state, sampling_parameters);
}
}  // namespace sampler_node
