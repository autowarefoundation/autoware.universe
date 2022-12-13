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
#include "sampler_node/prepare_inputs.hpp"

#include <helper_functions/angle_utils.hpp>
#include <interpolation/linear_interpolation.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <iterator>
#include <limits>
#include <numeric>

namespace sampler_node
{
std::vector<sampler_common::Trajectory> generateCandidateTrajectories(
  const sampler_common::Configuration & initial_configuration,
  const sampler_common::Trajectory & base_trajectory,
  const sampler_common::transform::Spline2D & path_spline,
  const autoware_auto_planning_msgs::msg::Path & path_msg, gui::GUI & gui,
  const Parameters & params)
{
  std::vector<sampler_common::Trajectory> trajectories;
  (void)gui;

  if (params.sampling.enable_frenet) {
    const auto frenet_trajectories = generateFrenetTrajectories(
      initial_configuration, base_trajectory, path_msg, path_spline, params);
    gui.addFrenetTrajectories(frenet_trajectories);
    for (const auto & trajectory : frenet_trajectories) {
      trajectories.push_back(base_trajectory.extend(trajectory));
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
  return trajectories;
}

std::vector<frenet_planner::Trajectory> generateFrenetTrajectories(
  const sampler_common::Configuration & initial_configuration,
  const sampler_common::Trajectory & base_traj, const autoware_auto_planning_msgs::msg::Path & path,
  const sampler_common::transform::Spline2D & path_spline, const Parameters & params)
{
  const auto gen_fn = initial_configuration.velocity < 1.0
                        ? frenet_planner::generateLowVelocityTrajectories
                        : frenet_planner::generateTrajectories;
  const auto base_length = base_traj.lengths.empty() ? 0.0 : base_traj.lengths.back();
  const auto base_duration = base_traj.times.empty() ? 0.0 : base_traj.times.back();
  const auto sampling_parameters = prepareSamplingParameters(
    initial_configuration, path, base_length, base_duration, path_spline, params);
  std::cout << "Frenet Generation with " << sampling_parameters.parameters.size()
            << " parameters\n";

  frenet_planner::FrenetState initial_frenet_state;
  const auto final_s =
    path_spline.frenet({path.points.back().pose.position.x, path.points.back().pose.position.y}).s -
    params.sampling.resolution;
  initial_frenet_state.position = path_spline.frenet(initial_configuration.pose);
  const auto dyaw = autoware::common::helper_functions::wrap_angle(
    initial_configuration.heading - path_spline.yaw(initial_frenet_state.position.s));
  initial_frenet_state.longitudinal_velocity = std::cos(dyaw) * initial_configuration.velocity;
  initial_frenet_state.lateral_velocity = std::sin(dyaw) * initial_configuration.velocity;
  initial_frenet_state.longitudinal_acceleration =
    std::cos(dyaw) * initial_configuration.acceleration;
  initial_frenet_state.lateral_acceleration = std::sin(dyaw) * initial_configuration.acceleration;
  auto trajectories = gen_fn(path_spline, initial_frenet_state, sampling_parameters);
  // Stopping trajectories
  bool can_stop = initial_frenet_state.longitudinal_velocity > 0.1;
  // TODO(Maxime): find better condition to target the last path point
  bool close_to_end = (final_s - initial_frenet_state.position.s) < 50.0;
  if (can_stop) {
    frenet_planner::SamplingParameters stopping_sampling_parameters;
    stopping_sampling_parameters.resolution = params.sampling.resolution;
    frenet_planner::SamplingParameter sp;
    sp.target_state.longitudinal_velocity = 0.0;
    sp.target_state.longitudinal_acceleration = 0.0;
    sp.target_state.lateral_velocity = 0.0;
    sp.target_state.lateral_acceleration = 0.0;
    for (const auto target_const_decel :
         {params.constraints.hard.min_acceleration / 2.0,
          params.constraints.hard.min_acceleration}) {
      const auto duration = -initial_frenet_state.longitudinal_velocity / target_const_decel;
      const auto displacement = 0.5 * initial_frenet_state.longitudinal_velocity * duration;
      const auto target_s = initial_frenet_state.position.s + displacement;
      sp.target_duration = duration;
      if (target_s < final_s) {
        sp.target_state.position.s = target_s;
        for (const auto target_lateral_position :
             params.sampling.frenet.manual.target_lateral_positions) {
          sp.target_state.position.d = target_lateral_position;
          stopping_sampling_parameters.parameters.push_back(sp);
        }
      }
    }
    if (close_to_end) {
      sp.target_state.position.s = final_s;
      sp.target_state.position.d = 0.0;
      sp.target_state.longitudinal_velocity = 0.0;
      sp.target_state.lateral_velocity = 0.0;
      sp.target_state.lateral_acceleration = 0.0;
      // const auto delta_s = final_s - initial_frenet_state.position.s;
      constexpr auto min_decel_duration = 1.0;
      const auto max_decel_duration =
        -initial_frenet_state.longitudinal_velocity / params.constraints.hard.min_acceleration;
      constexpr auto decel_samples = 10;
      std::vector<double> durations = {min_decel_duration};
      if (max_decel_duration > min_decel_duration) {
        durations.push_back(max_decel_duration);
        const auto ratio_step = 1.0 / decel_samples;
        for (auto ratio = ratio_step; ratio < 1.0; ratio += ratio_step) {
          const auto duration = interpolation::lerp(min_decel_duration, max_decel_duration, ratio);
          durations.push_back(duration);
        }
      }
      for (const auto duration : durations) {
        sp.target_duration = duration;
        for (const auto target_dec : {-1.0, -0.5, 0.0}) {
          sp.target_state.longitudinal_acceleration = target_dec;
          for (const auto target_lateral_position :
               params.sampling.frenet.manual.target_lateral_positions) {
            sp.target_state.position.d = target_lateral_position;
            stopping_sampling_parameters.parameters.push_back(sp);
          }
        }
      }
    }
    auto stopping_trajectories = frenet_planner::generateLowVelocityTrajectories(
      path_spline, initial_frenet_state, stopping_sampling_parameters);
    std::cout << "\t Additional " << stopping_trajectories.size() << " stopping trajectories"
              << std::endl;
    for (auto & traj : stopping_trajectories) {
      traj.tag += " stopping";
    }
    trajectories.insert(
      trajectories.end(), stopping_trajectories.begin(), stopping_trajectories.end());
  }
  return trajectories;
}
}  // namespace sampler_node
