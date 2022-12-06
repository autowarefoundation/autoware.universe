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

#ifndef SAMPLER_NODE__CALCULATE_SAMPLING_PARAMETERS_HPP_
#define SAMPLER_NODE__CALCULATE_SAMPLING_PARAMETERS_HPP_

#include "frenet_planner/structures.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"
#include "sampler_node/parameters.hpp"

#include <interpolation/linear_interpolation.hpp>
#include <motion_common/trajectory_common.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>

#include <cmath>
#include <limits>

struct VelocityExtremum
{
  double min = std::numeric_limits<double>::max();
  double min_s{};
  double max = std::numeric_limits<double>::min();
  double max_s{};
};

inline VelocityExtremum findVelocityExtremum(
  const autoware_auto_planning_msgs::msg::Path & path,
  const double max_distance = std::numeric_limits<double>::max())
{
  VelocityExtremum extremum;
  if (path.points.empty()) return extremum;
  extremum.min = path.points.front().longitudinal_velocity_mps;
  extremum.max = path.points.front().longitudinal_velocity_mps;
  auto arc_length = 0.0;
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto vel = static_cast<double>(path.points[i].longitudinal_velocity_mps);
    if (vel < extremum.min) {
      extremum.min = vel;
      extremum.min_s = arc_length;
    } else if (vel > extremum.max) {
      extremum.max = vel;
      extremum.max_s = arc_length;
    }
    arc_length += autoware::common::geometry::distance_2d(
      path.points[i - 1].pose.position, path.points[i].pose.position);
    if (arc_length > max_distance) break;
  }
  if (extremum.min_s == extremum.max_s) {  // only one velocity value in the Path
    extremum.min_s = 0.0;
    extremum.max_s = max_distance;
  }
  return extremum;
}

inline double distanceBetweenVelocities(
  const double from_velocity, const double to_velocity, const double acceleration)
{
  return std::abs((to_velocity * to_velocity - from_velocity * from_velocity) / 2 * acceleration);
}

inline double durationBetweenVelocities(
  const double from_velocity, const double to_velocity, const double acceleration)
{
  return std::abs((to_velocity - from_velocity) / acceleration);
}

inline void gridSamplingParameters(
  frenet_planner::SamplingParameters & sampling_parameters, const double min_velocity,
  const double max_velocity, const double min_duration, const double max_duration,
  const int velocity_samples, const int duration_samples)
{
  const auto vel_step = (max_velocity - min_velocity) / velocity_samples;
  const auto dur_step = (max_duration - min_duration) / duration_samples;
  std::vector<double> target_velocities = {max_velocity};
  std::vector<double> target_durations = {max_duration};
  for (auto vel = min_velocity; vel < max_velocity; vel += vel_step)
    target_velocities.push_back(vel);
  for (auto duration = min_duration; duration < max_duration; duration += dur_step)
    target_durations.push_back(duration);

  frenet_planner::SamplingParameter sp;
  sp.target_state.longitudinal_acceleration = 0.0;
  sp.target_state.lateral_acceleration = 0.0;
  for (const auto target_vel : target_velocities) {
    sp.target_state.longitudinal_velocity = target_vel;
    for (const auto target_duration : target_durations) {
      sp.target_duration = target_duration;
      sampling_parameters.parameters.push_back(sp);
    }
  }
}

inline void calculateTargets(
  frenet_planner::SamplingParameters & sampling_parameters,
  const sampler_common::Configuration & initial_configuration,
  const autoware_auto_planning_msgs::msg::Path & path,
  const sampler_common::transform::Spline2D & path_spline, const Parameters & params,
  const double base_length, const double base_duration)
{
  const auto start_s = path_spline.frenet(initial_configuration.pose).s;
  const auto final_s =
    path_spline.frenet({path.points.back().pose.position.x, path.points.back().pose.position.y}).s;
  frenet_planner::SamplingParameter sp;
  if (params.sampling.frenet.manual.enable) {
    for (const auto target_lat_pos : params.sampling.frenet.manual.target_lateral_positions) {
      sp.target_state.position.d = target_lat_pos;
      for (const auto target_lat_vel : params.sampling.frenet.manual.target_lateral_velocities) {
        sp.target_state.lateral_velocity = target_lat_vel;
        for (const auto target_lat_acc :
             params.sampling.frenet.manual.target_lateral_accelerations) {
          sp.target_state.lateral_acceleration = target_lat_acc;
          for (const auto target_vel :
               params.sampling.frenet.manual.target_longitudinal_velocities) {
            sp.target_state.longitudinal_velocity = target_vel;
            for (const auto target_acc :
                 params.sampling.frenet.manual.target_longitudinal_accelerations) {
              sp.target_state.longitudinal_acceleration = target_acc;
              for (const auto d : params.sampling.frenet.manual.target_durations) {
                if (d - base_duration > 0.0) {
                  sp.target_duration = d - base_duration;
                  for (const auto offset :
                       params.sampling.frenet.manual.target_longitudinal_position_offsets) {
                    if (offset - base_length > 0.0) {
                      const auto target_s = start_s + offset - base_length;
                      if (target_s <= final_s) {
                        sp.target_state.position.s = target_s;
                        sampling_parameters.parameters.push_back(sp);
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  if (params.sampling.frenet.calc.enable) {
    sp.target_state.lateral_velocity = 0.0;
    sp.target_state.longitudinal_acceleration = 0.0;
    sp.target_state.lateral_acceleration = 0.0;
    // TODO(Maxime CLEMENT): don't hardcode 50m here
    const auto velocity_extremum = findVelocityExtremum(path, 50.0);
    const auto min_vel =
      std::min(velocity_extremum.min, params.constraints.hard.max_velocity / 2.0);
    const auto max_vel = std::min(velocity_extremum.max, params.constraints.hard.max_velocity);
    const auto samples = params.sampling.frenet.calc.target_longitudinal_velocity_samples;
    for (auto i = 0; i < samples; ++i) {
      const auto ratio = static_cast<double>(i) / (samples - 1.0);
      const auto target_vel = interpolation::lerp(min_vel, max_vel, ratio);
      const auto distance =
        interpolation::lerp(velocity_extremum.min_s, velocity_extremum.max_s, ratio);
      if (distance < params.sampling.resolution) continue;
      sp.target_state.position.s = start_s + distance;
      const auto confortable_target_vel = std::sqrt(
        initial_configuration.velocity * initial_configuration.velocity +
        2 * params.sampling.confortable_acceleration * distance);
      sp.target_state.longitudinal_velocity = std::min(target_vel, confortable_target_vel);
      if (sp.target_state.longitudinal_velocity == 0.0 && initial_configuration.velocity == 0.0) {
        // assuming bang band velocity profile
        const auto dist_to_max_vel =
          (max_vel * max_vel) / (2 * params.sampling.confortable_acceleration);
        if (dist_to_max_vel * 2.0 > distance) {
          // no time to reach max vel, assume "triangle" velocity profile to a reduced max vel
          const auto new_max_vel = std::sqrt(params.sampling.confortable_acceleration * distance);
          sp.target_duration = 4.0 * distance / new_max_vel;
        } else {
          // enough time to reach max vel, assume /‾‾\ profile, add the duration at const velocity
          const auto duration_to_max_vel = 2.0 * dist_to_max_vel / max_vel;
          const auto dist_at_max_vel = distance - 2.0 * dist_to_max_vel;
          const auto duration_at_max_vel = dist_at_max_vel / max_vel;
          sp.target_duration = 2 * duration_to_max_vel + duration_at_max_vel;
        }
      } else {
        // assuming const accel velocity profile
        sp.target_duration =
          (2 * distance) / (sp.target_state.longitudinal_velocity + initial_configuration.velocity);
      }
      // std::cout << sp << std::endl;
      for (const auto d : params.sampling.frenet.manual.target_lateral_positions) {
        sp.target_state.position.d = d;
        sampling_parameters.parameters.push_back(sp);
      }
    }
  }
}

#endif  // SAMPLER_NODE__CALCULATE_SAMPLING_PARAMETERS_HPP_
