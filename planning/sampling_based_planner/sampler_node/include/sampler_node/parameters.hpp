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

#ifndef SAMPLER_NODE__PARAMETERS_HPP_
#define SAMPLER_NODE__PARAMETERS_HPP_

#include "bezier_sampler/bezier_sampling.hpp"
#include "sampler_common/structures.hpp"

#include <vector>

struct Parameters
{
  sampler_common::Constraints constraints;
  struct
  {
    bool enable_frenet{};
    bool enable_bezier{};
    double resolution{};
    double confortable_acceleration = 1.0;  // TODO(Maxime CLEMENT): read from param file
    struct
    {
      struct
      {
        bool enable{};
        std::vector<double> target_durations{};
        std::vector<double> target_longitudinal_position_offsets{};
        std::vector<double> target_longitudinal_velocities{};
        std::vector<double> target_longitudinal_accelerations{};
        std::vector<double> target_lateral_positions{};
        std::vector<double> target_lateral_velocities{};
        std::vector<double> target_lateral_accelerations{};
      } manual;
      struct
      {
        bool enable{};
        int target_longitudinal_velocity_samples{};
      } calc;
    } frenet;
    bezier_sampler::SamplingParameters bezier{};
  } sampling;

  struct
  {
    bool force_zero_deviation{};
    bool force_zero_heading{};
    double desired_traj_behind_length{};
    std::vector<double> reuse_times{};
    double reuse_max_deviation{};
    bool smooth_reference{};
    double control_points_ratio{};
    double smooth_weight{};
  } preprocessing{};
};

#endif  // SAMPLER_NODE__PARAMETERS_HPP_
