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

#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__RETROACTIVE_RESAMPLER_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__RETROACTIVE_RESAMPLER_HPP_

#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

#include <iostream>
#include <optional>
namespace pcdless
{
namespace modularized_particle_filter
{
class RetroactiveResampler
{
public:
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using OptParticleArray = std::optional<ParticleArray>;
  RetroactiveResampler(
    float resampling_interval_seconds, int number_of_particles, int max_history_num);

  OptParticleArray retroactive_weighting(
    const ParticleArray & predicted_particles,
    const ParticleArray::ConstSharedPtr & weighted_particles);

  std::optional<ParticleArray> resampling(const ParticleArray & predicted_particles);

private:
  // The minimum resampling interval is longer than this.
  // It is assumed that users will call the resampling() function frequently.
  const float resampling_interval_seconds_;
  // Number of particles to be managed.
  const int number_of_particles_;
  // Number of updates to keep resampling history.
  // Resampling records prior to this will not be kept.
  const int max_history_num_;

  // Previous resampling time (At the first, it has nullopt)
  std::optional<double> previous_resampling_time_opt_;

  // This is handled like ring buffer.
  // It keeps track of which particles each particle has transformed into at each resampling.
  // NOTE: boost::circle_buffer<std::vector<int>> is better?
  std::vector<std::vector<int>> resampling_history_;

  // Working Pointer? I guess.
  int resampling_history_wp_;

  void initialize_resample_history();
};
}  // namespace modularized_particle_filter
}  // namespace pcdless

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__RETROACTIVE_RESAMPLER_HPP_
