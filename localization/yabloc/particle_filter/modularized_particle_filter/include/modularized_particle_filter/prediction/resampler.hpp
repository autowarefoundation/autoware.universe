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

#ifndef MODULARIZED_PARTICLE_FILTER__PREDICTION__RESAMPLER_HPP_
#define MODULARIZED_PARTICLE_FILTER__PREDICTION__RESAMPLER_HPP_

#include "modularized_particle_filter/prediction/resampling_history.hpp"

#include <rclcpp/logger.hpp>

#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

namespace yabloc::modularized_particle_filter
{
class RetroactiveResampler
{
public:
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;

  RetroactiveResampler(int number_of_particles, int max_history_num);

  ParticleArray add_weight_retroactively(
    const ParticleArray & predicted_particles, const ParticleArray & weighted_particles);

  ParticleArray resample(const ParticleArray & predicted_particles);

private:
  // Number of updates to keep resampling history.
  // Resampling records prior to this will not be kept.
  const int max_history_num_;
  // Number of particles to be managed.
  const int number_of_particles_;
  // ROS logger
  rclcpp::Logger logger_;
  // This is handled like ring buffer.
  // It keeps track of which particles each particle has transformed into at each resampling.
  ResamplingHistory resampling_history_;
  // Indicates how many times the particles were resampled.
  int latest_resampling_generation_;

  // Random generator from 0 to 1
  double random_from_01_uniformly() const;
  // Check the sanity of the particles obtained from the particle corrector.
  bool check_weighted_particles_validity(const ParticleArray & weighted_particles) const;
};
}  // namespace yabloc::modularized_particle_filter

#endif  // MODULARIZED_PARTICLE_FILTER__PREDICTION__RESAMPLER_HPP_