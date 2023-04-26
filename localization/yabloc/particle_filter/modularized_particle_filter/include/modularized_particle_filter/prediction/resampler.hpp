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

#include <rclcpp/logger.hpp>

#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <optional>
namespace pcdless::modularized_particle_filter
{

class History
{
public:
  History(int max_history_num, int number_of_particles)
  : max_history_num_(max_history_num), number_of_particles_(number_of_particles)
  {
    resampling_history_.resize(max_history_num);

    for (auto & generation : resampling_history_) {
      generation.resize(number_of_particles);
      std::iota(generation.begin(), generation.end(), 0);
    }
  }

  bool check_history_validity() const
  {
    for (auto & generation : resampling_history_) {
      bool result = std::any_of(generation.begin(), generation.end(), [this](int x) {
        return x < 0 || x >= number_of_particles_;
      });

      if (result) {
        return false;
      }
    }
    return true;
  }

  std::vector<int> & operator[](int generation_id)
  {
    return resampling_history_.at(generation_id % max_history_num_);
  }

  const std::vector<int> & operator[](int generation_id) const
  {
    return resampling_history_.at(generation_id % max_history_num_);
  }

private:
  // Number of updates to keep resampling history.
  // Resampling records prior to this will not be kept.
  const int max_history_num_;
  const int number_of_particles_;
  std::vector<std::vector<int>> resampling_history_;
};

class RetroactiveResampler
{
public:
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using OptParticleArray = std::optional<ParticleArray>;

  RetroactiveResampler(
    float resampling_interval_seconds, int number_of_particles, int max_history_num);

  OptParticleArray add_weight_retroactively(
    const ParticleArray & predicted_particles, const ParticleArray & weighted_particles);

  std::optional<ParticleArray> resample(const ParticleArray & predicted_particles);

private:
  // The minimum resampling interval is longer than this.
  // It is assumed that users will call the resampling() function frequently.
  const float resampling_interval_seconds_;
  // Number of updates to keep resampling history.
  // Resampling records prior to this will not be kept.
  const int max_history_num_;
  // Number of particles to be managed.
  const int number_of_particles_;
  //
  rclcpp::Logger logger_;
  // Previous resampling time
  std::optional<double> previous_resampling_time_opt_{std::nullopt};
  // This is handled like ring buffer.
  // It keeps track of which particles each particle has transformed into at each resampling.
  History resampling_history_;
  // Indicates how many times the particles were resampled.
  int latest_resampling_generation_;

  // Random generator from 0 to 1
  double random_from_01_uniformly() const;
  // Check the sanity of the particles obtained from the particle corrector.
  bool check_weighted_particles_validity(const ParticleArray & weighted_particles) const;
};
}  // namespace pcdless::modularized_particle_filter

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__RETROACTIVE_RESAMPLER_HPP_
