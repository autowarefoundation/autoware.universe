#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__RETROACTIVE_RESAMPLER_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__RETROACTIVE_RESAMPLER_HPP_

#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

#include <iostream>
#include <optional>
#include <tuple>

class RetroactiveResampler
{
public:
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using OptParticleArray = std::optional<ParticleArray>;
  RetroactiveResampler(
    float resampling_interval_seconds, int number_of_particles, bool dynamic_resampling_);

  OptParticleArray retroactiveWeighting(
    const ParticleArray & predicted_particles,
    const ParticleArray::ConstSharedPtr & weighted_particles);

  std::optional<ParticleArray> resampling(const ParticleArray & predicted_particles);

  static std::vector<std::vector<int>> initializeResampleHistory(
    int number_of_particles, int max_history_num);

private:
  const int number_of_particles_;

  const int max_history_num_;
  const float resampling_interval_seconds_;
  const bool dynamic_resampling_;

  std::optional<double> previous_resampling_time_opt_;

  // NOTE: circle_buffer<std::vector<int>> is better?
  std::vector<std::vector<int>> resampling_history_;
  int resampling_history_wp_;
};

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__RETROACTIVE_RESAMPLER_HPP_
