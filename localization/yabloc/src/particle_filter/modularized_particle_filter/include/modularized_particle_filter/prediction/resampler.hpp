#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__RETROACTIVE_RESAMPLER_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__RETROACTIVE_RESAMPLER_HPP_

#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

#include <iostream>
#include <optional>
#include <tuple>

class RetroactiveResampler
{
public:
  RetroactiveResampler(float resampling_interval_seconds, int number_of_particles);

  std::optional<modularized_particle_filter_msgs::msg::ParticleArray> retroactiveWeighting(
    const modularized_particle_filter_msgs::msg::ParticleArray & predicted_particles,
    const modularized_particle_filter_msgs::msg::ParticleArray::ConstSharedPtr &
      weighted_particles);

  std::optional<modularized_particle_filter_msgs::msg::ParticleArray> resampling(
    const modularized_particle_filter_msgs::msg::ParticleArray & predicted_particles);

  static std::vector<std::vector<int>> initializeResampleHistory(
    int number_of_particles, int max_history_num);

protected:
  int number_of_particles_;

private:
  const int max_history_num_;
  float resampling_interval_seconds_;

  std::optional<double> previous_resampling_time_opt_;

  // NOTE: circle_buffer<std::vector<int>> is better?
  std::vector<std::vector<int>> resampling_history_;
  int resampling_history_wp_;
};

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__RETROACTIVE_RESAMPLER_HPP_
