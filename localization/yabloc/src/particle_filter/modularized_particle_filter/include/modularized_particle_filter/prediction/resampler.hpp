#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__RETROACTIVE_RESAMPLER_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__RETROACTIVE_RESAMPLER_HPP_

#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

#include <iostream>
#include <optional>

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

  OptParticleArray retroactiveWeighting(
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

  void initializeResampleHistory();
};
}  // namespace modularized_particle_filter

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__RETROACTIVE_RESAMPLER_HPP_
