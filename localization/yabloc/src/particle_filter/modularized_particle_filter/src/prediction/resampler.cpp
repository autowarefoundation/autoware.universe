#include "modularized_particle_filter/prediction/resampler.hpp"

#include <rclcpp/rclcpp.hpp>

#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

#include <cmath>
#include <numeric>

RetroactiveResampler::RetroactiveResampler(
  float resampling_interval_seconds, int number_of_particles)
: max_history_num_(100), resampling_interval_seconds_(resampling_interval_seconds)
{
  number_of_particles_ = number_of_particles;
  resampling_history_ = initializeResampleHistory(number_of_particles_, max_history_num_);
  resampling_history_wp_ = 0;
}

std::vector<std::vector<int>> RetroactiveResampler::initializeResampleHistory(
  int number_of_particles, int max_history_num)
{
  std::vector<std::vector<int>> resample_history;

  resample_history.resize(max_history_num);
  for (int i = 0; i < max_history_num; i++) {
    resample_history[i].resize(number_of_particles);
    for (int m = 0; m < number_of_particles; m++) {
      resample_history[i][m] = m;
    }
  }

  return resample_history;
}

std::optional<modularized_particle_filter_msgs::msg::ParticleArray>
RetroactiveResampler::retroactiveWeighting(
  const modularized_particle_filter_msgs::msg::ParticleArray & predicted_particles,
  const modularized_particle_filter_msgs::msg::ParticleArray::ConstSharedPtr & weighted_particles)
{
  rclcpp::Logger logger = rclcpp::get_logger("modularized_particle_filter.retroactive_resampler");
  if (!(weighted_particles->id <= resampling_history_wp_ &&                    // not future data
        weighted_particles->id > resampling_history_wp_ - max_history_num_ &&  // not old data
        weighted_particles->id >= 0))                                          // not error data
  {
    RCLCPP_WARN(logger, "out of history");
    return std::nullopt;
  }

  modularized_particle_filter_msgs::msg::ParticleArray reweighted_particles{predicted_particles};

  RCLCPP_INFO_STREAM(
    logger, "current generation " << resampling_history_wp_ << " callback generation "
                                  << weighted_particles->id);

  // initialize corresponding index lookup table
  std::vector<int> index_table(static_cast<int>(weighted_particles->particles.size()));
  for (int m{0}; m < static_cast<int>(weighted_particles->particles.size()); m++) {
    index_table[m] = m;
  }

  // lookup corresponding indices
  for (int history_wp{resampling_history_wp_}; history_wp > weighted_particles->id; history_wp--) {
    for (int m{0}; m < static_cast<int>(weighted_particles->particles.size()); m++) {
      if (
        0 <= index_table[m] &&
        index_table[m] < static_cast<int>(weighted_particles->particles.size())) {
        index_table[m] = resampling_history_[history_wp % max_history_num_][index_table[m]];
      } else {
        return std::nullopt;
      }
    }
  }

  // weighting to current particles
  float sum_weight = 0;
  for (int m{0}; m < static_cast<int>(weighted_particles->particles.size()); m++) {
    reweighted_particles.particles[m].weight *=
      weighted_particles->particles[index_table[m]].weight;
    sum_weight += reweighted_particles.particles[m].weight;
  }
  for (int m{0}; m < static_cast<int>(weighted_particles->particles.size()); m++) {
    reweighted_particles.particles[m].weight /= sum_weight;
  }

  return reweighted_particles;
}

std::optional<modularized_particle_filter_msgs::msg::ParticleArray>
RetroactiveResampler::resampling(
  const modularized_particle_filter_msgs::msg::ParticleArray & predicted_particles)
{
  double current_time{rclcpp::Time(predicted_particles.header.stamp).seconds()};

  if (!previous_resampling_time_opt_.has_value()) {
    previous_resampling_time_opt_ = current_time;
    return std::nullopt;
  }

  if (current_time - previous_resampling_time_opt_.value() <= resampling_interval_seconds_) {
    return std::nullopt;
  }

  modularized_particle_filter_msgs::msg::ParticleArray resampled_particles{predicted_particles};

  resampling_history_wp_++;
  resampled_particles.id = resampling_history_wp_;

  const double num_of_particles_inv{
    1.0 / static_cast<double>(predicted_particles.particles.size())};
  const double sum_weight_inv{
    1.0 / std::accumulate(
            predicted_particles.particles.begin(), predicted_particles.particles.end(), 0.0,
            [](double weight, const modularized_particle_filter_msgs::msg::Particle & ps) {
              return weight + ps.weight;
            })};

  if (!std::isfinite(sum_weight_inv)) {
    exit(EXIT_FAILURE);
  }

  const double r{
    (rand() / static_cast<double>(RAND_MAX)) /
    (static_cast<double>(predicted_particles.particles.size()))};

  double c{predicted_particles.particles[0].weight * sum_weight_inv};

  int i{0};
  for (int m{0}; m < static_cast<int>(predicted_particles.particles.size()); m++) {
    const double u{r + m * num_of_particles_inv};

    while (c < u) {
      i++;
      c += predicted_particles.particles[i].weight * sum_weight_inv;
    }

    resampled_particles.particles[m] = predicted_particles.particles[i];
    resampled_particles.particles[m].weight = num_of_particles_inv;  // TODO:
    resampling_history_[resampling_history_wp_ % max_history_num_][m] = i;
  }

  previous_resampling_time_opt_ = current_time;

  return resampled_particles;
}
