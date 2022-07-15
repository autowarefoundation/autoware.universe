#include "modularized_particle_filter/common/mean.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <numeric>

namespace
{
double meanRadian(const std::vector<double> & angles, const std::vector<double> & weights)
{
  std::complex<double> c{};
  for (int i{0}; i < static_cast<int>(angles.size()); i++) {
    c += weights[i] * std::polar(1.0, angles[i]);
  }
  std::complex<double> cw{std::accumulate(weights.begin(), weights.end(), 0.0)};
  return std::arg(c / cw);
}
}  // namespace

geometry_msgs::msg::Pose meanPose(
  const modularized_particle_filter_msgs::msg::ParticleArray & particle_array)
{
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using Pose = geometry_msgs::msg::Pose;

  ParticleArray normalized_particle_array{particle_array};
  Pose mean_pose;

  auto minmax_weight = std::minmax_element(
    normalized_particle_array.particles.begin(), normalized_particle_array.particles.end(),
    [](auto p1, auto p2) { return p1.weight < p2.weight; });

  const float num_of_particles_inv{1.0f / normalized_particle_array.particles.size()};
  const float dif_weight{minmax_weight.second->weight - minmax_weight.first->weight};

  for (auto & particle : normalized_particle_array.particles) {
    if (dif_weight != 0.0f) {
      particle.weight = (particle.weight - minmax_weight.first->weight) / dif_weight;
    } else {
      particle.weight = num_of_particles_inv;
    }
  }

  double sum_weight{std::accumulate(
    normalized_particle_array.particles.begin(), normalized_particle_array.particles.end(), 0.0,
    [](double weight, modularized_particle_filter_msgs::msg::Particle & particle) {
      return weight + particle.weight;
    })};

  if (std::isinf(sum_weight)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("meanPose"), "sum_weight: " << sum_weight);
  }

  std::vector<double> rolls{};
  std::vector<double> pitches{};
  std::vector<double> yaws{};
  std::vector<double> weights{};
  for (modularized_particle_filter_msgs::msg::Particle particle :
       normalized_particle_array.particles) {
    double weight{1.0 / normalized_particle_array.particles.size()};
    if (0.0f < sum_weight) weight = particle.weight / sum_weight;

    mean_pose.position.x += particle.pose.position.x * weight;
    mean_pose.position.y += particle.pose.position.y * weight;
    mean_pose.position.z += particle.pose.position.z * weight;

    double yaw{0.0}, pitch{0.0}, roll{0.0};
    tf2::getEulerYPR(particle.pose.orientation, yaw, pitch, roll);

    rolls.push_back(roll);
    pitches.push_back(pitch);
    yaws.push_back(yaw);
    weights.push_back(weight);
  }

  const double mean_roll{meanRadian(rolls, weights)};
  const double mean_pitch{meanRadian(pitches, weights)};
  const double mean_yaw{meanRadian(yaws, weights)};

  tf2::Quaternion q;
  q.setRPY(mean_roll, mean_pitch, mean_yaw);
  mean_pose.orientation = tf2::toMsg(q);
  return mean_pose;
}