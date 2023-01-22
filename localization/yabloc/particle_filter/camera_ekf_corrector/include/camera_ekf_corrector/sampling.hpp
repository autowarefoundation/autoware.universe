#pragma once
#include <Eigen/Core>

#include <modularized_particle_filter_msgs/msg/particle_array.hpp>

#include <cmath>
#include <random>

namespace pcdless::ekf_corrector
{
extern std::random_device seed_gen;
extern std::default_random_engine engine;

Eigen::Vector2d nrand_2d(const Eigen::Matrix2d & cov);

template <typename T = float>
T nrand(T cov)
{
  std::normal_distribution<T> dist(0.0, std::sqrt(cov));
  return dist(engine);
}

geometry_msgs::msg::Pose mean_pose(
  const modularized_particle_filter_msgs::msg::ParticleArray & particle_array);

Eigen::Matrix3f covariance_of_distribution(
  const modularized_particle_filter_msgs::msg::ParticleArray & array);

float covariance_of_angle_distribution(
  const modularized_particle_filter_msgs::msg::ParticleArray & array);

}  // namespace pcdless::ekf_corrector