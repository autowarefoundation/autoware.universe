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

struct MeanResult
{
  geometry_msgs::msg::Pose pose_;
  Eigen::Matrix3f cov_xyz_;
  float cov_theta_;
};

MeanResult compile_distribution(
  const modularized_particle_filter_msgs::msg::ParticleArray & particle_array);

}  // namespace pcdless::ekf_corrector