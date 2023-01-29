#pragma once
#include <Eigen/Core>

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>

#include <cmath>
#include <random>

namespace pcdless::ekf_corrector
{
extern std::random_device seed_gen;
extern std::default_random_engine engine;

Eigen::Vector2d nrand_2d(const Eigen::Matrix2d & cov);

class NormalDistribution2d
{
public:
  NormalDistribution2d(const Eigen::Matrix2d & cov);
  std::pair<double, Eigen::Vector2d> operator()() const;

private:
  Eigen::Vector2d std_;
  Eigen::Matrix2d rotation_;
};

template <typename T = float>
T nrand(T cov)
{
  std::normal_distribution<T> dist(0.0, std::sqrt(cov));
  return dist(engine);
}

geometry_msgs::msg::PoseWithCovariance debayes_distribution(
  const geometry_msgs::msg::PoseWithCovariance & post,
  const geometry_msgs::msg::PoseWithCovariance & prior);

struct MeanResult
{
  geometry_msgs::msg::Pose pose_;
  Eigen::Matrix3f cov_xyz_;
  float cov_theta_;
};

MeanResult compile_distribution(
  const modularized_particle_filter_msgs::msg::ParticleArray & particle_array);

}  // namespace pcdless::ekf_corrector