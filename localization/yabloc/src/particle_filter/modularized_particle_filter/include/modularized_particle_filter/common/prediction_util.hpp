#ifndef MODULARIZED_PARTICLE_FILTER__COMMON__PREDICTION_UTIL_HPP_
#define MODULARIZED_PARTICLE_FILTER__COMMON__PREDICTION_UTIL_HPP_

#include <eigen3/Eigen/SVD>

#include <numeric>
#include <optional>
#include <random>
#include <vector>

namespace modularized_particle_filter::util
{
std::random_device seed_gen;
std::default_random_engine engine(seed_gen());

Eigen::Vector2d nrand2d(const Eigen::Matrix2d cov)
{
  Eigen::JacobiSVD<Eigen::Matrix2d> svd;
  svd.compute(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector2d std = svd.singularValues();
  std = std.cwiseMax(0.01);

  std::normal_distribution<> dist(0.0, 1.0);
  Eigen::Vector2d xy;
  xy.x() = std::sqrt(std.x()) * dist(engine);
  xy.y() = std::sqrt(std.y()) * dist(engine);
  return svd.matrixU() * xy;
}

double nrand(double std)
{
  std::normal_distribution<> dist(0.0, std);
  return dist(engine);
}

double normalizeRadian(const double rad, const double min_rad = -M_PI)
{
  const auto max_rad = min_rad + 2 * M_PI;

  const auto value = std::fmod(rad, 2 * M_PI);

  if (min_rad <= value && value <= max_rad) {
    return value;
  }

  return value - std::copysign(2 * M_PI, value);
}

}  // namespace modularized_particle_filter::util
#endif  // MODULARIZED_PARTICLE_FILTER__COMMON__PREDICTION_UTIL_HPP_
