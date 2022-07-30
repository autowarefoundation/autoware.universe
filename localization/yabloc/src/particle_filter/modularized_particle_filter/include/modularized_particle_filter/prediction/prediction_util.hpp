#ifndef MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTION_UTIL_HPP_
#define MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTION_UTIL_HPP_

#include <numeric>
#include <optional>
#include <random>
#include <vector>
namespace prediction_util
{
std::random_device seed_gen;
std::default_random_engine engine(seed_gen());

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

}  // namespace prediction_util
#endif  // MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTION_UTIL_HPP_
