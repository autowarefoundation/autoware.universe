#ifndef MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTION_UTIL_HPP_
#define MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTION_UTIL_HPP_

#include <complex>
#include <numeric>
#include <vector>
#include <optional>

namespace prediction_util
{

double nrand(double n)
{
  double r;
  // Box-Muller
  r = n * sqrt(-2.0 * log(static_cast<double>(rand()) / RAND_MAX)) *
    cos(2.0 * M_PI * rand() / RAND_MAX);
  return r;
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

double meanRadian(
  const std::vector<double> & angles,
  const std::vector<double> & weights)
{
  std::complex<double> c{};
  for (int i{0}; i < static_cast<int>(angles.size()); i++) {
    c += weights[i] * std::polar(1.0, angles[i]);
  }
  std::complex<double> cw{std::accumulate(weights.begin(), weights.end(), 0.0)};
  return std::arg(c / cw);
}

} // namespace prediction_util
#endif // MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTION_UTIL_HPP_
