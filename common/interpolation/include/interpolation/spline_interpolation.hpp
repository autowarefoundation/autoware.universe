// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INTERPOLATION__SPLINE_INTERPOLATION_HPP_
#define INTERPOLATION__SPLINE_INTERPOLATION_HPP_

// #include "autoware/universe_utils/geometry/geometry.hpp"
// #include "interpolation/interpolation_utils.hpp"

#include <Eigen/Core>

#include <cmath>
#include <vector>

namespace interpolation
{

namespace detail
{
Eigen::VectorXd solve_tridiagonal_matrix_algorithm(
  const Eigen::Ref<const Eigen::VectorXd> & a, const Eigen::Ref<const Eigen::VectorXd> & b,
  const Eigen::Ref<const Eigen::VectorXd> & c, const Eigen::Ref<const Eigen::VectorXd> & d);
}  // namespace detail

// static spline interpolation functions
std::vector<double> spline(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys);

std::vector<double> spline_by_akima(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys);
}  // namespace interpolation

class SplineInterpolation
{
public:
  SplineInterpolation(
    const std::vector<double> & base_keys, const std::vector<double> & base_values);

  //!< @brief get values of spline interpolation on designated sampling points.
  //!< @details Assuming that query_keys are t vector for sampling, and interpolation is for x,
  //            meaning that spline interpolation was applied to x(t),
  //            return value will be x(t) vector
  [[nodiscard]] std::vector<double> compute(const std::vector<double> & query_keys) const;

  //!< @brief get 1st differential values of spline interpolation on designated sampling points.
  //!< @details Assuming that query_keys are t vector for sampling, and interpolation is for x,
  //            meaning that spline interpolation was applied to x(t),
  //            return value will be dx/dt(t) vector
  [[nodiscard]] std::vector<double> compute_diff(const std::vector<double> & query_keys) const;

  //!< @brief get 2nd differential values of spline interpolation on designated sampling points.
  //!< @details Assuming that query_keys are t vector for sampling, and interpolation is for x,
  //            meaning that spline interpolation was applied to x(t),
  //            return value will be d^2/dt^2(t) vector
  [[nodiscard]] std::vector<double> compute_quad_diff(const std::vector<double> & query_keys) const;

  [[nodiscard]] size_t get_size() const { return base_keys_.size(); }

private:
  Eigen::VectorXd a_;
  Eigen::VectorXd b_;
  Eigen::VectorXd c_;
  Eigen::VectorXd d_;

  std::vector<double> base_keys_;

  void calc_spline_coefficients(
    const std::vector<double> & base_keys, const std::vector<double> & base_values);

  [[nodiscard]] Eigen::Index get_index(double key) const;
};

#endif  // INTERPOLATION__SPLINE_INTERPOLATION_HPP_
