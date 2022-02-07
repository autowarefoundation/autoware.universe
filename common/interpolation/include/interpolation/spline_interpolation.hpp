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

#include "interpolation/interpolation_utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace interpolation
{
// NOTE: X(s) = a_i (s - s_i)^3 + b_i (s - s_i)^2 + c_i (s - s_i) + d_i : (i = 0, 1, ... N-1)
struct MultiSplineCoef
{
  MultiSplineCoef() = default;

  explicit MultiSplineCoef(const size_t num_spline)
  {
    a.resize(num_spline);
    b.resize(num_spline);
    c.resize(num_spline);
    d.resize(num_spline);
  }

  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  std::vector<double> d;
};

// static spline interpolation functions
std::vector<double> slerp(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys);

// std::vector<double> slerpDiff(
//   const std::vector<double> & base_keys, const std::vector<double> & base_values,
//   const std::vector<double> & query_keys);

// TODO(murooka) use template
// template <typename T>
// std::vector<double> slerpYawFromPoints(const std::vector<T> & points);
std::vector<double> slerpYawFromPoints(const std::vector<geometry_msgs::msg::Point> & points);
}  // namespace interpolation

// non-static 1-dimensional spline interpolation
//
// Usage:
// ```
// SplineInterpolation1d spline;
// spline.calcSplineCoefficients(base_keys, base_values);  // memorize pre-interpolation result
// internally const auto interpolation_result1 = spline.getSplineInterpolatedValues(base_keys,
// query_keys1); const auto interpolation_result2 = spline.getSplineInterpolatedValues(base_keys,
// query_keys2);
// ```
class SplineInterpolation1d
{
public:
  SplineInterpolation1d() = default;

  void calcSplineCoefficients(
    const std::vector<double> & base_keys, const std::vector<double> & base_values);

  std::vector<double> getSplineInterpolatedValues(
    const std::vector<double> & base_keys, const std::vector<double> & query_keys) const;

private:
  interpolation::MultiSplineCoef multi_spline_coef_;
};

// non-static points spline interpolation
// NOTE: We can calculate yaw from the x and y by interpolation derivatives.
//
// Usage:
// ```
// SplineInterpolationPoint spline;
// spline.calcSplineCoefficients(base_keys, base_values);  // memorize pre-interpolation result
// internally const auto interpolation_result1 = spline.getSplineInterpolatedPoint(base_keys,
// query_keys1); const auto interpolation_result2 = spline.getSplineInterpolatedPoint(base_keys,
// query_keys2); const auto yaw_interpolation_result = spline.getSplineInterpolatedValues(base_keys,
// query_keys1);
// ```
class SplineInterpolationPoint
{
public:
  SplineInterpolationPoint() = default;

  // TODO(murooka) use template
  // template <typename T>
  // void calcSplineCoefficients(const std::vector<T> & points);
  void calcSplineCoefficients(const std::vector<geometry_msgs::msg::Point> & points);

  // TODO(murooka) implement these functions
  // std::vector<geometry_msgs::msg::Point> getSplineInterpolatedPoints(const double width);
  // std::vector<geometry_msgs::msg::Pose> getSplineInterpolatedPoses(const double width);

  geometry_msgs::msg::Point getSplineInterpolatedPoint(const size_t idx, const double s) const;
  double getSplineInterpolatedYaw(const size_t idx, const double s) const;

  double getAccumulatedDistance(const size_t idx) const;

private:
  interpolation::MultiSplineCoef multi_spline_coef_x_;
  interpolation::MultiSplineCoef multi_spline_coef_y_;
  std::vector<double> base_s_vec_;
};

#endif  // INTERPOLATION__SPLINE_INTERPOLATION_HPP_
