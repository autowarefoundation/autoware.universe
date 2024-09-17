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

#ifndef INTERPOLATION__SPLINE_INTERPOLATION_POINTS_2D_HPP_
#define INTERPOLATION__SPLINE_INTERPOLATION_POINTS_2D_HPP_

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "interpolation/spline_interpolation.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <optional>
#include <vector>

namespace interpolation
{

template <typename T>
std::vector<double> spline_yaw_from_points(const std::vector<T> & points);
}  // namespace interpolation

// non-static points spline interpolation
// NOTE: We can calculate yaw from the x and y by interpolation derivatives.
//
// Usage:
// ```
// SplineInterpolationPoints2d spline;
// // memorize pre-interpolation result internally
// spline.calcSplineCoefficients(base_keys, base_values);
// const auto interpolation_result1 = spline.getSplineInterpolatedPoint(
//   base_keys, query_keys1);
// const auto interpolation_result2 = spline.getSplineInterpolatedPoint(
//   base_keys, query_keys2);
// const auto yaw_interpolation_result = spline.getSplineInterpolatedYaw(
//   base_keys, query_keys1);
// ```
class SplineInterpolationPoints2d
{
public:
  SplineInterpolationPoints2d() = default;
  template <typename T>
  explicit SplineInterpolationPoints2d(const std::vector<T> & points)
  {
    std::vector<geometry_msgs::msg::Point> points_inner;
    for (const auto & p : points) {
      points_inner.push_back(autoware::universe_utils::getPoint(p));
    }
    calc_coefficients_inner(points_inner);
  }

  // TODO(murooka) implement these functions
  // std::vector<geometry_msgs::msg::Point> getSplineInterpolatedPoints(const double width);
  // std::vector<geometry_msgs::msg::Pose> getSplineInterpolatedPoses(const double width);

  // pose (= getSplineInterpolatedPoint + getSplineInterpolatedYaw)
  [[nodiscard]] geometry_msgs::msg::Pose compute_pose(const size_t idx, const double s) const;

  // point
  [[nodiscard]] geometry_msgs::msg::Point compute_point(const size_t idx, const double s) const;

  // yaw
  [[nodiscard]] double compute_yaw(const size_t idx, const double s) const;
  [[nodiscard]] std::vector<double> compute_yaws() const;

  // curvature
  [[nodiscard]] double compute_curvature(const size_t idx, const double s) const;
  [[nodiscard]] std::vector<double> compute_curvaures() const;

  [[nodiscard]] size_t get_size() const { return base_s_vec_.size(); }
  [[nodiscard]] size_t get_offset_index(const size_t idx, const double offset) const;
  [[nodiscard]] double get_accumulated_length(const size_t idx) const;

private:
  void calc_coefficients_inner(const std::vector<geometry_msgs::msg::Point> & points);

  std::optional<SplineInterpolation> spline_x_;
  std::optional<SplineInterpolation> spline_y_;
  std::optional<SplineInterpolation> spline_z_;

  std::vector<double> base_s_vec_;
};

#endif  // INTERPOLATION__SPLINE_INTERPOLATION_POINTS_2D_HPP_
