// Copyright 2024 Tier IV, Inc.
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

#include "autoware/motion_utils/trajectory_container/interpolator/cubic_spline.hpp"

#include <Eigen/Dense>

#include <algorithm>

namespace autoware::motion_utils::trajectory_container::interpolator
{

void CubicSpline::compute_parameters(
  const Eigen::Ref<const Eigen::VectorXd> & axis, const Eigen::Ref<const Eigen::VectorXd> & values)
{
  int n = axis.size() - 1;

  h = axis.tail(n) - axis.head(n);
  a = values.transpose();

  for (int i = 0; i < n; ++i) {
    h(i) = axis(i + 1) - axis(i);
  }

  Eigen::VectorXd alpha(n - 1);
  for (int i = 1; i < n; ++i) {
    alpha(i - 1) = (3.0 / h(i)) * (a(i + 1) - a(i)) - (3.0 / h(i - 1)) * (a(i) - a(i - 1));
  }

  Eigen::VectorXd l(n + 1), mu(n + 1), z(n + 1);
  l(0) = 1.0;
  mu(0) = z(0) = 0.0;

  for (int i = 1; i < n; ++i) {
    l(i) = 2.0 * (axis(i + 1) - axis(i - 1)) - h(i - 1) * mu(i - 1);
    mu(i) = h(i) / l(i);
    z(i) = (alpha(i - 1) - h(i - 1) * z(i - 1)) / l(i);
  }
  b.resize(n);
  d.resize(n);
  c.resize(n + 1);

  l(n) = 1.0;
  z(n) = c(n) = 0.0;

  for (int j = n - 1; j >= 0; --j) {
    c(j) = z(j) - mu(j) * c(j + 1);
    b(j) = (a(j + 1) - a(j)) / h(j) - h(j) * (c(j + 1) + 2.0 * c(j)) / 3.0;
    d(j) = (c(j + 1) - c(j)) / (3.0 * h(j));
  }
}

void CubicSpline::build_(
  const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<double> & values)
{
  this->axis = axis;
  compute_parameters(axis, Eigen::Map<const Eigen::VectorXd>(values.data(), values.size()));
}

double CubicSpline::compute_(const double & s) const
{
  if (s <= this->start()) return a(0);
  if (s >= this->end()) return a(a.size() - 1);

  auto it = std::lower_bound(this->axis.data(), this->axis.data() + this->axis.size(), s);
  int i = std::max(int(it - this->axis.data()) - 1, 0);

  double dx = s - this->axis(i);
  return a(i) + b(i) * dx + c(i) * dx * dx + d(i) * dx * dx * dx;
}

double CubicSpline::compute_first_derivative_(const double & s) const
{
  if (s < this->start() || s > this->end()) return 0.0;

  auto it = std::lower_bound(this->axis.data(), this->axis.data() + this->axis.size(), s);
  int i = std::max(int(it - this->axis.data()) - 1, 0);

  double dx = s - this->axis(i);
  return b(i) + 2 * c(i) * dx + 3 * d(i) * dx * dx;
}

double CubicSpline::compute_second_derivative_(const double & s) const
{
  if (s < this->start() || s > this->end()) return 0.0;

  auto it = std::lower_bound(this->axis.data(), this->axis.data() + this->axis.size(), s);
  int i = std::max(int(it - this->axis.data()) - 1, 0);

  double dx = s - this->axis(i);
  return 2 * c(i) + 6 * d(i) * dx;
}

}  // namespace autoware::motion_utils::trajectory_container::interpolator
