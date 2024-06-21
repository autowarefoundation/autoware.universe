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

#include "motion_utils/trajectory_v2/interpolator/akima_spline.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <vector>

namespace motion_utils::trajectory_v2::interpolator
{

void AkimaSpline::compute_parameters(
  const Eigen::Ref<const Eigen::VectorXd> & axis, const Eigen::Ref<const Eigen::VectorXd> & values)
{
  int n = axis.size();

  Eigen::VectorXd h = axis.tail(n - 1) - axis.head(n - 1);

  Eigen::VectorXd m(n - 1);
  for (int i = 0; i < n - 1; ++i) {
    m[i] = (values[i + 1] - values[i]) / h[i];
  }

  Eigen::VectorXd s(n);
  s[0] = m[0];
  s[1] = (m[0] + m[1]) / 2;
  for (int i = 2; i < n - 2; ++i) {
    if ((std::abs(m[i + 1] - m[i]) + std::abs(m[i - 1] - m[i - 2])) == 0) {
      s[i] = (m[i] + m[i - 1]) / 2;
    } else {
      s[i] = (std::abs(m[i + 1] - m[i]) * m[i - 1] + std::abs(m[i - 1] - m[i - 2]) * m[i]) /
             (std::abs(m[i + 1] - m[i]) + std::abs(m[i - 1] - m[i - 2]));
    }
  }
  s[n - 2] = (m[n - 2] + m[n - 3]) / 2;
  s[n - 1] = m[n - 2];

  a.resize(n - 1);
  b.resize(n - 1);
  c.resize(n - 1);
  d.resize(n - 1);
  for (int i = 0; i < n - 1; ++i) {
    a[i] = values[i];
    b[i] = s[i];
    c[i] = (3 * m[i] - 2 * s[i] - s[i + 1]) / h[i];
    d[i] = (s[i] + s[i + 1] - 2 * m[i]) / (h[i] * h[i]);
  }
}

void AkimaSpline::build_(
  const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<double> & values)
{
  this->axis = axis;
  compute_parameters(axis, Eigen::Map<const Eigen::VectorXd>(values.data(), values.size()));
}

double AkimaSpline::compute_(const double & s) const
{
  if (s < this->start()) return a[0];
  if (s > this->end()) return a[a.size() - 1];

  int i = std::lower_bound(this->axis.data(), this->axis.data() + this->axis.size(), s) -
          this->axis.data() - 1;
  if (i < 0) i = 0;

  double dx = s - this->axis[i];
  return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
}

double AkimaSpline::compute_first_derivative_(const double & s) const
{
  if (s < this->start() || s > this->end()) return 0.0;

  int i = std::lower_bound(this->axis.data(), this->axis.data() + this->axis.size(), s) -
          this->axis.data() - 1;
  if (i < 0) i = 0;

  double dx = s - this->axis[i];
  return b[i] + 2 * c[i] * dx + 3 * d[i] * dx * dx;
}

double AkimaSpline::compute_second_derivative_(const double & s) const
{
  if (s < this->start() || s > this->end()) return 0.0;

  int i = std::lower_bound(this->axis.data(), this->axis.data() + this->axis.size(), s) -
          this->axis.data() - 1;
  if (i < 0) i = 0;

  double dx = s - this->axis[i];
  return 2 * c[i] + 6 * d[i] * dx;
}

}  // namespace motion_utils::trajectory_v2::interpolator
