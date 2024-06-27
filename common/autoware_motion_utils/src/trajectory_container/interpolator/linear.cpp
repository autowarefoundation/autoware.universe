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

#include "autoware/motion_utils/trajectory_container/interpolator/linear.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <vector>

namespace autoware::motion_utils::trajectory_container::interpolator
{

void Linear::build_(
  const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<double> & values)
{
  this->axis = axis;
  this->values = Eigen::Map<const Eigen::VectorXd>(values.data(), values.size());
}

double Linear::compute_(const double & s) const
{
  auto it = std::lower_bound(this->axis.data(), this->axis.data() + this->axis.size(), s);
  if (it == this->axis.data()) return this->values(0);
  if (it == this->axis.data() + this->axis.size()) return this->values(this->axis.size() - 1);
  int idx = it - this->axis.data() - 1;
  double x0 = this->axis(idx), x1 = this->axis(idx + 1);
  double y0 = this->values(idx), y1 = this->values(idx + 1);
  return y0 + (y1 - y0) * (s - x0) / (x1 - x0);
}

double Linear::compute_first_derivative_(const double & s) const
{
  auto it = std::lower_bound(this->axis.data(), this->axis.data() + this->axis.size(), s);
  if (it == this->axis.data() || it == this->axis.data() + this->axis.size()) return 0.0;
  int idx = it - this->axis.data() - 1;
  double x0 = this->axis(idx), x1 = this->axis(idx + 1);
  double y0 = this->values(idx), y1 = this->values(idx + 1);
  return (y1 - y0) / (x1 - x0);
}

double Linear::compute_second_derivative_(const double &) const
{
  return 0.0;
}

size_t Linear::minimum_required_points() const
{
  return 2;
}

}  // namespace autoware::motion_utils::trajectory_container::interpolator
