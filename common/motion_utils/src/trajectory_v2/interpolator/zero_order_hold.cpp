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

#include "motion_utils/trajectory_v2/interpolator/zero_order_hold.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <vector>

namespace motion_utils::trajectory_v2::interpolator
{

namespace detail
{

template <typename T>
T ZeroOrderHold_<T>::compute_(const double & s) const
{
  auto it = std::lower_bound(this->axis.data(), this->axis.data() + this->axis.size(), s);
  if (it == this->axis.data()) return this->values[0];
  if (it == this->axis.data() + this->axis.size()) return this->values[this->axis.size() - 1];
  return this->values[it - this->axis.data() - 1];
}

template <typename T>
void ZeroOrderHold_<T>::build_(
  const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<T> & values)
{
  this->axis = axis;
  this->values = values;
}

// Explicit template instantiation
template class ZeroOrderHold_<double>;
template class ZeroOrderHold_<std::vector<int64_t>>;
template class ZeroOrderHold_<std::vector<std::vector<int64_t>>>;

}  // namespace detail

}  // namespace motion_utils::trajectory_v2::interpolator
