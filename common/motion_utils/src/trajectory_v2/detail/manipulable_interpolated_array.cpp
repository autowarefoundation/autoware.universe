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

#include "motion_utils/trajectory_v2/detail/manipulable_interpolated_array.hpp"

#include <algorithm>
#include <stdexcept>

namespace motion_utils::trajectory_v2::detail
{

template <typename T>
RangeSetter<T> ManipulableInterpolatedArray<T>::operator()(const double & start, const double & end)
{
  return RangeSetter<T>{*this, start, end};
}

template <typename T>
ManipulableInterpolatedArray<T> & ManipulableInterpolatedArray<T>::setInterpolator(
  const interpolator::Interpolator<T> & interpolator)
{
  interpolator_ = std::shared_ptr<interpolator::Interpolator<T>>(interpolator.clone());
  return *this;
}

template <typename T>
ManipulableInterpolatedArray<T> & ManipulableInterpolatedArray<T>::build(
  const std::vector<double> & axis, const std::vector<T> & values)
{
  if (axis.size() != values.size()) {
    throw std::invalid_argument("Axis and values must have the same size.");
  }
  if (axis.empty()) {
    throw std::invalid_argument("Axis and values cannot be empty.");
  }
  axis_ = axis;
  values_ = values;
  interpolator_->build(axis, values);
  is_built_ = true;
  return *this;
}

template <typename T>
ManipulableInterpolatedArray<T> & ManipulableInterpolatedArray<T>::build(
  const Eigen::VectorXd & axis, const std::vector<T> & values)
{
  return build(std::vector<double>(axis.data(), axis.data() + axis.size()), values);
}

template <typename T>
ManipulableInterpolatedArray<T> & ManipulableInterpolatedArray<T>::operator=(const T & value)
{
  if (values_.empty()) {
    throw std::runtime_error("Values array is empty.");
  }
  std::fill(values_.begin(), values_.end(), value);
  interpolator_ = std::shared_ptr<interpolator::Interpolator<T>>(interpolator_->clone());
  interpolator_->build(axis_, values_);
  return *this;
}

template <typename T>
T ManipulableInterpolatedArray<T>::compute(const double & x) const
{
  if (!interpolator_) {
    throw std::runtime_error("Interpolator is not set.");
  }
  if (!is_built_) {
    throw std::runtime_error("Interpolator has not been built yet.");
  }
  return interpolator_->compute(x);
}

template <typename T>
RangeSetter<T>::RangeSetter(ManipulableInterpolatedArray<T> & parent, double start, double end)
: parent_(parent), start_(start), end_(end)
{
  if (start >= end) {
    throw std::invalid_argument("Start value must be less than end value.");
  }
}

template <typename T>
RangeSetter<T> & RangeSetter<T>::operator=(const T & value)
{
  auto & axis = parent_.axis_;
  auto & values = parent_.values_;

  // Insert start if not present
  auto start_it = std::lower_bound(axis.begin(), axis.end(), start_);
  if (start_it == axis.end() || *start_it != start_) {
    start_it = axis.insert(start_it, start_);
    values.insert(values.begin() + (start_it - axis.begin()), value);
  }

  // Insert end if not present
  auto end_it = std::lower_bound(axis.begin(), axis.end(), end_);
  if (end_it == axis.end() || *end_it != end_) {
    end_it = axis.insert(end_it, end_);
    values.insert(values.begin() + (end_it - axis.begin()), value);
  }

  // Set value in the specified range
  for (auto it = start_it; it != axis.end() && *it <= end_; ++it) {
    values[it - axis.begin()] = value;
  }

  // Clone the interpolator with updated values
  parent_.interpolator_ =
    std::shared_ptr<interpolator::Interpolator<T>>(parent_.interpolator_->clone());
  parent_.interpolator_->build(axis, values);

  return *this;
}

template class RangeSetter<double>;
template class RangeSetter<std::vector<int64_t>>;
template class RangeSetter<std::vector<std::vector<int64_t>>>;

template class ManipulableInterpolatedArray<double>;
template class ManipulableInterpolatedArray<std::vector<int64_t>>;
template class ManipulableInterpolatedArray<std::vector<std::vector<int64_t>>>;

}  // namespace motion_utils::trajectory_v2::detail
