/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "utilization/interpolate.h"
#include "utilization/util.h"

namespace interpolation
{
/*
 * linear interpolation
 */
bool LinearInterpolate::interpolate(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index, std::vector<double> & return_value)
{
  if (!isValidInput(base_index, base_value, return_index, return_value)) {
    std::cerr << "[interpolate] invalid input. interpolation failed." << std::endl;
    return false;
  }

  // calculate linear interpolation
  int i = 0;
  for (const auto idx : return_index) {
    if (base_index[i] == idx) {
      return_value.push_back(base_value[i]);
      continue;
    }
    while (base_index[i] < idx) ++i;
    if (i <= 0 || static_cast<int>(base_index.size()) - 1 < i) {
      std::cerr << "[interpolate] undesired condition. skip this idx!" << std::endl;
      continue;
    }

    const double base_dist = base_index[i] - base_index[i - 1];
    const double to_forward = base_index[i] - idx;
    const double to_backward = idx - base_index[i - 1];
    if (to_forward < 0.0 || to_backward < 0.0) {
      std::cerr << "[interpolate] undesired condition. skip this idx!!" << std::endl;
      std::cerr << "i = " << i << ", base_index[i - 1] = " << base_index[i - 1] << ", idx = " << idx
                << ", base_index[i] = " << base_index[i] << std::endl;
      continue;
    }

    const double value = (to_backward * base_value[i] + to_forward * base_value[i - 1]) / base_dist;
    return_value.push_back(value);
  }
  return true;
}

/*
 * spline interpolation
 */
SplineInterpolate::SplineInterpolate() {}
SplineInterpolate::SplineInterpolate(const std::vector<double> & x) { generateSpline(x); }
void SplineInterpolate::generateSpline(const std::vector<double> & x)
{
  int N = x.size();

  a_.clear();
  b_.clear();
  c_.clear();
  d_.clear();

  a_ = x;

  c_.push_back(0.0);
  for (int i = 1; i < N - 1; i++) c_.push_back(3.0 * (a_[i - 1] - 2.0 * a_[i] + a_[i + 1]));
  c_.push_back(0.0);

  std::vector<double> w_;
  w_.push_back(0.0);

  double tmp;
  for (int i = 1; i < N - 1; i++) {
    tmp = 1.0 / (4.0 - w_[i - 1]);
    c_[i] = (c_[i] - c_[i - 1]) * tmp;
    w_.push_back(tmp);
  }

  for (int i = N - 2; i > 0; i--) c_[i] = c_[i] - c_[i + 1] * w_[i];

  for (int i = 0; i < N - 1; i++) {
    d_.push_back((c_[i + 1] - c_[i]) / 3.0);
    b_.push_back(a_[i + 1] - a_[i] - c_[i] - d_[i]);
  }
  d_.push_back(0.0);
  b_.push_back(0.0);

  initialized_ = true;
};

double SplineInterpolate::getValue(const double & s)
{
  if (!initialized_) {
    std::cerr << "[interpolate] spline is uninitialized" << std::endl;
    return 0.0;
  }

  int j = std::max(std::min<int>(std::floor(s), static_cast<int>(a_.size()) - 1), 0);
  const double ds = s - j;
  return a_[j] + (b_[j] + (c_[j] + d_[j] * ds) * ds) * ds;
}

std::vector<double> SplineInterpolate::getValueVector(const std::vector<double> & s_v)
{
  if (!initialized_) {
    std::cerr << "[interpolate] spline is uninitialized" << std::endl;
    return {};
  }
  std::vector<double> value_v;
  for (int i = 0; i < static_cast<int>(s_v.size()); ++i) {
    value_v.push_back(getValue(s_v[i]));
  }
  return value_v;
}

bool SplineInterpolate::interpolate(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index, std::vector<double> & return_value)
{
  if (!isValidInput(base_index, base_value, return_index, return_value)) {
    std::cerr << "[interpolate] invalid input. interpolation failed." << std::endl;
    return false;
  }

  std::vector<double> normalized_idx;

  // calculate normalized index
  int i = 0;
  for (const auto idx : return_index) {
    if (base_index[i] == idx) {
      normalized_idx.push_back(i);
      continue;
    }
    while (base_index[i] < idx) ++i;
    if (i <= 0 || static_cast<int>(base_index.size()) - 1 < i) {
      std::cerr << "[interpolate] undesired condition. skip this idx!" << std::endl;
      continue;
    }

    const double base_dist = base_index[i] - base_index[i - 1];
    const double to_forward = base_index[i] - idx;
    const double to_backward = idx - base_index[i - 1];
    if (to_forward <= 0.0 || to_backward <= 0.0) {
      std::cerr << "[interpolate] undesired condition. skip this idx!!" << std::endl;
      std::cerr << "i = " << i << ", base_index[i - 1] = " << base_index[i - 1] << ", idx = " << idx
                << ", base_index[i] = " << base_index[i] << std::endl;
      continue;
    }

    const double value = (to_backward * i + to_forward * (i - 1)) / base_dist;
    normalized_idx.push_back(value);
  }

  // calculate spline coefficients
  generateSpline(base_value);

  // interpolate by spline with normalized index
  for (size_t i = 0; i < normalized_idx.size(); ++i) {
    return_value.push_back(getValue(normalized_idx[i]));
  }
  return true;
}

/*
 * helper functions
 */
bool isIncrease(const std::vector<double> & x)
{
  for (int i = 0; i < static_cast<int>(x.size()) - 1; ++i) {
    if (x[i] > x[i + 1]) return false;
  }
  return true;
};

bool isValidInput(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index, std::vector<double> & return_value)
{
  if (base_index.empty() || base_value.empty() || return_index.empty()) {
    std::cout << "bad index : some vector is empty. base_index: " << base_index.size()
              << ", base_value: " << base_value.size() << ", return_index: " << return_index.size()
              << std::endl;
    return false;
  }
  if (!isIncrease(base_index)) {
    std::cout << "bad index : base_index is not monotonically increasing. base_index = ["
              << base_index.front() << ", " << base_index.back() << "]" << std::endl;
    return false;
  }
  if (!isIncrease(return_index)) {
    std::cout << "bad index : base_index is not monotonically increasing. return_index = ["
              << return_index.front() << ", " << return_index.back() << "]" << std::endl;
    return false;
  }
  if (return_index.front() < base_index.front()) {
    std::cout << "bad index : return_index.front() < base_index.front()" << std::endl;
    return false;
  }
  if (base_index.back() < return_index.back()) {
    std::cout << "bad index : base_index.back() < return_index.back()" << std::endl;
    return false;
  }
  if (base_index.size() != base_value.size()) {
    std::cout << "bad index : base_index.size() != base_value.size()" << std::endl;
    return false;
  }

  return true;
}

std::vector<double> calcEuclidDist(const std::vector<double> & x, const std::vector<double> & y)
{
  if (x.size() != y.size()) {
    std::cerr << "x y vector size should be the same." << std::endl;
  }

  std::vector<double> dist_v;
  dist_v.push_back(0.0);
  for (unsigned int i = 0; i < x.size() - 1; ++i) {
    const double dx = x.at(i + 1) - x.at(i);
    const double dy = y.at(i + 1) - y.at(i);
    dist_v.push_back(dist_v.at(i) + std::hypot(dx, dy));
  }

  return dist_v;
}
}  // namespace interpolation
