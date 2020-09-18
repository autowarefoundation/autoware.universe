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
#include <cmath>
#include <iostream>
#include <vector>

#include "obstacle_avoidance_planner/spline_interpolate.h"

namespace spline
{
/*
 * spline interpolation
 */
SplineInterpolate::SplineInterpolate(){};
SplineInterpolate::SplineInterpolate(const std::vector<double> & x) { generateSpline(x); };
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

  std::vector<double> e_v;
  e_v.push_back(0.0);

  for (int i = 1; i < N - 1; i++) {
    double tmp = 1.0 / (4.0 - e_v[i - 1]);
    c_[i] = (c_[i] - c_[i - 1]) * tmp;
    e_v.push_back(tmp);
  }

  for (int i = N - 2; i > 0; i--) c_[i] = c_[i] - c_[i + 1] * e_v[i];

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
  if (!initialized_) return 0.0;

  int j = std::max(std::min(int(std::floor(s)), (int)a_.size() - 1), 0);
  const double ds = s - j;
  return a_[j] + (b_[j] + (c_[j] + d_[j] * ds) * ds) * ds;
}

void SplineInterpolate::getValueVector(
  const std::vector<double> & s_v, std::vector<double> & value_v)
{
  if (!initialized_) return;
  value_v.clear();
  for (int i = 0; i < (int)s_v.size(); ++i) {
    value_v.push_back(getValue(s_v[i]));
  }
}

bool SplineInterpolate::interpolate(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index, std::vector<double> & return_value)
{
  auto isIncrease = [](const std::vector<double> & x) {
    for (int i = 0; i < (int)x.size() - 1; ++i) {
      if (x[i] > x[i + 1]) return false;
    }
    return true;
  };

  if (base_index.size() == 0 || base_value.size() == 0 || return_index.size() == 0) {
    printf(
      "[interpolate] some vector size is zero: base_index.size() = %lu, base_value.size() = %lu, "
      "return_index.size() = %lu\n",
      base_index.size(), base_value.size(), return_index.size());
    return false;
  }

  // check if inputs are valid
  if (
    !isIncrease(base_index) || !isIncrease(return_index) ||
    return_index.front() < base_index.front() || base_index.back() < return_index.back() ||
    base_index.size() != base_value.size()) {
    std::cerr << "[isIncrease] bad index, return false" << std::endl;
    bool b1 = !isIncrease(base_index);
    bool b2 = !isIncrease(return_index);
    bool b3 = return_index.front() < base_index.front();
    bool b4 = base_index.back() < return_index.back();
    bool b5 = base_index.size() != base_value.size();
    printf("%d, %d, %d, %d, %d\n", b1, b2, b3, b4, b5);
    printf("%f, %f\n", base_index.front(), base_index.back());
    printf("%f, %f\n", return_index.front(), return_index.back());
    printf("%lu, %lu\n", base_index.size(), base_value.size());
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
    if (i <= 0 || (int)base_index.size() - 1 < i) {
      std::cerr << "? something wrong. skip this idx." << std::endl;
      continue;
    }

    const double dist_base_idx = base_index[i] - base_index[i - 1];
    const double dist_to_forward = base_index[i] - idx;
    const double dist_to_backward = idx - base_index[i - 1];
    if (dist_to_forward < 0.0 || dist_to_backward < 0.0) {
      std::cerr << "?? something wrong. skip this idx." << std::endl;
      continue;
    }

    const double value = (dist_to_backward * i + dist_to_forward * (i - 1)) / dist_base_idx;
    normalized_idx.push_back(value);
  }

  // calculate spline coefficients
  generateSpline(base_value);

  // interpolate by spline  with normalized index
  for (int i = 0; i < (int)normalized_idx.size(); ++i) {
    return_value.push_back(getValue(normalized_idx[i]));
  }
  return true;
}

/*
 * calculate distance in x-y 2D space
 */
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

}  // namespace spline
