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

#ifndef INTERPOLATION__INTERPOLATION_UTILS_HPP_
#define INTERPOLATION__INTERPOLATION_UTILS_HPP_

#include <array>
#include <stdexcept>
#include <vector>

namespace interpolation_utils
{
inline bool isIncreasing(const std::vector<double> & x)
{
  if (x.empty()) {
    throw std::invalid_argument("Points is empty.");
  }

  for (size_t i = 0; i < x.size() - 1; ++i) {
    if (x.at(i) >= x.at(i + 1)) {
      return false;
    }
  }

  return true;
}

inline bool isNotDecreasing(const std::vector<double> & x)
{
  if (x.empty()) {
    throw std::invalid_argument("Points is empty.");
  }

  for (size_t i = 0; i < x.size() - 1; ++i) {
    if (x.at(i) > x.at(i + 1)) {
      return false;
    }
  }

  return true;
}

inline void validateKeys(
  const std::vector<double> & base_keys, const std::vector<double> & query_keys)
{
  // when vectors are empty
  if (base_keys.empty() || query_keys.empty()) {
    throw std::invalid_argument("Points is empty.");
  }

  // when size of vectors are less than 2
  if (base_keys.size() < 2) {
    throw std::invalid_argument(
      "The size of points is less than 2. base_keys.size() = " + std::to_string(base_keys.size()));
  }

  // when indices are not sorted
  if (!isIncreasing(base_keys) || !isNotDecreasing(query_keys)) {
    throw std::invalid_argument("Either base_keys or query_keys is not sorted.");
  }

  // when query_keys is out of base_keys (This function does not allow exterior division.)
  if (query_keys.front() < base_keys.front() || base_keys.back() < query_keys.back()) {
    throw std::invalid_argument("query_keys is out of base_keys");
  }
}

inline void validateKeysAndValues(
  const std::vector<double> & base_keys, const std::vector<double> & base_values)
{
  // when vectors are empty
  if (base_keys.empty() || base_values.empty()) {
    throw std::invalid_argument("Points is empty.");
  }

  // when size of vectors are less than 2
  if (base_keys.size() < 2 || base_values.size() < 2) {
    throw std::invalid_argument(
      "The size of points is less than 2. base_keys.size() = " + std::to_string(base_keys.size()) +
      ", base_values.size() = " + std::to_string(base_values.size()));
  }

  // when sizes of indices and values are not same
  if (base_keys.size() != base_values.size()) {
    throw std::invalid_argument("The size of base_keys and base_values are not the same.");
  }
}

// solve Ax = d
// where A is tridiagonal matrix
//     [b_0 c_0 ...                       ]
//     [a_0 b_1 c_1 ...               O   ]
// A = [            ...                   ]
//     [   O         ... a_N-3 b_N-2 c_N-2]
//     [                   ... a_N-2 b_N-1]
struct TDMACoef
{
  explicit TDMACoef(const size_t num_row)
  {
    a.resize(num_row - 1);
    b.resize(num_row);
    c.resize(num_row - 1);
    d.resize(num_row);
  }

  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  std::vector<double> d;
};

inline std::vector<double> solveTridiagonalMatrixAlgorithm(const TDMACoef & tdma_coef)
{
  const auto & a = tdma_coef.a;
  const auto & b = tdma_coef.b;
  const auto & c = tdma_coef.c;
  const auto & d = tdma_coef.d;

  const size_t num_row = b.size();

  std::vector<double> x(num_row);
  if (num_row != 1) {
    // calculate p and q
    std::vector<double> p;
    std::vector<double> q;
    p.push_back(-c[0] / b[0]);
    q.push_back(d[0] / b[0]);

    for (size_t i = 1; i < num_row; ++i) {
      const double den = b[i] + a[i - 1] * p[i - 1];
      p.push_back(-c[i - 1] / den);
      q.push_back((d[i] - a[i - 1] * q[i - 1]) / den);
    }

    // calculate solution
    x[num_row - 1] = q[num_row - 1];

    for (size_t i = 1; i < num_row; ++i) {
      const size_t j = num_row - 1 - i;
      x[j] = p[j] * x[j + 1] + q[j];
    }
  } else {
    x.push_back(d[0] / b[0]);
  }

  return x;
}
}  // namespace interpolation_utils

#endif  // INTERPOLATION__INTERPOLATION_UTILS_HPP_
