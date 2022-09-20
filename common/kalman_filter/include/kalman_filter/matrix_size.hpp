// Copyright 2022 Autoware Foundation
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

#ifndef KALMAN_FILTER__MATRIX_SIZE_HPP_
#define KALMAN_FILTER__MATRIX_SIZE_HPP_

#include "kalman_filter/kalman_filter.hpp"

inline bool hasZeroElements(const Eigen::MatrixXd & M)
{
  return M.size() == 0;
}

#endif  // KALMAN_FILTER__MATRIX_SIZE_HPP_
