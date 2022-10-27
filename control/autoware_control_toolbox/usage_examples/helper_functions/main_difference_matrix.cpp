// Copyright 2022 The Autoware Foundation.
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

#include "autoware_control_toolbox.hpp"

#include <fmt/core.h>

int main()
{
  size_t n = 10;
  size_t k = 5;
  size_t d = 2;

  auto D = ns_eigen_utils::difference_matrix<double>(n, d);
  ns_eigen_utils::printEigenMat(D, "Ordinary difference matrix");

  // Test B-spline difference matrix
  auto Db = ns_eigen_utils::difference_matrix_bspline<double>(n, k, d);
  ns_eigen_utils::printEigenMat(Db, "Bspline difference matrix : ");

  return 0;
}
