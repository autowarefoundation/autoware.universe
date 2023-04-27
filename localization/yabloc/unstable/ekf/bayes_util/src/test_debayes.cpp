// Copyright 2023 TIER IV, Inc.
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

#include "bayes_util/bayes_util.hpp"

#include <gtest/gtest.h>

void test(double a, double b, double c, double d)
{
  Eigen::Matrix2d S;
  S << a, b, c, d;

  Eigen::Matrix2d approx = yabloc::bayes_util::approximate_by_spd(S, true);
  std::cout << "target:\n " << S << std::endl;
  std::cout << "opt:\n " << approx << std::endl;
  std::cout << std::endl;
}

TEST(BayesUtilTestSuite, debayes)
{
  test(2, 1, 1, 1);

  test(2, 2, 2, 1);

  test(4.1, 2, 2, 1);
}
