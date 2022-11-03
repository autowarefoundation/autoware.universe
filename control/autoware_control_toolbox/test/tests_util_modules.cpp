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

#include "act_test_suite.hpp"
#include "gtest/gtest.h"

TEST(ACTutils, linearInterpolateExtrapolate)
{
  // Generate a  sinusoidal signal with arc-length parametrization. This is our test signal.
  size_t const num_of_points = 100;  // number of points in the signal.

  // Generate x.
  std::vector<double> xvec = ns_utils::linspace(0.0, 10.0, num_of_points + 1);

  // Generate y = ax + b.
  double const a = 5.;
  double const b = 1.;

  std::vector<double> yvec;
  std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [&a, &b](auto const & x) {
    return a * x + b;
  });

  std::vector<double> yextrapolation;
  std::vector<double> xnew{-1., 11, 5};
  ns_utils::interp1d_linear(xvec, yvec, xnew, yextrapolation);

  ns_utils::print_container(yextrapolation);
  ASSERT_DOUBLE_EQ(yextrapolation[0], a * xnew[0] + b);
  ASSERT_DOUBLE_EQ(yextrapolation[1], a * xnew[1] + b);
  ASSERT_DOUBLE_EQ(yextrapolation[2], a * xnew[2] + b);
}

TEST(ACTutils, angleDistance)
{
  auto theta_ref = ns_utils::deg2rad(0.);
  auto theta_1 = ns_utils::deg2rad(5.);
  auto theta_2 = ns_utils::deg2rad(-5.);

  auto theta_3 = ns_utils::deg2rad(179.);
  auto theta_4 = ns_utils::deg2rad(181.);
  auto theta_5 = ns_utils::deg2rad(-179.);
  auto theta_6 = ns_utils::deg2rad(365.);

  ns_utils::print(theta_ref, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6);
  ASSERT_TRUE(true);

  std::vector<double> thetas{theta_1, theta_2, theta_3, theta_4, theta_5, theta_6};
  std::vector<double> distances;

  for (auto const & th : thetas) {
    auto dist = ns_utils::angleDistance(th, theta_ref);
    distances.emplace_back(dist);
  }

  ns_utils::print_container(distances);
  ns_utils::print_container(thetas);

  ASSERT_LE(std::fabs(distances[0] - theta_1), 1e-12);
  ASSERT_LE(std::fabs(distances[1] - theta_2), 1e-12);
  ASSERT_LE(std::fabs(distances[2] - theta_3), 1e-12);
  ASSERT_LE(std::fabs(distances[3] + theta_3), 1e-12);  // theta_3 = -theta_4
  ASSERT_LE(std::fabs(distances[4] + theta_3), 1e-12);  // the same.
  ASSERT_LE(std::fabs(distances[5] - theta_1), 1e-12);  // theta_1 == theta_6

  // Test the single arg angle_distance.
  auto theta_6_dist = ns_utils::angleDistance(theta_6);
  ASSERT_LE(std::fabs(theta_6_dist - theta_1), 1e-12);  // theta_1 == theta_6
}

TEST(ACTutils, unwrap)
{
  double Nx = 512;
  std::vector<double> xvec = ns_utils::linspace<double>(0.0, Nx, static_cast<size_t>(Nx));

  // Generate y = 6*sin(2*pi*n/N).
  std::vector<double> yvec;
  std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [&](auto const & x) {
    return 6 * sin(2 * M_PI * x / Nx);
  });

  /**
   * Wrap the signal into [-pi, pi]
   * */
  std::vector<double> xw;
  std::transform(yvec.cbegin(), yvec.cend(), std::back_inserter(xw), [&](auto const & x) {
    return std::atan2(sin(x), cos(x));
  });

  /**
   * unWrap the wrapped signal xw.
   * */
  ns_utils::unWrap(xw);

  for (size_t k = 0; k < xw.size(); ++k) {
    ASSERT_DOUBLE_EQ(yvec[k], xw[k]);
  }
}
