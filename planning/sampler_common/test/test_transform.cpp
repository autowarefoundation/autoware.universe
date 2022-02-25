// Copyright 2022 Tier IV, Inc.
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

#include <sampler_common/transform/spline_transform.hpp>

#include <gtest/gtest.h>

constexpr auto TOL = 1E-6;  // 1µm tolerance

TEST(splineTransform, makeSpline2D)
{
  sampler_common::transform::Spline2D spline1({0.0, 1.0, 2.0}, {0.0, 1.0, 2.0});
  sampler_common::transform::Spline2D spline2({-10.0, 5.0, -2.0}, {99.0, 3.0, -20.0});
}

TEST(splineTransform, toFrenet)
{
  sampler_common::transform::Spline2D spline({0.0, 1.0, 2.0}, {0.0, 0.0, 0.0});
  auto frenet = spline.frenet({0.0, 0.0});
  EXPECT_NEAR(frenet.s, 0.0, TOL);
  EXPECT_NEAR(frenet.d, 0.0, TOL);
  frenet = spline.frenet({1.0, 0.0});
  EXPECT_NEAR(frenet.s, 1.0, TOL);
  EXPECT_NEAR(frenet.d, 0.0, TOL);
  frenet = spline.frenet({2.0, 0.0});
  EXPECT_NEAR(frenet.s, 2.0, TOL);
  EXPECT_NEAR(frenet.d, 0.0, TOL);
  frenet = spline.frenet({1.0, 1.0});
  EXPECT_NEAR(frenet.s, 1.0, TOL);
  EXPECT_NEAR(frenet.d, 1.0, TOL);
  frenet = spline.frenet({1.5, -2.0});
  EXPECT_NEAR(frenet.s, 1.5, TOL);
  EXPECT_NEAR(frenet.d, -2.0, TOL);
  /* TODO(Maxime CLEMENT) failing edge cases
  // EDGE CASE before spline
  frenet = spline.frenet({-1.0, 1.0});
  EXPECT_NEAR(frenet.s, -1.0, TOL);
  EXPECT_NEAR(frenet.d, 1.0, TOL);
  // EDGE CASE after spline
  frenet = spline.frenet({3.0, -1.0});
  EXPECT_NEAR(frenet.s, 1.0, TOL);
  EXPECT_NEAR(frenet.d, -1.0, TOL);
  // EDGE CASE 90 deg angle
  sampler_common::transform::Spline2D spline2({0.0, 1.0, 2.0}, {0.0, 1.0, 0.0});
  frenet = spline2.frenet({1.0, 2.0});
  EXPECT_NEAR(frenet.s, 1.0, TOL);
  EXPECT_NEAR(frenet.d, 1.0, TOL);
  */
}

TEST(splineTransform, toCartesian)
{
  sampler_common::transform::Spline2D spline({0.0, 1.0, 2.0}, {0.0, 0.0, 0.0});
  auto cart = spline.cartesian({1.0, 0.0});
  EXPECT_NEAR(cart.x(), 1.0, TOL);
  EXPECT_NEAR(cart.y(), 0.0, TOL);
}
