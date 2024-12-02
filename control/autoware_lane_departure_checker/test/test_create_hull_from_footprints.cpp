// Copyright 2024 TIER IV, Inc.
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

#include "autoware/lane_departure_checker/utils.hpp"

#include <Eigen/Core>

#include <gtest/gtest.h>

using autoware::universe_utils::LinearRing2d;

struct CreateHullFromFootprintsTestParam
{
  std::string description;
  std::vector<LinearRing2d> footprints;
  LinearRing2d expected_hull;
};

std::ostream & operator<<(std::ostream & os, const CreateHullFromFootprintsTestParam & p)
{
  return os << p.description;
}

class CreateHullFromFootprintsTest
: public ::testing::TestWithParam<CreateHullFromFootprintsTestParam>
{
};

TEST_P(CreateHullFromFootprintsTest, test_create_hull_from_footprints)
{
  const auto p = GetParam();
  const auto hull = autoware::lane_departure_checker::utils::createHullFromFootprints(p.footprints);

  ASSERT_EQ(hull.size(), p.expected_hull.size());

  for (size_t i = 0; i < p.expected_hull.size(); ++i) {
    EXPECT_DOUBLE_EQ(hull[i].x(), p.expected_hull[i].x());
    EXPECT_DOUBLE_EQ(hull[i].y(), p.expected_hull[i].y());
  }
}

INSTANTIATE_TEST_SUITE_P(
  LaneDepartureCheckerTest, CreateHullFromFootprintsTest,
  ::testing::Values(
    CreateHullFromFootprintsTestParam{"EmptyFootprints", {}, {}},
    CreateHullFromFootprintsTestParam{
      "SingleFootprint",
      {{{0.0, 0.0}, {0.0, 1.0}, {2.0, 1.0}, {2.0, 0.0}, {0.0, 0.0}}},
      {{0.0, 0.0}, {0.0, 1.0}, {2.0, 1.0}, {2.0, 0.0}, {0.0, 0.0}}}),
  ::testing::PrintToStringParamName());
