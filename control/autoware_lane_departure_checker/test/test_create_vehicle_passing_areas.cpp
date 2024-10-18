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

struct CreateVehiclePassingAreasTestParam
{
  std::string description;
  std::vector<LinearRing2d> vehicle_footprints;
  std::vector<LinearRing2d> expected_areas;
};

std::ostream & operator<<(std::ostream & os, const CreateVehiclePassingAreasTestParam & p)
{
  return os << p.description;
}

class CreateVehiclePassingAreasTest
: public ::testing::TestWithParam<CreateVehiclePassingAreasTestParam>
{
};

TEST_P(CreateVehiclePassingAreasTest, test_create_vehicle_passing_areas)
{
  const auto p = GetParam();
  const auto areas =
    autoware::lane_departure_checker::utils::createVehiclePassingAreas(p.vehicle_footprints);

  ASSERT_EQ(areas.size(), p.expected_areas.size());

  for (size_t i = 0; i < p.expected_areas.size(); ++i) {
    ASSERT_EQ(areas[i].size(), p.expected_areas[i].size());
    for (size_t j = 0; j < p.expected_areas[i].size(); ++j) {
      EXPECT_DOUBLE_EQ(areas[i][j].x(), p.expected_areas[i][j].x());
      EXPECT_DOUBLE_EQ(areas[i][j].y(), p.expected_areas[i][j].y());
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  LaneDepartureCheckerTest, CreateVehiclePassingAreasTest,
  ::testing::Values(
    CreateVehiclePassingAreasTestParam{"EmptyFootprints", {}, {}},
    CreateVehiclePassingAreasTestParam{
      "SingleFootprint", {{{0.0, 0.0}, {0.0, 1.0}, {2.0, 1.0}, {2.0, 0.0}, {0.0, 0.0}}}, {}},
    CreateVehiclePassingAreasTestParam{
      "TwoFootprints",
      {{{0.0, 0.0}, {0.0, 1.0}, {2.0, 1.0}, {2.0, 0.0}, {0.0, 0.0}},
       {{0.5, 0.0}, {0.5, 1.0}, {2.5, 1.0}, {2.5, 0.0}, {0.5, 0.0}}},
      {{{0.0, 0.0}, {0.0, 1.0}, {2.5, 1.0}, {2.5, 0.0}, {0.0, 0.0}}}}),
  ::testing::PrintToStringParamName());
