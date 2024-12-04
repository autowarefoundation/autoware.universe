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

#include <autoware/behavior_velocity_blind_spot_module/util.hpp>
#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

using autoware::behavior_velocity_planner::PlannerData;

class TestUtilWithMap : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string intersection_map_file =
      autoware::test_utils::get_absolute_path_to_lanelet_map(
        "autoware_test_utils", "test_map/intersection/lanelet2_map.osm");
    return;
  }

private:
  std::shared_ptr<PlannerData> planner_data;
};
