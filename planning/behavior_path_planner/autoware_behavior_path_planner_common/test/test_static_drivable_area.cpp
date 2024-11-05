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

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

// constexpr auto eps = 1e-9;

using autoware::behavior_path_planner::DrivableLanes;

lanelet::ConstLanelet make_lanelet(
  const lanelet::BasicPoint2d & left0, const lanelet::BasicPoint2d & left1,
  const lanelet::BasicPoint2d & right0, const lanelet::BasicPoint2d & right1)
{
  lanelet::LineString3d left_bound;
  left_bound.push_back(lanelet::Point3d(lanelet::InvalId, left0.x(), left0.y(), 0.0));
  left_bound.push_back(lanelet::Point3d(lanelet::InvalId, left1.x(), left1.y(), 0.0));
  lanelet::LineString3d right_bound;
  right_bound.push_back(lanelet::Point3d(lanelet::InvalId, right0.x(), right0.y(), 0.0));
  right_bound.push_back(lanelet::Point3d(lanelet::InvalId, right1.x(), right1.y(), 0.0));
  return {lanelet::utils::getId(), left_bound, right_bound};
}

DrivableLanes make_drivable_lanes(const lanelet::ConstLanelet & ll)
{
  DrivableLanes l;
  l.left_lane = ll;
  l.right_lane = ll;
  l.middle_lanes = {ll};
  return l;
}

TEST(StaticDrivableArea, getOverlappedLaneletId)
{
  using autoware::behavior_path_planner::utils::getOverlappedLaneletId;

  std::vector<DrivableLanes> lanes;
  {  // empty lanes
    const auto result = getOverlappedLaneletId(lanes);
    EXPECT_FALSE(result.has_value());
  }
  {  // lanes at 0
    const DrivableLanes l =
      make_drivable_lanes(make_lanelet({0.0, 1.0}, {5.0, 1.0}, {5.0, -1.0}, {5.0, -1.0}));
    lanes.push_back(l);
    const auto result = getOverlappedLaneletId(lanes);
    EXPECT_FALSE(result.has_value());
  }
  {  // lanes at 1, overlap with 0 but ignored since it is the following lane
    const DrivableLanes l =
      make_drivable_lanes(make_lanelet({4.0, 1.0}, {8.0, 1.0}, {4.0, -1.0}, {8.0, -1.0}));
    lanes.push_back(l);
    const auto result = getOverlappedLaneletId(lanes);
    EXPECT_FALSE(result.has_value());
  }
  {  // lanes at 2, overlap with 1 but ignored since it is the following lane
    const DrivableLanes l =
      make_drivable_lanes(make_lanelet({6.0, 1.0}, {10.0, 1.0}, {6.0, -1.0}, {10.0, -1.0}));
    lanes.push_back(l);
    const auto result = getOverlappedLaneletId(lanes);
    EXPECT_FALSE(result.has_value());
  }
  {  // lanes at 3, overlap with 1 so 3 is returned
    const DrivableLanes l =
      make_drivable_lanes(make_lanelet({5.0, 0.0}, {5.0, 5.0}, {6.0, 0.0}, {6.0, 5.0}));
    lanes.push_back(l);
    const auto result = getOverlappedLaneletId(lanes);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 3UL);
  }
  {  // lanes at 4, overlap with 2 but since 3 overlaps first it is still returned
    const DrivableLanes l =
      make_drivable_lanes(make_lanelet({7.0, 0.0}, {7.0, 5.0}, {8.0, 0.0}, {8.0, 5.0}));
    lanes.push_back(l);
    const auto result = getOverlappedLaneletId(lanes);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 3UL);
  }
  {  // change 1 to no longer overlap with 3 and now 4 is the first overlap
    const DrivableLanes l = make_drivable_lanes(
      make_lanelet({100.0, 110.0}, {110.0, 100.0}, {100.0, 90.0}, {100.0, 90.0}));
    lanes[1] = l;
    const auto result = getOverlappedLaneletId(lanes);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 4UL);
  }
}
