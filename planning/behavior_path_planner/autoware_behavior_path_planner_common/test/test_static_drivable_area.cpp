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

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

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

bool equal(const DrivableLanes & l1, const DrivableLanes & l2)
{
  if (l1.middle_lanes.size() != l2.middle_lanes.size()) {
    return false;
  }
  auto are_equal = true;
  are_equal &= boost::geometry::equals(
    l1.left_lane.polygon2d().basicPolygon(), l2.left_lane.polygon2d().basicPolygon());
  are_equal &= boost::geometry::equals(
    l1.right_lane.polygon2d().basicPolygon(), l2.right_lane.polygon2d().basicPolygon());
  for (auto i = 0UL; i < l1.middle_lanes.size(); ++i) {
    are_equal &= boost::geometry::equals(
      l1.middle_lanes[i].polygon2d().basicPolygon(), l2.middle_lanes[i].polygon2d().basicPolygon());
  }
  return are_equal;
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

TEST(StaticDrivableArea, cutOverlappedLanes)
{
  using autoware::behavior_path_planner::utils::cutOverlappedLanes;
  tier4_planning_msgs::msg::PathWithLaneId path;
  std::vector<DrivableLanes> lanes;
  {  // empty inputs
    const auto result = cutOverlappedLanes(path, lanes);
    EXPECT_TRUE(result.empty());
    EXPECT_TRUE(path.points.empty());
  }
  constexpr auto path_size = 10UL;
  const auto reset_path = [&]() {
    path.points.clear();
    for (auto i = 0UL; i < path_size; ++i) {
      path.points.emplace_back();
      path.points.back().point.pose.position.x = static_cast<double>(i) * 1.0;
      path.points.back().point.pose.position.y = 0.0;
    }
  };
  {  // add some path points
    reset_path();
    const auto result = cutOverlappedLanes(path, lanes);
    EXPECT_TRUE(result.empty());
    ASSERT_EQ(path.points.size(), path_size);
    for (auto i = 0UL; i < path_size; ++i) {
      EXPECT_EQ(path.points[i].point.pose.position.x, i * 1.0);
      EXPECT_EQ(path.points[i].point.pose.position.y, 0.0);
    }
  }
  {  // add some drivable lanes without any overlap (no overlap -> path is not modified)
    reset_path();
    lanes.push_back(
      make_drivable_lanes(make_lanelet({0.0, 1.0}, {2.0, 1.0}, {0.0, -1.0}, {2.0, -1.0})));
    const auto result = cutOverlappedLanes(path, lanes);
    ASSERT_EQ(result.size(), lanes.size());
    EXPECT_TRUE(equal(result.front(), lanes.front()));
    ASSERT_EQ(path.points.size(), path_size);
    for (auto i = 0UL; i < path_size; ++i) {
      EXPECT_EQ(path.points[i].point.pose.position.x, i * 1.0);
      EXPECT_EQ(path.points[i].point.pose.position.y, 0.0);
    }
  }
  {  // add more drivable lanes without an overlap (no overlap -> path is not modified)
    reset_path();
    lanes.push_back(
      make_drivable_lanes(make_lanelet({2.0, 1.0}, {4.0, 1.0}, {2.0, -1.0}, {4.0, -1.0})));
    lanes.push_back(
      make_drivable_lanes(make_lanelet({4.0, 1.0}, {6.0, 1.0}, {4.0, -1.0}, {6.0, -1.0})));
    const auto result = cutOverlappedLanes(path, lanes);
    ASSERT_EQ(result.size(), lanes.size());
    for (auto i = 0UL; i < result.size(); ++i) {
      EXPECT_TRUE(equal(result[i], lanes[i]));
    }
    ASSERT_EQ(path.points.size(), path_size);
    for (auto i = 0UL; i < path_size; ++i) {
      EXPECT_EQ(path.points[i].point.pose.position.x, i * 1.0);
      EXPECT_EQ(path.points[i].point.pose.position.y, 0.0);
    }
  }
  {  // add an overlapping lane
    reset_path();
    lanes.push_back(
      make_drivable_lanes(make_lanelet({2.5, -1.0}, {2.5, 1.0}, {3.5, -1.0}, {3.5, 1.0})));
    const auto result = cutOverlappedLanes(path, lanes);
    // the last lane is cut
    ASSERT_EQ(result.size() + 1, lanes.size());
    for (auto i = 0UL; i < result.size(); ++i) {
      EXPECT_TRUE(equal(result[i], lanes[i]));
    }
    // since the path points do not have ids, all points are cut
    EXPECT_TRUE(path.points.empty());
  }
  {  // add the overlapping lane id to the path points
    reset_path();
    for (auto & p : path.points) {
      p.lane_ids.push_back(lanes.back().left_lane.id());
    }
    cutOverlappedLanes(path, lanes);
    // since the overlapped lane was removed, the path points were still cut
    EXPECT_TRUE(path.points.empty());
  }
  {  // add the first lane id to some path points, only these points will be kept
    reset_path();
    constexpr auto filtered_start = 3UL;
    constexpr auto filtered_size = 5UL;
    for (auto i = filtered_start; i < filtered_start + filtered_size; ++i) {
      path.points[i].lane_ids.push_back(lanes.front().left_lane.id());
    }
    cutOverlappedLanes(path, lanes);
    ASSERT_EQ(path.points.size(), filtered_size);
    for (auto i = 0UL; i < filtered_size; ++i) {
      EXPECT_EQ(path.points[i].point.pose.position.x, (i + filtered_start) * 1.0);
      EXPECT_EQ(path.points[i].point.pose.position.y, 0.0);
    }
  }
}
