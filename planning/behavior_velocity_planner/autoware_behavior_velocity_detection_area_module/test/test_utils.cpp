// Copyright 2024 Tier IV, Inc.
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

#include "../src/utils.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Polygon.h>

TEST(TestUtils, getStopLine)
{
  using autoware::behavior_velocity_planner::detection_area::get_stop_line_geometry2d;
  lanelet::LineString3d line;
  line.push_back(lanelet::Point3d(lanelet::InvalId, 0.0, -1.0));
  line.push_back(lanelet::Point3d(lanelet::InvalId, 0.0, 1.0));
  lanelet::Polygons3d detection_areas;
  lanelet::Polygon3d area;
  area.push_back(lanelet::Point3d(lanelet::InvalId, 1.0, -1.0));
  area.push_back(lanelet::Point3d(lanelet::InvalId, 1.0, 1.0));
  area.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, 1.0));
  area.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, -1.0));
  detection_areas.push_back(area);
  auto detection_area =
    lanelet::autoware::DetectionArea::make(lanelet::InvalId, {}, detection_areas, line);
  {
    const double extend_length = 0.0;
    const auto stop_line = get_stop_line_geometry2d(*detection_area, extend_length);
    ASSERT_EQ(stop_line.size(), 2UL);
    EXPECT_EQ(stop_line[0].x(), line[0].x());
    EXPECT_EQ(stop_line[0].y(), line[0].y());
    EXPECT_EQ(stop_line[1].x(), line[1].x());
    EXPECT_EQ(stop_line[1].y(), line[1].y());
  }
  // extended line
  for (auto extend_length = -2.0; extend_length < 2.0; extend_length += 0.1) {
    const auto stop_line = get_stop_line_geometry2d(*detection_area, extend_length);
    ASSERT_EQ(stop_line.size(), 2UL);
    EXPECT_EQ(stop_line[0].x(), line[0].x());
    EXPECT_EQ(stop_line[0].y(), line[0].y() - extend_length);
    EXPECT_EQ(stop_line[1].x(), line[1].x());
    EXPECT_EQ(stop_line[1].y(), line[1].y() + extend_length);
  }
}
