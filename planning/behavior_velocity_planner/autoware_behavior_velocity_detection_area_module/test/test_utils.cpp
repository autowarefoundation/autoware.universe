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

TEST(TestUtils, getObstaclePoints)
{
  using autoware::behavior_velocity_planner::detection_area::get_obstacle_points;
  lanelet::ConstPolygons3d detection_areas;
  lanelet::Polygon3d area;
  area.push_back(lanelet::Point3d(lanelet::InvalId, 1.0, -1.0));
  area.push_back(lanelet::Point3d(lanelet::InvalId, 1.0, 1.0));
  area.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, 1.0));
  area.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, -1.0));
  detection_areas.push_back(area);
  pcl::PointCloud<pcl::PointXYZ> points;
  // empty points
  {
    const auto obstacle_points = get_obstacle_points(detection_areas, points);
    EXPECT_TRUE(obstacle_points.empty());
  }
  // add points outside the detection area
  points.emplace_back(0.0, 0.0, 0.0);
  points.emplace_back(4.0, 4.0, 0.0);
  {
    const auto obstacle_points = get_obstacle_points(detection_areas, points);
    EXPECT_TRUE(obstacle_points.empty());
  }
  // add point on the edge of the detection area (will not be found)
  points.emplace_back(1.0, 1.0, 0.0);
  {
    const auto obstacle_points = get_obstacle_points(detection_areas, points);
    EXPECT_TRUE(obstacle_points.empty());
  }
  // add point inside the detection area (will be found)
  points.emplace_back(2.0, 0.0, 0.0);
  {
    const auto obstacle_points = get_obstacle_points(detection_areas, points);
    ASSERT_EQ(obstacle_points.size(), 1UL);
    EXPECT_EQ(obstacle_points[0].x, points[3].x);
    EXPECT_EQ(obstacle_points[0].y, points[3].y);
  }
  // add a detection area that covers all points
  lanelet::Polygon3d full_area;
  full_area.push_back(lanelet::Point3d(lanelet::InvalId, -10.0, -10.0));
  full_area.push_back(lanelet::Point3d(lanelet::InvalId, -10.0, 10.0));
  full_area.push_back(lanelet::Point3d(lanelet::InvalId, 10.0, 10.0));
  full_area.push_back(lanelet::Point3d(lanelet::InvalId, 10.0, -10.0));
  detection_areas.push_back(full_area);
  {
    const auto obstacle_points = get_obstacle_points(detection_areas, points);
    ASSERT_EQ(obstacle_points.size(), 2UL);  // only the 1st point found for each area are returned
    EXPECT_EQ(obstacle_points[0].x, points[3].x);
    EXPECT_EQ(obstacle_points[0].y, points[3].y);
    EXPECT_EQ(obstacle_points[1].x, points[0].x);
    EXPECT_EQ(obstacle_points[1].y, points[0].y);
  }
}
