// Copyright 2024 The Autoware Contributors
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

#include <autoware/costmap_generator/utils/object_map_utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

namespace
{
grid_map::GridMap construct_gridmap(
  const double length_x, const double length_y, const double resolution, const double position_x,
  const double position_y)
{
  grid_map::GridMap gm;

  gm.setFrameId("map");
  // set gridmap size,resolution
  gm.setGeometry(grid_map::Length(length_x, length_y), resolution);
  // center of grid to position p in map frame
  gm.setPosition(grid_map::Position(position_x, position_y));

  // set initial value
  gm.add("primitives", 0);

  return gm;
}

std::vector<geometry_msgs::msg::Point> get_primitive_points(
  const double min_x, const double min_y, const double max_x, const double max_y)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(autoware::universe_utils::createPoint(min_x, min_y, 0.0));
  points.push_back(autoware::universe_utils::createPoint(min_x, max_y, 0.0));
  points.push_back(autoware::universe_utils::createPoint(max_x, max_y, 0.0));
  points.push_back(autoware::universe_utils::createPoint(max_x, min_y, 0.0));
  return points;
}
}  // namespace

namespace autoware::cosmap_generator
{
TEST(ObjectMapUtilsTest, testFillPolygonAreas)
{
  const double grid_length_x = 21;
  const double grid_length_y = 21;
  const double grid_resolution = 1.0;
  const double grid_position_x = 0.0;
  const double grid_position_y = 0.0;
  grid_map::GridMap gridmap = construct_gridmap(
    grid_length_x, grid_length_y, grid_resolution, grid_position_x, grid_position_y);

  std::vector<std::vector<geometry_msgs::msg::Point>> primitives_points;
  primitives_points.emplace_back(get_primitive_points(-15.0, -2.0, 15.0, 2.0));
  primitives_points.emplace_back(get_primitive_points(-5.0, -5.0, 5.0, 5.0));

  const double min_value = 0.0;
  const double max_value = 1.0;

  object_map::FillPolygonAreas(
    gridmap, primitives_points, "primitives", max_value, min_value, min_value, max_value);

  const auto costmap = gridmap["primitives"];

  int empty_grid_cell_num = 0;
  for (int i = 0; i < costmap.rows(); i++) {
    for (int j = 0; j < costmap.cols(); j++) {
      if (costmap(i, j) == min_value) {
        // std::cout << "i:"<< i <<",j:"<<j<< std::endl;
        empty_grid_cell_num += 1;
      }
    }
  }

  EXPECT_EQ(empty_grid_cell_num, 171);
}
}  // namespace autoware::cosmap_generator
