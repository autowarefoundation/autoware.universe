// Copyright 2021 Tier IV, Inc.
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

#ifndef SCENE_MODULE__OCCLUSION_SPOT__GRID_UTILS_HPP_
#define SCENE_MODULE__OCCLUSION_SPOT__GRID_UTILS_HPP_

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/opencv.hpp>
#include <utilization/boost_geometry_helper.hpp>
#include <utilization/util.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <vector>

namespace behavior_velocity_planner
{
namespace grid_utils
{
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using nav_msgs::msg::OccupancyGrid;
namespace occlusion_cost_value
{
static constexpr unsigned char FREE_SPACE = 0;
static constexpr unsigned char UNKNOWN = 50;
static constexpr unsigned char OCCUPIED = 100;
static constexpr unsigned char FREE_SPACE_IMAGE = 0;
static constexpr unsigned char UNKNOWN_IMAGE = 128;
static constexpr unsigned char OCCUPIED_IMAGE = 255;
}  // namespace occlusion_cost_value

struct GridParam
{
  int free_space_max;  // maximum value of a freespace cell in the occupancy grid
  int occupied_min;    // minimum value of an occupied cell in the occupancy grid
};
struct OcclusionSpotSquare
{
  grid_map::Index index;        // index of the anchor
  grid_map::Position position;  // position of the anchor
  int min_occlusion_size;       // number of cells for each side of the square
};
// @brief structure representing a OcclusionSpot on the OccupancyGrid
struct OcclusionSpot
{
  double distance_along_lanelet;
  lanelet::ConstLanelet lanelet;
  lanelet::BasicPoint2d position;
};
//!< @brief Return true
// if the given cell is a occlusion_spot square of size min_size*min_size in the given grid
bool isOcclusionSpotSquare(
  OcclusionSpotSquare & occlusion_spot, const grid_map::Matrix & grid_data,
  const grid_map::Index & cell, const int min_occlusion_size, const grid_map::Size & grid_size);
//!< @brief Find all occlusion spots inside the given lanelet
void findOcclusionSpots(
  std::vector<grid_map::Position> & occlusion_spot_positions, const grid_map::GridMap & grid,
  const Polygon2d & polygon, const double min_size);
//!< @brief Return true if the path between the two given points is free of occupied cells
bool isCollisionFree(
  const grid_map::GridMap & grid, const grid_map::Position & p1, const grid_map::Position & p2,
  const double radius);
//!< @brief get the corner positions of the square described by the given anchor
void getCornerPositions(
  std::vector<grid_map::Position> & corner_positions, const grid_map::GridMap & grid,
  const OcclusionSpotSquare & occlusion_spot_square);
void imageToOccupancyGrid(const cv::Mat & cv_image, nav_msgs::msg::OccupancyGrid * occupancy_grid);
void toQuantizedImage(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid, cv::Mat * cv_image, const GridParam & param);
void denoiseOccupancyGridCV(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid_ptr,
  grid_map::GridMap & grid_map, const GridParam & param, const bool is_show_debug_window,
  const bool filter_occupancy_grid);
void addObjectsToGridMap(const std::vector<PredictedObject> & objs, grid_map::GridMap & grid);
}  // namespace grid_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OCCLUSION_SPOT__GRID_UTILS_HPP_
