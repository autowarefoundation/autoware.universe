// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#ifndef COSTMAP_GENERATOR__OBJECT_MAP_UTILS_HPP_
#define COSTMAP_GENERATOR__OBJECT_MAP_UTILS_HPP_

#include <string>
#include <vector>

#include "grid_map_cv/grid_map_cv.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace object_map
{
/*!
  * Projects the in_area_points forming the road, stores the result in out_grid_map.
  * @param[out] out_grid_map GridMap object to add the road grid
  * @param[in] in_area_points Array of points containing the wayareas
  * @param[in] in_grid_layer_name Name to assign to the layer
  * @param[in] in_layer_background_value Empty state value
  * @param[in] in_fill_color Value to fill on wayareas
  * @param[in] in_layer_min_value Minimum value in the layer
  * @param[in] in_layer_max_value Maximum value in the later
  * @param[in] in_transform Most recent transform for wayarea points (from map to costmap frame)
  */
void fillPolygonAreas(
  grid_map::GridMap & out_grid_map,
  const std::vector<std::vector<geometry_msgs::msg::Point>> & in_area_points,
  const std::string & in_grid_layer_name, const int in_layer_background_value,
  const int in_fill_color, const int in_layer_min_value, const int in_layer_max_value,
  const geometry_msgs::msg::TransformStamped & in_transform);

}  // namespace object_map

#endif  // COSTMAP_GENERATOR__OBJECT_MAP_UTILS_HPP_
