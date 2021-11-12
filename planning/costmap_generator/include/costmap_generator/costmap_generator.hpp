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

/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#ifndef COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
#define COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_

#include <lanelet2_core/primitives/Lanelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_auto_planning_msgs/action/planner_costmap.hpp>
#include <costmap_generator/visibility_control.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>
#include <tuple>


namespace autoware
{
namespace planning
{
namespace costmap_generator
{
/// \brief Struct holding costmap layer names
struct COSTMAP_GENERATOR_PUBLIC LayerName
{
  /// Name of layer only with lanelet2 info
  static constexpr const char * WAYAREA = "wayarea";
  /// Name of layer which has all information applied
  static constexpr const char * COMBINED = "combined";
};

/// \brief Parameters for CostmapGenerator class
struct COSTMAP_GENERATOR_PUBLIC CostmapGeneratorParams
{
  bool use_wayarea;           ///< Decide if apply Lanelet2 info to costmap
  bool bound_costmap;         ///< Decide if truncate costmap regarding only crucial information
  double grid_min_value;      ///< Minimal costmap grid value (freespace)
  double grid_max_value;      ///< Maximal costmap grid value (obstacle)
  double grid_resolution;     ///< Costmap resolution
  double grid_length_x;       ///< Costmap x direction size
  double grid_length_y;       ///< Costmap y direction size
  double grid_position_x;     ///< X position of costmap in its frame
  double grid_position_y;     ///< Y position of costmap in its frame
  std::string costmap_frame;  ///< Costmap frame name
};

/// \class CostmapGenerator
/// \brief Costmap generation algorithm regarding lanelet2 data
class COSTMAP_GENERATOR_PUBLIC CostmapGenerator
{
public:
  /// \brief Initialize gridmap parameters based on rosparam
  /// \param [in] generator_params Costmap generation algorithm configuration
  explicit CostmapGenerator(const CostmapGeneratorParams & generator_params);

  /// \brief Generate costmap
  /// \details This is main function which is capable of creating
  ///          GridMap, basing on lanelet2 information and
  ///          transforms between map, costmap, and vehicle
  ///          positions. It is possible to decide if the function
  ///          should truncate the final costmap or apply driveable
  ///          areas by setting proper configuration parameters
  /// \param [in] lanelet_ptr Pointer to lanelet2 map, used for applying driveable areas to costmap
  /// \param [in] vehicle_to_grid_position Translation between costmap and vehicle frame used to
  ///                                     align costmap and vehicle center
  /// \param [in] map_to_costmap_transform Transform between map and costmap. Used for converting
  ///                                     lanelet polygon points when marking driveable areas
  /// \return Generated costmap
  grid_map::GridMap generateCostmap(
    lanelet::LaneletMapPtr lanelet_ptr, const grid_map::Position & vehicle_to_grid_position,
    const geometry_msgs::msg::TransformStamped & map_to_costmap_transform);

private:
  grid_map::GridMap costmap_;
  CostmapGeneratorParams params_;
  std::vector<std::vector<geometry_msgs::msg::Point>> area_points_;

  /// \brief Fills costmap data according to given lanelet roads and parking areas
  void loadDrivableAreasFromLaneletMap(lanelet::LaneletMapPtr lanelet_ptr);

  /// \brief Find bounding box for all received lanelet polygons
  std::tuple<grid_map::Position, grid_map::Position> calculateAreaPointsBoundingBox() const;

  /// \brief Perform final costmap bounding regarding received lanelet information
  grid_map::GridMap boundCostmap(
    const grid_map::Position & min_point, const grid_map::Position & max_point) const;

  /// \brief set area_points_ from lanelet polygons
  void loadRoadAreasFromLaneletMap(lanelet::LaneletMapPtr lanelet_map);

  /// \brief set area_points_ from parking areas
  void loadParkingAreasFromLaneletMap(lanelet::LaneletMapPtr lanelet_map);

  /// \brief calculate cost from lanelet2 map
  grid_map::Matrix generateWayAreaCostmap(
    const geometry_msgs::msg::TransformStamped & map_to_costmap_transform) const;

  /// \brief Calculate costmap layer costs for final output
  /// \return Costmap layer
  grid_map::Matrix generateCombinedCostmap() const;
};

}  // namespace costmap_generator
}  // namespace planning
}  // namespace autoware

#endif  // COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
