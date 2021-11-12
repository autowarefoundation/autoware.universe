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
 *  * Redistributions of source code must retain the above copyright notice, private_node
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    private_node list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    private_node software without specific prior written permission.
 *
 *  private_node SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF private_node SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#include "costmap_generator/costmap_generator.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <tuple>
#include <limits>
#include <algorithm>

#include "costmap_generator/object_map_utils.hpp"
#include "had_map_utils/had_map_conversion.hpp"
#include "had_map_utils/had_map_query.hpp"
#include "had_map_utils/had_map_visualization.hpp"
#include "tf2/utils.h"

// Convert from Point32 to Point
std::vector<geometry_msgs::msg::Point> poly2vector(const geometry_msgs::msg::Polygon & poly)
{
  std::vector<geometry_msgs::msg::Point> ps;
  for (const auto & p32 : poly.points) {
    geometry_msgs::msg::Point p;
    p.x = p32.x;
    p.y = p32.y;
    p.z = p32.z;
    ps.push_back(p);
  }
  return ps;
}

namespace autoware
{
namespace planning
{
namespace costmap_generator
{
CostmapGenerator::CostmapGenerator(const CostmapGeneratorParams & generator_params)
: params_(generator_params)
{
  costmap_.setFrameId(params_.costmap_frame);
  costmap_.setGeometry(
    grid_map::Length(params_.grid_length_x, params_.grid_length_y), params_.grid_resolution,
    grid_map::Position(params_.grid_position_x, params_.grid_position_y));

  costmap_.add(LayerName::WAYAREA, params_.grid_min_value);
  costmap_.add(LayerName::COMBINED, params_.grid_min_value);
}

void CostmapGenerator::loadDrivableAreasFromLaneletMap(lanelet::LaneletMapPtr lanelet_ptr)
{
  loadRoadAreasFromLaneletMap(lanelet_ptr);
  loadParkingAreasFromLaneletMap(lanelet_ptr);
}

void CostmapGenerator::loadParkingAreasFromLaneletMap(const lanelet::LaneletMapPtr lanelet_map)
{
  // Extract parking areas
  auto areas = autoware::common::had_map_utils::getAreaLayer(lanelet_map);
  auto parking_spot_areas = autoware::common::had_map_utils::subtypeAreas(areas, "parking_spot");
  auto parking_access_areas =
    autoware::common::had_map_utils::subtypeAreas(areas, "parking_access");

  // Collect points from parking spots
  for (const auto & parking_spot_area : parking_spot_areas) {
    auto parking_spot_poly = autoware::common::had_map_utils::area2Polygon(parking_spot_area);
    area_points_.push_back(poly2vector(parking_spot_poly));
  }

  // Collect points from parking spaces
  for (const auto & parking_access_area : parking_access_areas) {
    auto parking_access_poly = autoware::common::had_map_utils::area2Polygon(parking_access_area);
    area_points_.push_back(poly2vector(parking_access_poly));
  }
}

void CostmapGenerator::loadRoadAreasFromLaneletMap(const lanelet::LaneletMapPtr lanelet_map)
{
  auto road_lanelets = autoware::common::had_map_utils::getConstLaneletLayer(lanelet_map);

  for (const auto & road_lanelet : road_lanelets) {
    auto road_poly = autoware::common::had_map_utils::lanelet2Polygon(road_lanelet);
    area_points_.push_back(poly2vector(road_poly));
  }
}

grid_map::GridMap CostmapGenerator::generateCostmap(
  lanelet::LaneletMapPtr lanelet_ptr, const grid_map::Position & vehicle_to_grid_position,
  const geometry_msgs::msg::TransformStamped & map_to_costmap_transform)
{
  // Clear data points
  area_points_.clear();

  // Supply Lanelet map to costmap generator
  loadDrivableAreasFromLaneletMap(lanelet_ptr);

  // Move grid map with data to robot's center position
  costmap_.setPosition(vehicle_to_grid_position);

  // Apply lanelet2 info to costmap
  if (params_.use_wayarea) {
    costmap_[LayerName::WAYAREA] = generateWayAreaCostmap(map_to_costmap_transform);
  }

  costmap_[LayerName::COMBINED] = generateCombinedCostmap();

  auto result = costmap_;

  if (params_.bound_costmap) {
    grid_map::Position had_map_min_point, had_map_max_point;
    std::tie(had_map_min_point, had_map_max_point) = calculateAreaPointsBoundingBox();

    result = boundCostmap(had_map_min_point, had_map_max_point);
  }

  return result;
}

grid_map::Matrix CostmapGenerator::generateCombinedCostmap() const
{
  // assuming combined_costmap is calculated by element wise max operation
  grid_map::GridMap combined_costmap = costmap_;

  combined_costmap[LayerName::COMBINED].setConstant(static_cast<float>(params_.grid_min_value));

  combined_costmap[LayerName::COMBINED] =
    combined_costmap[LayerName::COMBINED].cwiseMax(combined_costmap[LayerName::WAYAREA]);

  return combined_costmap[LayerName::COMBINED];
}

grid_map::Matrix CostmapGenerator::generateWayAreaCostmap(
  const geometry_msgs::msg::TransformStamped & map_to_costmap_transform) const
{
  grid_map::GridMap lanelet2_costmap = costmap_;
  if (!area_points_.empty()) {
    const auto & grid_min_value = static_cast<int>(params_.grid_min_value);
    const auto & grid_max_value = static_cast<int>(params_.grid_max_value);
    object_map::fillPolygonAreas(
      lanelet2_costmap, area_points_, LayerName::WAYAREA,
      grid_max_value, grid_min_value, grid_min_value,
      grid_max_value, map_to_costmap_transform);
  }
  return lanelet2_costmap[LayerName::WAYAREA];
}

grid_map::GridMap CostmapGenerator::boundCostmap(
  const grid_map::Position & min_point, const grid_map::Position & max_point) const
{
  auto success = false;
  const auto length =
    grid_map::Length(max_point.x() - min_point.x(), max_point.y() - min_point.y());
  const auto position =
    grid_map::Position((max_point.x() + min_point.x()) / 2, (max_point.y() + min_point.y()) / 2);

  auto subcostmap = costmap_.getSubmap(position, length, success);

  if (!success) {
    return costmap_;
  }

  return subcostmap;
}

std::tuple<grid_map::Position, grid_map::Position>
CostmapGenerator::calculateAreaPointsBoundingBox() const
{
  grid_map::Position had_map_min_point = {
    std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  grid_map::Position had_map_max_point = {
    std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};

  for (const auto & area : area_points_) {
    for (const auto & point : area) {
      had_map_max_point.x() = std::max(had_map_max_point.x(), point.x);
      had_map_max_point.y() = std::max(had_map_max_point.y(), point.y);
      had_map_min_point.x() = std::min(had_map_min_point.x(), point.x);
      had_map_min_point.y() = std::min(had_map_min_point.y(), point.y);
    }
  }

  return {had_map_min_point, had_map_max_point};
}

}  // namespace costmap_generator
}  // namespace planning
}  // namespace autoware
