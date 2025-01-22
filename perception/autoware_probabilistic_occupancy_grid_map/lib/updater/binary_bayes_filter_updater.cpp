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

#include "autoware/probabilistic_occupancy_grid_map/updater/binary_bayes_filter_updater.hpp"

#include "autoware/probabilistic_occupancy_grid_map/cost_value/cost_value.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::occupancy_grid_map
{
namespace costmap_2d
{

OccupancyGridMapBBFUpdater::OccupancyGridMapBBFUpdater(
  const unsigned int cells_size_x, const unsigned int cells_size_y, const float resolution)
: OccupancyGridMapUpdaterInterface(cells_size_x, cells_size_y, resolution)
{
}

void OccupancyGridMapBBFUpdater::initRosParam(rclcpp::Node & node)
{
  probability_matrix_(Index::OCCUPIED, Index::OCCUPIED) =
    node.declare_parameter<double>("probability_matrix.occupied_to_occupied");
  probability_matrix_(Index::FREE, Index::OCCUPIED) =
    node.declare_parameter<double>("probability_matrix.occupied_to_free");
  probability_matrix_(Index::FREE, Index::FREE) =
    node.declare_parameter<double>("probability_matrix.free_to_free");
  probability_matrix_(Index::OCCUPIED, Index::FREE) =
    node.declare_parameter<double>("probability_matrix.free_to_occupied");
  v_ratio_ = node.declare_parameter<double>("v_ratio");
}

inline unsigned char OccupancyGridMapBBFUpdater::applyBBF(
  const unsigned char & z, const unsigned char & o)
{
  constexpr float cost2p = 1.f / 255.f;
  const float po = o * cost2p;
  float pz{};
  float not_pz{};
  float po_hat{};
  if (z == cost_value::LETHAL_OBSTACLE) {
    pz = probability_matrix_(Index::OCCUPIED, Index::OCCUPIED);
    not_pz = probability_matrix_(Index::FREE, Index::OCCUPIED);
    po_hat = ((po * pz) / ((po * pz) + ((1.f - po) * not_pz)));
  } else if (z == cost_value::FREE_SPACE) {
    pz = 1.f - probability_matrix_(Index::FREE, Index::FREE);
    not_pz = 1.f - probability_matrix_(Index::OCCUPIED, Index::FREE);
    po_hat = ((po * pz) / ((po * pz) + ((1.f - po) * not_pz)));
  } else if (z == cost_value::NO_INFORMATION) {
    const float inv_v_ratio = 1.f / v_ratio_;
    po_hat = ((po + (0.5f * inv_v_ratio)) / ((1.f * inv_v_ratio) + 1.f));
  }
  return std::min(
    std::max(
      static_cast<unsigned char>(std::lround(po_hat * 255.f)), static_cast<unsigned char>(1)),
    static_cast<unsigned char>(254));
}

bool OccupancyGridMapBBFUpdater::update(const Costmap2D & single_frame_occupancy_grid_map)
{
  updateOrigin(
    single_frame_occupancy_grid_map.getOriginX(), single_frame_occupancy_grid_map.getOriginY());
  for (unsigned int x = 0; x < getSizeInCellsX(); x++) {
    for (unsigned int y = 0; y < getSizeInCellsY(); y++) {
      unsigned int index = getIndex(x, y);
      costmap_[index] = applyBBF(single_frame_occupancy_grid_map.getCost(x, y), costmap_[index]);
    }
  }
  return true;
}

}  // namespace costmap_2d
}  // namespace autoware::occupancy_grid_map
