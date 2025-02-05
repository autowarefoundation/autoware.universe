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

#ifndef AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_FIXED_HPP_
#define AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_FIXED_HPP_

#include "autoware/probabilistic_occupancy_grid_map/costmap_2d/occupancy_grid_map_base.hpp"
#include "autoware/probabilistic_occupancy_grid_map/utils/cuda_pointcloud.hpp"

namespace autoware::occupancy_grid_map
{
namespace costmap_2d
{
using geometry_msgs::msg::Pose;
using sensor_msgs::msg::PointCloud2;

class OccupancyGridMapFixedBlindSpot : public OccupancyGridMapInterface
{
public:
  OccupancyGridMapFixedBlindSpot(
    const bool use_cuda, const unsigned int cells_size_x, const unsigned int cells_size_y,
    const float resolution);

  void updateWithPointCloud(
    const CudaPointCloud2 & raw_pointcloud, const CudaPointCloud2 & obstacle_pointcloud,
    const Pose & robot_pose, const Pose & scan_origin) override;

  void initRosParam(rclcpp::Node & node) override;

protected:
  double distance_margin_;

  autoware::cuda_utils::CudaUniquePtr<std::uint64_t[]> raw_points_tensor_;
  autoware::cuda_utils::CudaUniquePtr<std::uint64_t[]> obstacle_points_tensor_;
};

}  // namespace costmap_2d
}  // namespace autoware::occupancy_grid_map

#endif  // AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_FIXED_HPP_
