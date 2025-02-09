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

#ifndef AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_PROJECTIVE_HPP_
#define AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_PROJECTIVE_HPP_

#include "autoware/probabilistic_occupancy_grid_map/costmap_2d/occupancy_grid_map_base.hpp"

#include <grid_map_core/GridMap.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>

namespace autoware::occupancy_grid_map
{
namespace costmap_2d
{
using geometry_msgs::msg::Pose;
using sensor_msgs::msg::PointCloud2;

class OccupancyGridMapProjectiveBlindSpot : public OccupancyGridMapInterface
{
public:
  OccupancyGridMapProjectiveBlindSpot(
    const bool use_cuda, const unsigned int cells_size_x, const unsigned int cells_size_y,
    const float resolution);

  void updateWithPointCloud(
    const CudaPointCloud2 & raw_pointcloud, const CudaPointCloud2 & obstacle_pointcloud,
    const Pose & robot_pose, const Pose & scan_origin) override;

  void initRosParam(rclcpp::Node & node) override;

private:
  float projection_dz_threshold_;
  float obstacle_separation_threshold_;

  autoware::cuda_utils::CudaUniquePtr<std::uint64_t[]> raw_points_tensor_;
  autoware::cuda_utils::CudaUniquePtr<std::uint64_t[]> obstacle_points_tensor_;
  autoware::cuda_utils::CudaUniquePtr<Eigen::Vector3f> device_translation_scan_origin_;
};

}  // namespace costmap_2d
}  // namespace autoware::occupancy_grid_map

#endif  // AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_PROJECTIVE_HPP_
