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
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#ifndef AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_BASE_HPP_
#define AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_BASE_HPP_

#include "autoware/probabilistic_occupancy_grid_map/utils/cuda_pointcloud.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoware::occupancy_grid_map
{
namespace costmap_2d
{
using geometry_msgs::msg::Pose;
using sensor_msgs::msg::PointCloud2;

class OccupancyGridMapInterface : public nav2_costmap_2d::Costmap2D
{
public:
  OccupancyGridMapInterface(
    const bool use_cuda, const unsigned int cells_size_x, const unsigned int cells_size_y,
    const float resolution);

  virtual void updateWithPointCloud(
    [[maybe_unused]] const CudaPointCloud2 & raw_pointcloud,
    [[maybe_unused]] const CudaPointCloud2 & obstacle_pointcloud,
    [[maybe_unused]] const Pose & robot_pose, [[maybe_unused]] const Pose & scan_origin) {};

  void updateOrigin(double new_origin_x, double new_origin_y) override;

  void resetMaps() override;

  virtual void initRosParam(rclcpp::Node & node) = 0;

  void setHeightLimit(const double min_height, const double max_height);

  double min_height_;
  double max_height_;

  const double min_angle_ = autoware::universe_utils::deg2rad(-180.0);
  const double max_angle_ = autoware::universe_utils::deg2rad(180.0);
  const double angle_increment_inv_ = 1.0 / autoware::universe_utils::deg2rad(0.1);

  Eigen::Matrix4f mat_map_, mat_scan_;

  bool isCudaEnabled() const;

  const autoware::cuda_utils::CudaUniquePtr<std::uint8_t[]> & getDeviceCostmap() const;

  void copyDeviceCostmapToHost() const;

protected:
  rclcpp::Logger logger_{rclcpp::get_logger("pointcloud_based_occupancy_grid_map")};
  rclcpp::Clock clock_{RCL_ROS_TIME};

  double resolution_inv_;

  cudaStream_t stream_;

  bool use_cuda_;
  autoware::cuda_utils::CudaUniquePtr<std::uint8_t[]> device_costmap_;
  autoware::cuda_utils::CudaUniquePtr<std::uint8_t[]> device_costmap_aux_;

  autoware::cuda_utils::CudaUniquePtr<Eigen::Matrix3f> device_rotation_map_;
  autoware::cuda_utils::CudaUniquePtr<Eigen::Vector3f> device_translation_map_;
  autoware::cuda_utils::CudaUniquePtr<Eigen::Matrix3f> device_rotation_scan_;
  autoware::cuda_utils::CudaUniquePtr<Eigen::Vector3f> device_translation_scan_;
};

}  // namespace costmap_2d
}  // namespace autoware::occupancy_grid_map

#endif  // AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_BASE_HPP_
