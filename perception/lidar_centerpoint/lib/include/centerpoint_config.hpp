// Copyright 2021 TIER IV, Inc.
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

#ifndef CENTERPOINT_CONFIG_HPP_
#define CENTERPOINT_CONFIG_HPP_

#include <cstddef>

namespace centerpoint
{
class CenterPointConfig
{
public:
  // input params
  const std::size_t point_dim_size_{3};  // x, y and z
  std::size_t point_feature_size_{4};    // x, y, z and timelag
  std::size_t max_num_points_per_voxel_{32};
  std::size_t max_num_voxels_{40000};
  float range_min_x_{-89.6f};
  float range_min_y_{-89.6f};
  float range_min_z_{-3.0f};
  float range_max_x_{89.6f};
  float range_max_y_{89.6f};
  float range_max_z_{5.0f};
  float voxel_size_x_{0.32f};
  float voxel_size_y_{0.32f};
  float voxel_size_z_{8.0f};

  // network params
  const std::size_t batch_size_{1};
  std::size_t downsample_factor_{2};
  std::size_t encoder_in_feature_size_{9};
  const std::size_t encoder_out_feature_size_{32};
  const std::size_t head_out_size_{6};
  const std::size_t head_out_offset_size_{2};
  const std::size_t head_out_z_size_{1};
  const std::size_t head_out_dim_size_{3};
  const std::size_t head_out_rot_size_{2};
  const std::size_t head_out_vel_size_{2};

  // calculated params
  std::size_t grid_size_x_ = (range_max_x_ - range_min_x_) / voxel_size_x_;
  std::size_t grid_size_y_ = (range_max_y_ - range_min_y_) / voxel_size_y_;
  std::size_t grid_size_z_ = (range_max_z_ - range_min_z_) / voxel_size_z_;
  float offset_x_ = range_min_x_ + voxel_size_x_ / 2;
  float offset_y_ = range_min_y_ + voxel_size_y_ / 2;
  float offset_z_ = range_min_z_ + voxel_size_z_ / 2;
  std::size_t down_grid_size_x_ = grid_size_x_ / downsample_factor_;
  std::size_t down_grid_size_y_ = grid_size_y_ / downsample_factor_;
};

}  // namespace centerpoint

#endif  // CENTERPOINT_CONFIG_HPP_
