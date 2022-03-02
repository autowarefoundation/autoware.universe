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

#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <stdint.h>

namespace centerpoint
{
class Config
{
public:
  // input params
  constexpr static int num_point_dims = 3;      // x, y and z
  constexpr static int num_point_features = 4;  // x, y, z and timelag
  constexpr static int max_num_points_per_voxel = 32;
  constexpr static int max_num_voxels = 40000;
  constexpr static float range_min_x = -89.6f;
  constexpr static float range_min_y = -89.6f;
  constexpr static float range_min_z = -3.0f;
  constexpr static float range_max_x = 89.6f;
  constexpr static float range_max_y = 89.6f;
  constexpr static float range_max_z = 5.0f;
  constexpr static float voxel_size_x = 0.32f;
  constexpr static float voxel_size_y = 0.32f;
  constexpr static float voxel_size_z = 8.0f;

  // network params
  constexpr static int batch_size = 1;
  constexpr static int downsample_factor = 2;
  constexpr static int num_encoder_input_features = 9;
  constexpr static int num_encoder_output_features = 32;
  constexpr static int num_output_features = 6;
  constexpr static int num_output_offset_features = 2;
  constexpr static int num_output_z_features = 1;
  constexpr static int num_output_dim_features = 3;
  constexpr static int num_output_rot_features = 2;
  constexpr static int num_output_vel_features = 2;
  constexpr static int box_feature_size = 9;  // x, y, z, l, w, h, rot, vel_x, vel_y

  // calculated params
  constexpr static int grid_size_x = (range_max_x - range_min_x) / voxel_size_x;
  constexpr static int grid_size_y = (range_max_y - range_min_y) / voxel_size_y;
  constexpr static int grid_size_z = (range_max_z - range_min_z) / voxel_size_z;
  constexpr static float offset_x = range_min_x + voxel_size_x / 2;
  constexpr static float offset_y = range_min_y + voxel_size_y / 2;
  constexpr static float offset_z = range_min_z + voxel_size_z / 2;
  constexpr static int down_grid_size_x = grid_size_x / downsample_factor;
  constexpr static int down_grid_size_y = grid_size_y / downsample_factor;
  constexpr static int max_num_detections = down_grid_size_y * down_grid_size_x;
};

}  // namespace centerpoint

#endif  // CONFIG_HPP_
