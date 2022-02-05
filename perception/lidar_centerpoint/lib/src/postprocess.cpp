// Copyright 2022 Tier IV, Inc.
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

#include <config.hpp>
#include <postprocess.hpp>

#include <cmath>

namespace centerpoint
{
void generatePredictedBoxes(
  std::vector<float> & output_heatmap, std::vector<float> & output_offset,
  std::vector<float> & output_z, std::vector<float> & output_dim, std::vector<float> & output_rot,
  std::vector<float> & output_vel, std::vector<Box> & boxes)
{
  // output: shape of (N, GRID_SIZE_Y, GRID_SIZE_X)
  // heatmap: N = num_class, offset: N = 2, z: N = 1, dim: N = 3, rot: N = 2, vel: N = 2

  const size_t downsample_grid_y =
    static_cast<size_t>(static_cast<float>(Config::grid_size_y) / Config::downsample_factor);
  const size_t downsample_grid_x =
    static_cast<size_t>(static_cast<float>(Config::grid_size_x) / Config::downsample_factor);
  const size_t downsample_grid_size = downsample_grid_x * downsample_grid_x;
  const float score_threshold = 0.2;
  boxes.reserve(downsample_grid_size);

  for (size_t iy = 0; iy < downsample_grid_y; iy++) {
    for (size_t ix = 0; ix < downsample_grid_x; ix++) {
      size_t idx = downsample_grid_x * iy + ix;

      size_t label = 0;
      float max_score = -1;
      float score;
      for (size_t ci = 0; ci < 3; ci++) {
        score = output_heatmap[downsample_grid_size * ci + idx];
        score = 1.0 / std::exp(-score);  // sigmoid
        if (score > max_score) {
          label = ci;
          max_score = score;
        }
      }
      if (max_score < score_threshold) {
        continue;
      }

      float offset_x = output_offset[downsample_grid_size * 0 + idx];
      float offset_y = output_offset[downsample_grid_size * 1 + idx];
      float x =
        Config::voxel_size_x * Config::downsample_factor * (ix + offset_x) + Config::range_min_x;
      float y =
        Config::voxel_size_y * Config::downsample_factor * (iy + offset_y) + Config::range_min_y;

      Box box;
      box.score = max_score;
      box.label = label;
      box.loc_x = x;
      box.loc_y = y;
      box.loc_z = output_z[downsample_grid_size * 0 + idx];
      box.dim_x = std::exp(output_dim[downsample_grid_size * 0 + idx]);
      box.dim_y = std::exp(output_dim[downsample_grid_size * 1 + idx]);
      box.dim_z = std::exp(output_dim[downsample_grid_size * 2 + idx]);
      box.rot_x = output_rot[downsample_grid_size * 0 + idx];
      box.rot_y = output_rot[downsample_grid_size * 1 + idx];
      box.vel_x = output_vel[downsample_grid_size * 0 + idx];
      box.vel_y = output_vel[downsample_grid_size * 1 + idx];
      boxes.emplace_back(box);
    }
  }
}

}  // namespace centerpoint
