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

#ifndef POSTPROCESS_HPP_
#define POSTPROCESS_HPP_

#include <vector>

namespace centerpoint
{
struct Box
{
  size_t label;
  float score;
  float loc_x;
  float loc_y;
  float loc_z;
  float dim_x;
  float dim_y;
  float dim_z;
  float vel_x;
  float vel_y;
  float rot_x;
  float rot_y;
};

void generatePredictedBoxes(
  std::vector<float> & output_heatmap, std::vector<float> & output_offset,
  std::vector<float> & output_z, std::vector<float> & output_dim, std::vector<float> & output_rot,
  std::vector<float> & output_vel, std::vector<Box> & pred_boxes);

}  // namespace centerpoint

#endif  // POSTPROCESS_HPP_
