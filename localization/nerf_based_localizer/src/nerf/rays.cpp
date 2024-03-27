// Copyright 2023 Autoware Foundation
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
#include "rays.hpp"

#include "common.hpp"

using Tensor = torch::Tensor;

Rays get_rays_from_pose(const Tensor & pose, const Tensor & intrinsic, const Tensor & ij)
{
  // Shift half pixel
  Tensor i = ij.index({"...", 0}).to(torch::kFloat32) + .5f;
  Tensor j = ij.index({"...", 1}).to(torch::kFloat32) + .5f;

  Tensor cx = intrinsic.index({Slc(), 0, 2});
  Tensor cy = intrinsic.index({Slc(), 1, 2});
  Tensor fx = intrinsic.index({Slc(), 0, 0});
  Tensor fy = intrinsic.index({Slc(), 1, 1});

  Tensor u_tensor = ((j - cx) / fx).unsqueeze(-1);
  Tensor v_tensor = -((i - cy) / fy).unsqueeze(-1);
  Tensor w_tensor = -torch::ones_like(u_tensor);

  Tensor dir_tensor = torch::cat({u_tensor, v_tensor, w_tensor}, 1).unsqueeze(-1);
  Tensor ori_tensor = pose.index({Slc(), Slc(0, 3), Slc(0, 3)});
  Tensor pos_tensor = pose.index({Slc(), Slc(0, 3), 3});
  Tensor rays_d = torch::matmul(ori_tensor, dir_tensor).squeeze();
  Tensor rays_o = pos_tensor.expand({rays_d.sizes()[0], 3}).contiguous();

  return {rays_o, rays_d};
}
