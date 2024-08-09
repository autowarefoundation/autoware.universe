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
#ifndef NERF__RAYS_HPP_
#define NERF__RAYS_HPP_

#include <torch/torch.h>

struct alignas(32) Rays
{
  torch::Tensor origins;
  torch::Tensor dirs;
};

Rays get_rays_from_pose(
  const torch::Tensor & pose, const torch::Tensor & intrinsic, const torch::Tensor & ij);

#endif  // NERF__RAYS_HPP_
