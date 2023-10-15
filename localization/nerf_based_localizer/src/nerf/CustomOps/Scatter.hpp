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
//
// Created by ppwang on 2023/3/27.
//

#ifndef NERF_BASED_LOCALIZER__SCATTER_HPP_
#define NERF_BASED_LOCALIZER__SCATTER_HPP_

#include "../common.hpp"

#include <torch/torch.h>

class Scatter
{
};

namespace CustomOps
{

torch::Tensor ScatterAdd(torch::Tensor emb, torch::Tensor idx, torch::Tensor to_add);
torch::Tensor ScatterIdx(int n_all_pts, torch::Tensor idx_start_end, torch::Tensor emb_idx);
}  // namespace CustomOps

#endif  // NERF_BASED_LOCALIZER__SCATTER_HPP_
