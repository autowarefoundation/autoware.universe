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
// Created by ppwang on 2023/2/11.
//

#ifndef NERF__CUSTOMOPS__FLEXOPS_HPP_
#define NERF__CUSTOMOPS__FLEXOPS_HPP_

#include "../common.hpp"

#include <torch/torch.h>

namespace FlexOps
{

torch::Tensor Sum(torch::Tensor val, torch::Tensor idx_start_end);
torch::Tensor AccumulateSum(torch::Tensor val, torch::Tensor idx_start_end, bool include_this);

}  // namespace FlexOps

#endif  // NERF__CUSTOMOPS__FLEXOPS_HPP_
