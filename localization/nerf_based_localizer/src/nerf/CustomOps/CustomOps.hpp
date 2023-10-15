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
// Created by ppwang on 2022/10/5.
//

#ifndef NERF_BASED_LOCALIZER__CUSTOM_OPS_HPP_
#define NERF_BASED_LOCALIZER__CUSTOM_OPS_HPP_

#include <torch/torch.h>

namespace torch::autograd
{

class TruncExp : public Function<TruncExp>
{
public:
  static variable_list forward(AutogradContext * ctx, Tensor input);

  static variable_list backward(AutogradContext * ctx, variable_list grad_output);
};

}  // namespace torch::autograd

namespace CustomOps
{

torch::Tensor WeightVar(torch::Tensor weights, torch::Tensor idx_start_end);

}

#endif  // NERF_BASED_LOCALIZER__CUSTOM_OPS_HPP_
