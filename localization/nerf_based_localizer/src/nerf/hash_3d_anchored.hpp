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
// Created by ppwang on 2022/7/17.
//

#ifndef NERF_BASED_LOCALIZER__HASH_3D_ANCHORED_HPP_
#define NERF_BASED_LOCALIZER__HASH_3D_ANCHORED_HPP_

#include <torch/torch.h>

static constexpr int64_t N_CHANNELS = 2;
static constexpr int64_t N_LEVELS = 16;

class Hash3DAnchored : public torch::nn::Module
{
  using Tensor = torch::Tensor;

public:
  Hash3DAnchored();

  Tensor query(const Tensor & points);

  std::vector<torch::optim::OptimizerParamGroup> optim_param_groups(float lr);

  int pool_size_;
  int local_size_;

  Tensor feat_pool_;  // [ pool_size_, n_channels_ ];
  Tensor prim_pool_;  // [ n_levels, 3 ];
  Tensor bias_pool_;  // [ n_levels, 3 ];

  torch::nn::Linear mlp_ = nullptr;
};

class Hash3DAnchoredInfo : public torch::CustomClassHolder
{
public:
  Hash3DAnchored * hash3d_ = nullptr;
};

namespace torch::autograd
{

class Hash3DAnchoredFunction : public Function<Hash3DAnchoredFunction>
{
public:
  static variable_list forward(
    AutogradContext * ctx, Tensor points, Tensor feat_pool_, IValue hash3d_info);

  static variable_list backward(AutogradContext * ctx, variable_list grad_output);
};

}  // namespace torch::autograd

#endif  // NERF_BASED_LOCALIZER__HASH_3D_ANCHORED_HPP_
