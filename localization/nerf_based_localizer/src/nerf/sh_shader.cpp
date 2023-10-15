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
// Created by ppwang on 2022/10/8.
//

#include "sh_shader.hpp"

#include "common.hpp"

using Tensor = torch::Tensor;

SHShader::SHShader()
{
  const int d_in = 32;
  const int d_hidden = 64;
  const int d_out = 3;

  mlp_ = torch::nn::Sequential(
    torch::nn::Linear(d_in, d_hidden), torch::nn::ReLU(), torch::nn::Linear(d_hidden, d_out));
  register_module("mlp", mlp_);
}

Tensor SHShader::query(const Tensor & feats, const Tensor & dirs)
{
  Tensor enc = encode(dirs);
  Tensor input = torch::cat({feats, enc}, -1);
  Tensor output = mlp_->forward(input);
  float eps = 1e-3f;
  return (1.f + 2.f * eps) / (1.f + torch::exp(-output)) - eps;
}

std::vector<torch::optim::OptimizerParamGroup> SHShader::optim_param_groups(float lr)
{
  auto opt = std::make_unique<torch::optim::AdamOptions>(lr);
  opt->betas() = {0.9, 0.99};
  opt->eps() = 1e-15;
  opt->weight_decay() = 1e-6;

  std::vector<Tensor> params = mlp_->parameters();
  return {torch::optim::OptimizerParamGroup(params, std::move(opt))};
}
