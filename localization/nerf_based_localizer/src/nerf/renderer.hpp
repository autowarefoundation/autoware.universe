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
// Created by ppwang on 2022/5/7.
//

#ifndef NERF__RENDERER_HPP_
#define NERF__RENDERER_HPP_

#include "hash_3d_anchored.hpp"
#include "points_sampler.hpp"
#include "sh_shader.hpp"

#include <memory>
#include <vector>

struct RenderResult
{
  using Tensor = torch::Tensor;
  Tensor colors;
  Tensor depths;
  Tensor weights;
  Tensor idx_start_end;
};

class Renderer : public torch::nn::Module
{
  using Tensor = torch::Tensor;

public:
  Renderer(int n_images, const int sample_num_per_ray = 1024);

  RenderResult render(
    const Tensor & rays_o, const Tensor & rays_d, const Tensor & emb_idx, RunningMode mode);

  std::tuple<Tensor, Tensor> render_all_rays(
    const Tensor & rays_o, const Tensor & rays_d, const int batch_size);

  std::tuple<Tensor, Tensor> render_image(
    const torch::Tensor & pose, const torch::Tensor & intrinsic, const int h, const int w,
    const int batch_size);

  std::vector<torch::optim::OptimizerParamGroup> optim_param_groups(float lr);

private:
  const int sample_num_per_ray_;

  std::shared_ptr<PtsSampler> pts_sampler_;
  std::shared_ptr<Hash3DAnchored> scene_field_;
  std::shared_ptr<SHShader> shader_;

  Tensor app_emb_;
};

#endif  // NERF__RENDERER_HPP_
