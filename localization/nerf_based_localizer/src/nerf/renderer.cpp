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
// This file is derived from the following file.
// https://github.com/Totoro97/f2-nerf/blob/main/src/Renderer/Renderer.cpp
//
// Created by ppwang on 2022/5/7.
//

#include "renderer.hpp"

#include "CustomOps/CustomOps.hpp"
#include "CustomOps/FlexOps.hpp"
#include "CustomOps/Scatter.hpp"
#include "common.hpp"
#include "rays.hpp"
#include "stop_watch.hpp"
#include "utils.hpp"

using Tensor = torch::Tensor;
namespace F = torch::nn::functional;

Renderer::Renderer(int n_images, const int sample_num_per_ray)
: sample_num_per_ray_(sample_num_per_ray)
{
  pts_sampler_ = std::make_shared<PtsSampler>(sample_num_per_ray_);

  scene_field_ = std::make_shared<Hash3DAnchored>();
  register_module("scene_field", scene_field_);

  shader_ = std::make_shared<SHShader>();
  register_module("shader", shader_);

  app_emb_ = torch::randn({n_images, 16}, CUDAFloat) * .1f;
  app_emb_.requires_grad_(true);
  register_parameter("app_emb", app_emb_);
}

RenderResult Renderer::render(
  const Tensor & rays_o, const Tensor & rays_d, const Tensor & emb_idx, RunningMode mode)
{
  int n_rays = rays_o.sizes()[0];
  SampleResultFlex sample_result = pts_sampler_->get_samples(rays_o, rays_d, mode);
  int n_all_pts = sample_result.pts.sizes()[0];
  CHECK(sample_result.pts_idx_bounds.max().item<int>() <= n_all_pts);
  CHECK(sample_result.pts_idx_bounds.min().item<int>() >= 0);

  Tensor bg_color =
    ((mode == RunningMode::TRAIN) ? torch::rand({n_rays, 3}, CUDAFloat)
                                  : torch::ones({n_rays, 3}, CUDAFloat) * .5f);

  if (n_all_pts <= 0) {
    return {
      bg_color, torch::zeros({n_rays}, CUDAFloat), torch::full({n_rays}, 512.f, CUDAFloat),
      Tensor()};
  }
  CHECK(rays_o.sizes()[0] == sample_result.pts_idx_bounds.sizes()[0]);

  auto DensityAct = [](Tensor x) -> Tensor {
    const float shift = 3.f;
    return torch::autograd::TruncExp::apply(x - shift)[0];
  };

  // First, inference - early stop
  SampleResultFlex sample_result_early_stop;
  {
    Tensor scene_feat = scene_field_->query(sample_result.pts);
    Tensor sampled_density = DensityAct(scene_feat.index({Slc(), Slc(0, 1)}));
    Tensor sec_density = sampled_density.index({Slc(), 0}) * sample_result.dt;
    Tensor alphas = 1.f - torch::exp(-sec_density);
    Tensor acc_density = FlexOps::AccumulateSum(sec_density, sample_result.pts_idx_bounds, false);
    Tensor trans = torch::exp(-acc_density);
    Tensor weights = trans * alphas;
    Tensor mask = trans > 1e-4f;
    Tensor mask_idx = torch::where(mask)[0];

    sample_result_early_stop.pts = sample_result.pts.index({mask_idx}).contiguous();
    sample_result_early_stop.dirs = sample_result.dirs.index({mask_idx}).contiguous();
    sample_result_early_stop.dt = sample_result.dt.index({mask_idx}).contiguous();
    sample_result_early_stop.t = sample_result.t.index({mask_idx}).contiguous();

    Tensor mask_2d = mask.reshape({n_rays, sample_num_per_ray_});
    Tensor num = mask_2d.sum(1);
    Tensor cum_num = torch::cumsum(num, 0);
    Tensor idx_bounds =
      torch::zeros({n_rays, 2}, torch::TensorOptions().dtype(torch::kInt).device(torch::kCUDA));
    idx_bounds.index_put_({Slc(), 0}, cum_num - num);
    idx_bounds.index_put_({Slc(), 1}, cum_num);
    sample_result_early_stop.pts_idx_bounds = idx_bounds;

    CHECK(
      sample_result_early_stop.pts_idx_bounds.max().item<int>() ==
      sample_result_early_stop.pts.size(0));
  }

  n_all_pts = sample_result_early_stop.pts.size(0);

  Tensor scene_feat = scene_field_->query(sample_result_early_stop.pts);
  Tensor sampled_density = DensityAct(scene_feat.index({Slc(), Slc(0, 1)}));

  Tensor shading_feat = torch::cat(
    {torch::ones_like(scene_feat.index({Slc(), Slc(0, 1)}), CUDAFloat),
     scene_feat.index({Slc(), Slc(1, torch::indexing::None)})},
    1);

  if (mode == RunningMode::TRAIN) {
    Tensor all_emb_idx =
      CustomOps::ScatterIdx(n_all_pts, sample_result_early_stop.pts_idx_bounds, emb_idx);
    shading_feat = CustomOps::ScatterAdd(app_emb_, all_emb_idx, shading_feat);
  }

  Tensor sampled_colors = shader_->query(shading_feat, sample_result_early_stop.dirs);
  Tensor sampled_t = (sample_result_early_stop.t + 1e-2f).contiguous();
  Tensor sec_density = sampled_density.index({Slc(), 0}) * sample_result_early_stop.dt;
  Tensor alphas = 1.f - torch::exp(-sec_density);
  Tensor idx_start_end = sample_result_early_stop.pts_idx_bounds;
  Tensor acc_density = FlexOps::AccumulateSum(sec_density, idx_start_end, false);
  Tensor trans = torch::exp(-acc_density);
  Tensor weights = trans * alphas;

  Tensor last_trans = torch::exp(-FlexOps::Sum(sec_density, idx_start_end));
  Tensor colors = FlexOps::Sum(weights.unsqueeze(-1) * sampled_colors, idx_start_end);
  colors = colors + last_trans.unsqueeze(-1) * bg_color;
  Tensor depths = FlexOps::Sum(weights * sampled_t, idx_start_end) / (1.f - last_trans + 1e-4f);

  CHECK(std::isfinite((colors).mean().item<float>()));

  return {colors, depths, weights, idx_start_end};
}

std::tuple<Tensor, Tensor> Renderer::render_all_rays(
  const Tensor & rays_o, const Tensor & rays_d, const int batch_size)
{
  const int n_rays = rays_d.sizes()[0];

  std::vector<Tensor> pred_colors;
  std::vector<Tensor> pred_depths;

  const int ray_batch_size = (1 << 16);
  for (int i = 0; i < n_rays; i += batch_size) {
    int i_high = std::min(i + batch_size, n_rays);
    Tensor cur_rays_o = rays_o.index({Slc(i, i_high)}).contiguous();
    Tensor cur_rays_d = rays_d.index({Slc(i, i_high)}).contiguous();

    RenderResult render_result = render(cur_rays_o, cur_rays_d, Tensor(), RunningMode::VALIDATE);
    Tensor colors = render_result.colors;
    Tensor depths = render_result.depths.squeeze();

    pred_colors.push_back(colors);
    pred_depths.push_back(depths.unsqueeze(-1));
  }

  Tensor pred_colors_ts = torch::cat(pred_colors, 0);
  Tensor pred_depths_ts = torch::cat(pred_depths, 0);

  return {pred_colors_ts, pred_depths_ts};
}

std::tuple<Tensor, Tensor> Renderer::render_image(
  const torch::Tensor & pose, const torch::Tensor & intrinsic, const int h, const int w,
  const int batch_size)
{
  Tensor ii = torch::linspace(0.f, h - 1.f, h, CUDAFloat);
  Tensor jj = torch::linspace(0.f, w - 1.f, w, CUDAFloat);
  auto ij = torch::meshgrid({ii, jj}, "ij");
  Tensor i = ij[0].reshape({-1});
  Tensor j = ij[1].reshape({-1});
  auto [rays_o, rays_d] =
    get_rays_from_pose(pose.unsqueeze(0), intrinsic.unsqueeze(0), torch::stack({i, j}, -1));
  auto [pred_colors, pred_depths] = render_all_rays(rays_o, rays_d, batch_size);
  pred_colors = pred_colors.reshape({h, w, 3});
  pred_depths = pred_depths.reshape({h, w, 1});

  pred_colors = pred_colors.clip(0.0f, 1.0f);
  pred_depths = pred_depths.repeat({1, 1, 3});

  return {pred_colors, pred_depths};
}

std::vector<torch::optim::OptimizerParamGroup> Renderer::optim_param_groups(float lr)
{
  std::vector<torch::optim::OptimizerParamGroup> ret;

  // scene_field_
  for (const auto & para_group : scene_field_->optim_param_groups(lr)) {
    ret.emplace_back(para_group);
  }

  // shader_
  for (const auto & para_group : shader_->optim_param_groups(lr)) {
    ret.emplace_back(para_group);
  }

  // app_emb_
  auto opt = std::make_unique<torch::optim::AdamOptions>(lr);
  opt->betas() = {0.9, 0.99};
  opt->eps() = 1e-15;
  opt->weight_decay() = 1e-6;
  std::vector<Tensor> params{app_emb_};
  ret.emplace_back(std::move(params), std::move(opt));

  return ret;
}
