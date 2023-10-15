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
// Created by ppwang on 2022/9/26.
//

#include "points_sampler.hpp"

#include "dataset.hpp"
#include "stop_watch.hpp"
#include "utils.hpp"

#include <algorithm>
#include <random>

using Tensor = torch::Tensor;

PtsSampler::PtsSampler()
{
}

SampleResultFlex PtsSampler::get_samples(
  const Tensor & rays_o_raw, const Tensor & rays_d_raw, RunningMode mode)
{
  Tensor rays_o = rays_o_raw.contiguous();
  Tensor rays_d = (rays_d_raw / torch::linalg_norm(rays_d_raw, 2, -1, true)).contiguous();

  int n_rays = rays_o.sizes()[0];

  // do ray marching
  const int n_all_pts = n_rays * MAX_SAMPLE_PER_RAY;

  Tensor rays_noise;
  if (mode == RunningMode::VALIDATE) {
    rays_noise = torch::ones({n_all_pts}, CUDAFloat);
  } else {
    rays_noise = ((torch::rand({n_all_pts}, CUDAFloat) - .5f) + 1.f).contiguous();
  }
  rays_noise = rays_noise.view({n_rays, MAX_SAMPLE_PER_RAY}).contiguous();
  Tensor cum_noise = torch::cumsum(rays_noise, 1) * SAMPLE_L;
  Tensor sampled_t = cum_noise.reshape({n_all_pts}).contiguous();

  rays_o = rays_o.view({n_rays, 1, 3}).contiguous();
  rays_d = rays_d.view({n_rays, 1, 3}).contiguous();
  cum_noise = cum_noise.unsqueeze(-1).contiguous();
  Tensor sampled_pts = rays_o + rays_d * cum_noise;

  Tensor sampled_distances = torch::diff(sampled_pts, 1, 1).norm(2, -1).contiguous();
  sampled_distances =
    torch::cat({torch::zeros({n_rays, 1}, CUDAFloat), sampled_distances}, 1).contiguous();
  sampled_pts = sampled_pts.view({n_all_pts, 3});
  sampled_distances = sampled_distances.view({n_all_pts}).contiguous();

  Tensor pts_idx_start_end =
    torch::ones({n_rays, 2}, torch::TensorOptions().dtype(torch::kInt).device(torch::kCUDA)) *
    MAX_SAMPLE_PER_RAY;
  Tensor pts_num = pts_idx_start_end.index({Slc(), 0});
  Tensor cum_num = torch::cumsum(pts_num, 0);
  pts_idx_start_end.index_put_({Slc(), 0}, cum_num - pts_num);
  pts_idx_start_end.index_put_({Slc(), 1}, cum_num);

  Tensor sampled_dirs =
    rays_d.expand({-1, MAX_SAMPLE_PER_RAY, -1}).reshape({n_all_pts, 3}).contiguous();

  return {sampled_pts, sampled_dirs, sampled_distances, sampled_t, pts_idx_start_end};
}
