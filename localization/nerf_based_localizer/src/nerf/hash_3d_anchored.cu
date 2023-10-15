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
#include "common.hpp"
#include "common_cuda.hpp"
#include "hash_3d_anchored.hpp"

#include <Eigen/Eigen>

#include <torch/torch.h>

using Tensor = torch::Tensor;

constexpr float RES_FINE_POW_2 = 10.f;
constexpr float RES_BASE_POW_2 = 3.f;

const auto CUDAFlex = torch::TensorOptions().dtype(torch::kFloat16).device(torch::kCUDA);
using FlexType = __half;

__device__ inline void calculate_pos_and_w(
  const Eigen::Vector3f & pt, const int local_size, const int * const prim_pool,
  const int level_idx, unsigned pos[8], float w[8])
{
  const int offset = level_idx * 3;
  const unsigned pa = prim_pool[offset + 0];
  const unsigned pb = prim_pool[offset + 1];
  const unsigned pc = prim_pool[offset + 2];

  const unsigned pos_x = static_cast<unsigned>(floorf(pt[0]));
  const unsigned pos_y = static_cast<unsigned>(floorf(pt[1]));
  const unsigned pos_z = static_cast<unsigned>(floorf(pt[2]));
  pos[0] = ((pos_x * pa) ^ (pos_y * pb) ^ (pos_z * pc)) % local_size;
  pos[1] = ((pos_x * pa) ^ (pos_y * pb) ^ ((pos_z + 1u) * pc)) % local_size;
  pos[2] = ((pos_x * pa) ^ ((pos_y + 1u) * pb) ^ (pos_z * pc)) % local_size;
  pos[3] = ((pos_x * pa) ^ ((pos_y + 1u) * pb) ^ ((pos_z + 1u) * pc)) % local_size;
  pos[4] = (((pos_x + 1u) * pa) ^ (pos_y * pb) ^ (pos_z * pc)) % local_size;
  pos[5] = (((pos_x + 1u) * pa) ^ (pos_y * pb) ^ ((pos_z + 1u) * pc)) % local_size;
  pos[6] = (((pos_x + 1u) * pa) ^ ((pos_y + 1u) * pb) ^ (pos_z * pc)) % local_size;
  pos[7] = (((pos_x + 1u) * pa) ^ ((pos_y + 1u) * pb) ^ ((pos_z + 1u) * pc)) % local_size;

  const float a = pt[0] - floorf(pt[0]);
  const float b = pt[1] - floorf(pt[1]);
  const float c = pt[2] - floorf(pt[2]);

  w[0] = (1.f - a) * (1.f - b) * (1.f - c);
  w[1] = (1.f - a) * (1.f - b) * c;
  w[2] = (1.f - a) * b * (1.f - c);
  w[3] = (1.f - a) * b * c;
  w[4] = a * (1.f - b) * (1.f - c);
  w[5] = a * (1.f - b) * c;
  w[6] = a * b * (1.f - c);
  w[7] = a * b * c;
}

template <typename T>
__global__ void Hash3DAnchoredForwardKernel(
  int n_points, int local_size, T * feat_pool, int * prim_pool, Eigen::Vector3f * bias_pool,
  Eigen::Vector3f * points_ptr, T * out_feat)
{
  const int pts_idx = blockIdx.x * blockDim.x + threadIdx.x;
  const int level_idx = blockIdx.y;
  if (pts_idx >= n_points) {
    return;
  }
  feat_pool = feat_pool + local_size * level_idx;

  const float mul = exp2f(
    (RES_FINE_POW_2 - RES_BASE_POW_2) * float(level_idx) / float(N_LEVELS - 1) + RES_BASE_POW_2);
  const Eigen::Vector3f pt = (points_ptr[pts_idx] * mul + bias_pool[level_idx]);

  float ws[8] = {};
  unsigned pos[8] = {};
  calculate_pos_and_w(pt, local_size, prim_pool, level_idx, pos, ws);

  out_feat = out_feat + pts_idx * (N_LEVELS * N_CHANNELS);

#pragma unroll
  for (int k = 0; k < N_CHANNELS; k++) {
    out_feat[level_idx * N_CHANNELS + k] = (T)(ws[0] * float(feat_pool[pos[0] * N_CHANNELS + k]) +
                                               ws[1] * float(feat_pool[pos[1] * N_CHANNELS + k]) +
                                               ws[2] * float(feat_pool[pos[2] * N_CHANNELS + k]) +
                                               ws[3] * float(feat_pool[pos[3] * N_CHANNELS + k]) +
                                               ws[4] * float(feat_pool[pos[4] * N_CHANNELS + k]) +
                                               ws[5] * float(feat_pool[pos[5] * N_CHANNELS + k]) +
                                               ws[6] * float(feat_pool[pos[6] * N_CHANNELS + k]) +
                                               ws[7] * float(feat_pool[pos[7] * N_CHANNELS + k]));
  }
}

template <typename T>
__global__ void Hash3DAnchoredBackwardKernel(
  int n_points, int local_size, T * feat_pool, int * prim_pool, Eigen::Vector3f * bias_pool,
  Eigen::Vector3f * points_ptr,
  T * grad_in,      // [ n_points, n_levels, n_channels ]
  T * grad_points,  // [ n_points, 3 ]
  T * grad_embeds   // [ pool_size, n_channels ]
)
{
  const int pts_idx = blockIdx.x * blockDim.x + threadIdx.x;
  const int level_idx = blockIdx.y;
  if (pts_idx >= n_points) {
    return;
  }
  feat_pool = feat_pool + local_size * level_idx;

  const float mul = exp2f(
    (RES_FINE_POW_2 - RES_BASE_POW_2) * float(level_idx) / float(N_LEVELS - 1) + RES_BASE_POW_2);
  const Eigen::Vector3f pt = (points_ptr[pts_idx] * mul + bias_pool[level_idx]);

  float ws[8] = {};
  unsigned pos[8] = {};
  calculate_pos_and_w(pt, local_size, prim_pool, level_idx, pos, ws);

  const float sign_x[8] = {-1.0f, -1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  const float sign_y[8] = {-1.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f, 1.0f, 1.0f};
  const float sign_z[8] = {-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f};

  grad_in = grad_in + (N_LEVELS * N_CHANNELS) * pts_idx + level_idx * N_CHANNELS;

  grad_points = grad_points + pts_idx * 3;
  grad_embeds = grad_embeds + local_size * level_idx;

#pragma unroll
  for (int d = 0; d < 8; d++) {
    for (int k = 0; k < N_CHANNELS; k += 2) {
      float w0 = (float)grad_in[k];
      float w1 = (float)grad_in[k + 1];
      if (w0 != 0.f || w1 != 0.f) {
        __half2 cur_w = {(__half)(float(w0) * ws[d]), (__half)(float(w1) * ws[d])};
        atomicAdd((__half2 *)(grad_embeds + pos[d] * N_CHANNELS + k), cur_w);
      }
    }
    for (int k = 0; k < N_CHANNELS; k++) {
      const float norm = (float)(feat_pool[pos[d] * N_CHANNELS + k]) * mul * (float)grad_in[k];
      atomicAdd(grad_points + 0, (T)(sign_x[d] * norm));
      atomicAdd(grad_points + 1, (T)(sign_y[d] * norm));
      atomicAdd(grad_points + 2, (T)(sign_z[d] * norm));
    }
  }
}

namespace torch::autograd
{

variable_list Hash3DAnchoredFunction::forward(
  AutogradContext * ctx, Tensor points, Tensor feat_pool, IValue hash3d_info)
{
  auto info_ptr = hash3d_info.toCustomClass<Hash3DAnchoredInfo>();
  ctx->saved_data["hash3d_info"] = hash3d_info;
  ctx->saved_data["points"] = points;
  ctx->saved_data["feat_pool"] = feat_pool;
  Tensor & prim_pool = info_ptr->hash3d_->prim_pool_;
  Tensor & bias_pool = info_ptr->hash3d_->bias_pool_;
  CHECK(points.device().is_cuda());

  int n_points = points.sizes()[0];

  dim3 block_dim = LIN_BLOCK_DIM;
  dim3 grid_dim = {DivUp(n_points, THREAD_CAP), unsigned(N_LEVELS), 1};

  Tensor out_feat = torch::zeros({n_points, N_LEVELS * N_CHANNELS}, CUDAFlex);
  CHECK(out_feat.is_contiguous());

  Tensor feat_pool_true = feat_pool.to(torch::kFloat16).contiguous();

  Hash3DAnchoredForwardKernel<FlexType><<<grid_dim, block_dim>>>(
    n_points, info_ptr->hash3d_->local_size_,
    reinterpret_cast<FlexType *>(feat_pool_true.data_ptr()), prim_pool.data_ptr<int>(),
    reinterpret_cast<Eigen::Vector3f *>(bias_pool.data_ptr()),
    reinterpret_cast<Eigen::Vector3f *>(points.data_ptr()),
    reinterpret_cast<FlexType *>(out_feat.data_ptr()));

  return {out_feat.to(torch::kFloat32)};
}

variable_list Hash3DAnchoredFunction::backward(AutogradContext * ctx, variable_list grad_output)
{
  auto info_ptr = ctx->saved_data["hash3d_info"].toCustomClass<Hash3DAnchoredInfo>();
  Tensor & points = ctx->saved_data["points"].toTensor();  // [ n_points, 3 ]
  Tensor & feat_pool = ctx->saved_data["feat_pool"].toTensor();
  Tensor & prim_pool = info_ptr->hash3d_->prim_pool_;
  Tensor & bias_pool = info_ptr->hash3d_->bias_pool_;
  CHECK(points.device().is_cuda());

  const float grad_scale = 128.f;
  int n_points = points.sizes()[0];

  int pool_size = info_ptr->hash3d_->pool_size_;

  dim3 block_dim = LIN_BLOCK_DIM;
  dim3 grid_dim = {DivUp(n_points, THREAD_CAP), unsigned(N_LEVELS), 1};

  Tensor feat_pool_true = feat_pool.to(torch::kFloat16).contiguous();

  Tensor grad_in = (grad_output[0] * grad_scale).to(torch::kFloat16).contiguous();

  Tensor points_grad = torch::zeros({n_points, 3}, CUDAFlex);
  Tensor embeds_grad = torch::zeros({pool_size, N_CHANNELS}, CUDAFlex);

  Hash3DAnchoredBackwardKernel<FlexType><<<grid_dim, block_dim>>>(
    n_points, info_ptr->hash3d_->local_size_,
    reinterpret_cast<FlexType *>(feat_pool_true.data_ptr()), prim_pool.data_ptr<int>(),
    reinterpret_cast<Eigen::Vector3f *>(bias_pool.data_ptr()),
    reinterpret_cast<Eigen::Vector3f *>(points.data_ptr()),
    reinterpret_cast<FlexType *>(grad_in.data_ptr()),
    reinterpret_cast<FlexType *>(points_grad.data_ptr()),
    reinterpret_cast<FlexType *>(embeds_grad.data_ptr()));

  points_grad = points_grad.to(torch::kFloat32) / grad_scale;
  embeds_grad = embeds_grad.to(torch::kFloat32) / grad_scale;

  return {points_grad, embeds_grad, Tensor()};
}

}  // namespace torch::autograd
