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
// Created by ppwang on 2023/3/17.
//

#include "CustomOps.hpp"
#include "../common.hpp"
#include "../common_cuda.hpp"

#define SCALE (16.f)

using Tensor = torch::Tensor;

__global__ void WeightVarLossForwardKernel(int n_outs, float* weights, int* idx_start_end, float* out_vars) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n_outs) return;
  int idx_start = idx_start_end[idx * 2];
  int idx_end   = idx_start_end[idx * 2 + 1];
  if (idx_start >= idx_end) {
    out_vars[idx] = 0.f;
    return;
  }
  float mean = 0.f;
  float weight_sum = 1e-6f;
  float len = SCALE;
  for (int i = 0; i + idx_start < idx_end; i++) {
    mean += weights[i + idx_start] * (float(i) / len);
    weight_sum += weights[i + idx_start];
  }
  mean /= weight_sum;
  float variance = 0.f;
  for (int i = 0; i + idx_start < idx_end; i++) {
    float bias = float(i) / len - mean;
    variance += weights[i + idx_start] * bias * bias;
  }
  out_vars[idx] = variance;
}


__global__ void WeightVarLossBackwardKernel(int n_outs, float* weights, int* idx_start_end, float* dl_dvars, float* dl_dw) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n_outs) return;
  int idx_start = idx_start_end[idx * 2];
  int idx_end   = idx_start_end[idx * 2 + 1];
  if (idx_start >= idx_end) {
    return;
  }
  float mean = 0.f;
  float weight_sum = 1e-6f;
  float len = SCALE;
  for (int i = 0; i + idx_start < idx_end; i++) {
    mean += weights[i + idx_start] * (float(i) / len);
    weight_sum += weights[i + idx_start];
  }
  mean /= weight_sum;
  float variance = 0.f;
  float tmp = 0.f;
  for (int i = 0; i + idx_start < idx_end; i++) {
    float bias = float(i) / len - mean;
    variance += weights[i + idx_start] * bias * bias;
    tmp += weights[i + idx_start] * 2.f * bias;
  }
  for (int i = 0; i + idx_start < idx_end; i++) {
    float bias = float(i) / len - mean;
    float grad = (bias * bias + tmp * -(float(i) / len) / weight_sum);
    dl_dw[i + idx_start] = dl_dvars[idx] * grad;
  }
}

namespace torch::autograd {

class WeightVarLoss : public Function<WeightVarLoss> {
public:
  static variable_list forward(AutogradContext *ctx,
                               Tensor weights,
                               Tensor idx_start_end) {
    CHECK(weights.is_contiguous());
    CHECK(idx_start_end.is_contiguous());
    int n_outs = idx_start_end.size(0);
    Tensor out_vars = torch::empty({ n_outs }, CUDAFloat);
    dim3 grid_dim  = LIN_GRID_DIM(n_outs);
    dim3 block_dim = LIN_BLOCK_DIM;
    WeightVarLossForwardKernel<<<grid_dim, block_dim>>>(n_outs,
                                                        weights.data_ptr<float>(),
                                                        idx_start_end.data_ptr<int>(),
                                                        out_vars.data_ptr<float>());
    ctx->save_for_backward({ weights, idx_start_end });
    return { out_vars };
  }

  static variable_list backward(AutogradContext *ctx,
                                variable_list grad_output) {
    Tensor dl_dvar = grad_output[0].contiguous();
    auto saved_tensors = ctx->get_saved_variables();
    Tensor& weights = saved_tensors[0];
    Tensor& idx_start_end = saved_tensors[1];

    int n_outs = idx_start_end.size(0);
    int n_all  = weights.size(0);

    Tensor dl_dw = torch::empty({ n_all }, CUDAFloat);
    dim3 grid_dim  = LIN_GRID_DIM(n_outs);
    dim3 block_dim = LIN_BLOCK_DIM;

    WeightVarLossBackwardKernel<<<grid_dim, block_dim>>>(n_outs,
                                                         weights.data_ptr<float>(),
                                                         idx_start_end.data_ptr<int>(),
                                                         dl_dvar.data_ptr<float>(),
                                                         dl_dw.data_ptr<float>());

    return { dl_dw, Tensor() };
  }
};

}

Tensor CustomOps::WeightVar(Tensor weights, Tensor idx_start_end) {
  return torch::autograd::WeightVarLoss::apply(weights.contiguous(), idx_start_end.contiguous())[0];
}
