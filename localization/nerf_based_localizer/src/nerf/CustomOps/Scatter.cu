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

#include "../common_cuda.hpp"
#include "Scatter.hpp"

using Tensor = torch::Tensor;


__global__ void ScatterAddFuncForward(int n_all, int n_channels, float* emb, int* scatter_idx, float* to_add) {
  int pts_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int c_idx = blockIdx.y;
  if (pts_idx >= n_all || c_idx >= n_channels) return;
  int emb_idx = scatter_idx[pts_idx];
  to_add = to_add + pts_idx * n_channels + c_idx;
  emb = emb + emb_idx * n_channels + c_idx;
  to_add[0] += emb[0];
}

__global__ void ScatterAddFuncBackwardBlock(int n_emb, int n_blocks, int n_all,
                                            int block_size, int n_channels,
                                            float* dl_demb_pool, int* scatter_idx, float* dl_dsum) {
  int block_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int emb_idx = blockIdx.y;
  if (emb_idx >= n_emb || block_idx >= n_blocks || block_idx * block_size >= n_all) return;
  scatter_idx = scatter_idx + block_idx * block_size;
  dl_dsum = dl_dsum + (block_idx * block_size) * n_channels;
  dl_demb_pool = dl_demb_pool + emb_idx * n_blocks * n_channels + block_idx * n_channels;
  int block_min = block_size;
  if (block_idx * block_size + block_size > n_all) {
    block_min = n_all - (block_idx * block_size);
  }
  for (int i = 0; i < block_min; i++) {
    if (scatter_idx[i] == emb_idx) {
      for (int c = 0; c < n_channels; c++) {
        dl_demb_pool[c] += dl_dsum[i * n_channels + c];
      }
    }
  }
}

namespace torch::autograd {

class ScatterAddFunc : public Function<ScatterAddFunc> {
public:
  static variable_list forward(AutogradContext *ctx,
                               Tensor emb,
                               Tensor idx,
                               Tensor to_add) {
    emb = emb.contiguous();
    idx = idx.contiguous();
    int n_all = idx.size(0);
    int n_channels = emb.size(1);
    CHECK(n_all == to_add.size(0));
    CHECK(n_channels == to_add.size(1));

    const unsigned thread_cap = 512;
    dim3 grid_dim  = { unsigned(n_all + thread_cap - 1) / thread_cap, unsigned(n_channels), 1 };
    dim3 block_dim = { unsigned(thread_cap), 1, 1 };


    Tensor sum = to_add.clone().contiguous();
    ScatterAddFuncForward<<<grid_dim, block_dim>>>(n_all, n_channels,
                                                   emb.data_ptr<float>(),
                                                   idx.data_ptr<int>(),
                                                   sum.data_ptr<float>());
    ctx->save_for_backward({ emb, idx });
    return { sum };
  }

  static variable_list backward(AutogradContext *ctx,
                                variable_list grad_output) {
    Tensor dl_dsum = grad_output[0].contiguous();
    auto saved_tensors = ctx->get_saved_variables();
    Tensor &emb = saved_tensors[0];
    Tensor &idx = saved_tensors[1];

    int n_all = idx.size(0);
    int n_channels = dl_dsum.size(1);

    int n_emb = emb.size(0);
    int block_size = (int(std::sqrt(n_all + 1024)) >> 5) << 5;
    int n_blocks = (n_all + block_size - 1) / block_size;

    const unsigned thread_cap = 512;
    dim3 grid_dim  = { unsigned(n_blocks + thread_cap - 1) / thread_cap, unsigned(n_emb), 1 };
    dim3 block_dim = { unsigned(thread_cap), 1, 1 };

    Tensor dl_demb_pool = torch::zeros({ n_emb, n_blocks, n_channels }, CUDAFloat);
    ScatterAddFuncBackwardBlock<<<grid_dim, block_dim>>>(n_emb, n_blocks, n_all,
                                                         block_size, n_channels,
                                                         dl_demb_pool.data_ptr<float>(),
                                                         idx.data_ptr<int>(),
                                                         dl_dsum.data_ptr<float>());

    Tensor dl_demb = torch::sum(dl_demb_pool, 1, false);
    Tensor dl_dto_add = dl_dsum.clone();

    return { dl_demb, Tensor(), dl_dto_add };
  }
};

}

Tensor CustomOps::ScatterAdd(torch::Tensor emb, torch::Tensor idx, torch::Tensor to_add) {
  return torch::autograd::ScatterAddFunc::apply(emb, idx, to_add)[0];
}


__global__ void ScatterIdxKernal(int n_rays, int* idx_start_end, int* emb_idx, int* all_emb_idx) {
  int ray_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (ray_idx >= n_rays) return;
  int idx_start = idx_start_end[ray_idx * 2];
  int idx_end = idx_start_end[ray_idx * 2 + 1];

  int fill = emb_idx[ray_idx];
  for (int i = idx_start; i < idx_end; i++) {
    all_emb_idx[i] = fill;
  }
}

Tensor CustomOps::ScatterIdx(int n_all_pts, Tensor idx_start_end, Tensor emb_idx) {
  Tensor ret = torch::empty({ n_all_pts }, torch::TensorOptions().dtype(torch::kInt).device(torch::kCUDA));
  int n_rays = idx_start_end.size(0);
  dim3 grid_dim = LIN_GRID_DIM(n_rays);
  dim3 block_dim = LIN_BLOCK_DIM;

  ScatterIdxKernal<<<grid_dim, block_dim>>>(n_rays, idx_start_end.data_ptr<int>(), emb_idx.data_ptr<int>(), ret.data_ptr<int>());

  return ret;
}
