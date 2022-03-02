// Copyright 2022 Tier IV, Inc.
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

// Modified from
// https://github.com/open-mmlab/OpenPCDet/blob/master/pcdet/ops/iou3d_nms/src/iou3d_nms_kernel.cu

/*
3D IoU Calculation and Rotated NMS(modified from 2D NMS written by others)
Written by Shaoshuai Shi
All Rights Reserved 2019-2020.
*/

#include <cuda_utils.hpp>
#include <nms_kernel.hpp>

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#define THREADS_PER_BLOCK_NMS 16
#define DIVUP(m, n) ((m) / (n) + ((m) % (n) > 0))

namespace centerpoint
{
__device__ inline float dist2d_pow_kernel(const Box3D * a, const Box3D * b)
{
  return powf(a->x - b->x, 2) + powf(a->y - b->y, 2);
}

__global__ void circleNMS_Kernel(
  const Box3D * boxes, const int num_boxes3d, const float dist2d_pow_threshold, uint64_t * mask)
{
  // params: boxes (N, 12)
  // params: mask (N, N/THREADS_PER_BLOCK_NMS)

  const int row_start = blockIdx.y;
  const int col_start = blockIdx.x;

  if (row_start > col_start) return;

  const int row_size =
    fminf(num_boxes3d - row_start * THREADS_PER_BLOCK_NMS, THREADS_PER_BLOCK_NMS);
  const int col_size =
    fminf(num_boxes3d - col_start * THREADS_PER_BLOCK_NMS, THREADS_PER_BLOCK_NMS);

  __shared__ Box3D block_boxes[THREADS_PER_BLOCK_NMS];

  if (threadIdx.x < col_size) {
    block_boxes[threadIdx.x] = boxes[THREADS_PER_BLOCK_NMS * col_start + threadIdx.x];
  }
  __syncthreads();

  if (threadIdx.x < row_size) {
    const int cur_box_idx = THREADS_PER_BLOCK_NMS * row_start + threadIdx.x;
    const Box3D * cur_box = boxes + cur_box_idx;

    uint64_t t = 0;
    int start = 0;
    if (row_start == col_start) {
      start = threadIdx.x + 1;
    }
    for (int i = start; i < col_size; i++) {
      if (dist2d_pow_kernel(cur_box, block_boxes + i) < dist2d_pow_threshold) {
        t |= 1ULL << i;
      }
    }
    const int col_blocks = DIVUP(num_boxes3d, THREADS_PER_BLOCK_NMS);
    mask[cur_box_idx * col_blocks + col_start] = t;
  }
}

cudaError_t circleNMS_launch(
  const thrust::device_vector<Box3D> & boxes3d, const int num_boxes3d,
  const float distance_threshold, uint64_t * mask)
{
  const float dist2d_pow_thres = powf(distance_threshold, 2);

  dim3 blocks(DIVUP(num_boxes3d, THREADS_PER_BLOCK_NMS), DIVUP(num_boxes3d, THREADS_PER_BLOCK_NMS));
  dim3 threads(THREADS_PER_BLOCK_NMS);
  circleNMS_Kernel<<<blocks, threads>>>(
    thrust::raw_pointer_cast(boxes3d.data()), num_boxes3d, dist2d_pow_thres, mask);

  return cudaGetLastError();
}

int circleNMS(
  thrust::device_vector<Box3D> & boxes3d, thrust::device_vector<bool> & keep_mask,
  const float distance_threshold)
{
  const int num_boxes3d = boxes3d.size();
  const int col_blocks = DIVUP(num_boxes3d, THREADS_PER_BLOCK_NMS);

  // TODO(yukke42): don't use cudaMaollc and cudaFree
  uint64_t * mask_data = NULL;
  CHECK_CUDA_ERROR(cudaMalloc((void **)&mask_data, num_boxes3d * col_blocks * sizeof(uint64_t)));
  CHECK_CUDA_ERROR(circleNMS_launch(boxes3d, num_boxes3d, distance_threshold, mask_data));

  std::vector<uint64_t> mask_cpu(num_boxes3d * col_blocks);
  CHECK_CUDA_ERROR(cudaMemcpy(
    &mask_cpu[0], mask_data, num_boxes3d * col_blocks * sizeof(uint64_t), cudaMemcpyDeviceToHost));
  cudaFree(mask_data);

  uint64_t * remv_cpu = new uint64_t[col_blocks]();
  thrust::host_vector<bool> keep_mask_h(num_boxes3d);
  int num_to_keep = 0;
  for (int i = 0; i < num_boxes3d; i++) {
    int nblock = i / THREADS_PER_BLOCK_NMS;
    int inblock = i % THREADS_PER_BLOCK_NMS;

    if (!(remv_cpu[nblock] & (1ULL << inblock))) {
      // keep_data[num_to_keep++] = i;
      keep_mask_h[i] = true;
      num_to_keep++;
      uint64_t * p = &mask_cpu[0] + i * col_blocks;
      for (int j = nblock; j < col_blocks; j++) {
        remv_cpu[j] |= p[j];
      }
    } else {
      keep_mask_h[i] = false;
    }
  }
  delete[] remv_cpu;

  // host to device
  keep_mask = keep_mask_h;

  return num_to_keep;
}

}  // namespace centerpoint

#undef THREADS_PER_BLOCK
#undef DIVUP
