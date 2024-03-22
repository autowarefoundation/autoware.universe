// Copyright 2024 TIER IV, Inc.
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

#include "preprocess/polyline_preprocess_kernel.cuh"

#include <iostream>

__global__ void transformPolylineKernel(
  const int K, const int P, const int PointDim, const float * in_polyline, const int B,
  const int AgentDim, const float * target_state, float * out_polyline, bool * out_polyline_mask)
{
  int b = blockIdx.x * blockDim.x + threadIdx.x;
  int k = blockIdx.y * blockDim.y + threadIdx.y;
  int p = blockIdx.z * blockDim.z + threadIdx.z;

  if (b >= B || k >= K || p >= P) {
    return;
  }

  const int src_polyline_idx = (k * P + p) * PointDim;
  const float x = in_polyline[src_polyline_idx];
  const float y = in_polyline[src_polyline_idx + 1];
  const float z = in_polyline[src_polyline_idx + 2];
  const float dx = in_polyline[src_polyline_idx + 3];
  const float dy = in_polyline[src_polyline_idx + 4];
  const float dz = in_polyline[src_polyline_idx + 5];
  const float type_id = in_polyline[src_polyline_idx + 6];

  const int center_idx = b * AgentDim;
  const float center_x = target_state[center_idx];
  const float center_y = target_state[center_idx + 1];
  const float center_z = target_state[center_idx + 2];
  const float center_yaw = target_state[center_idx + 6];
  const float center_cos = cos(center_yaw);
  const float center_sin = sin(center_yaw);

  // do transform
  const float trans_x = center_cos * (x - center_x) - center_sin * (y - center_y);
  const float trans_y = center_sin * (x - center_x) + center_cos * (y - center_y);
  const float trans_z = z - center_z;
  const float trans_dx = center_cos * dx - center_sin * dy;
  const float trans_dy = center_sin * dx + center_cos * dy;
  const float trans_dz = dz;

  const int out_idx = (b * K * P + k * P + p) * (PointDim + 2);
  out_polyline[out_idx] = trans_x;
  out_polyline[out_idx + 1] = trans_y;
  out_polyline[out_idx + 2] = trans_z;
  out_polyline[out_idx + 3] = trans_dx;
  out_polyline[out_idx + 4] = trans_dy;
  out_polyline[out_idx + 5] = trans_dz;
  out_polyline[out_idx + 6] = type_id;

  const int out_mask_idx = b * K * P + k * P + p;
  bool is_valid = false;
  for (size_t i = 0; i < 6; ++i) {
    is_valid += out_polyline[out_idx + i] != 0.0f;
  }
  out_polyline_mask[out_mask_idx] = is_valid;
}

__global__ void setPreviousPositionKernel(
  const int B, const int K, const int P, const int D, const bool * mask, float * polyline)
{
  int b = blockIdx.x * blockDim.x + threadIdx.x;
  int k = blockIdx.y * blockDim.y + threadIdx.y;
  int p = blockIdx.z * blockDim.z + threadIdx.z;

  if (b >= B || k >= K || p >= P) {
    return;
  }

  const int cur_idx = (b * K * P + k * P + p) * D;
  const int pre_idx = k == 0 ? cur_idx : (b * K * P + (k - 1) * P + p) * D;

  polyline[cur_idx + D - 2] = polyline[pre_idx];
  polyline[cur_idx + D - 1] = polyline[pre_idx + 1];

  const int mask_idx = b * K * P + k * P + p;
  if (!mask[mask_idx]) {
    for (int d = 0; d < D; ++d) {
      polyline[cur_idx + d] = 0.0f;
    }
  }
}

__global__ void extractTopkKernel(
  const int K, const int L, const int P, const int B, const float offsetX, const float offsetY,
  const int AgentDim, const float * targetState, const int PointDim, const float * inPolyline,
  float * outPolyline)
{
  // --- pseudo code ---
  // mask = All(polyline != 0.0, dim=2)
  // polylineCenter = polyline[:, :, 0:2].sum(dim=1) / clampMin(mask.sum(dim=1), min=1.0)
  // offset = rotateAlongZ((offset_x, offset_y), target_state[:, 6])
  // targetOffsetPos = target_state[:, 0:2] + offset
  // distances = (target_offset_pos - center)
  // _, topkIdxs = distances.topk(k=K, descending=True)
  // outPolyline = inPolyline[topkIdxs]
  // -------------------

  // int targetIdx = blockIdx.x;

  // const float targetX = targetState[targetIdx];
  // const float targetY = targetState[targetIdx + 1];
  // const float targetYaw = targetState[targetIdx + 6];
  // const float targetCos = cos(targetYaw);
  // const float targetSin = sin(targetYaw);

  // const float transTargetX = targetCos * offsetX + targetSin * offsetY + targetX;
  // const float transTargetY = -targetSin * offsetX + targetCos * offsetY + targetY;
}

__global__ void calculatePolylineCenterKernel(
  const int B, const int K, const int P, const int PointDim, const float * polyline,
  const bool * mask, float * center)
{
  // --- pseudo code ---
  // sum = (polylines[:, :, :, 0:3] * mask[:, :, :, None]).sum(dim=2)
  // center = sum / clampMIN(mask.sum(dim=2), min=1.0)
  // -------------------

  int b = blockIdx.x * blockDim.x + threadIdx.x;
  int k = blockIdx.y * blockDim.y + threadIdx.y;

  if (b >= B || k >= K) {
    return;
  }

  // initialize with 0.0
  int center_idx = (b * K + k) * 3;
  for (int d = 0; d < 3; ++d) {
    center[center_idx + d] = 0.0f;
  }

  float sum_xyz[3] = {0.0f, 0.0f, 0.0f};
  int count = 0;
  for (int p = 0; p < P; ++p) {
    int src_idx = b * K * P + k * P + p;
    if (mask[src_idx]) {
      for (int d = 0; d < 3; ++d) {
        sum_xyz[d] += polyline[src_idx * PointDim + d];
      }
      ++count;
    }
  }
  count = max(count, 1);

  for (int d = 0; d < 3; ++d) {
    center[center_idx + d] = sum_xyz[d] / static_cast<float>(count);
  }
}

cudaError_t polylinePreprocessWithTopkLauncher(
  const int L, const int K, const int P, const int PointDim, const float * in_polyline, const int B,
  const int AgentDim, const float * target_state, const float offset_x, const float offset_y,
  int * topk_index, float * out_polyline, bool * out_polyline_mask, float * out_polyline_center,
  cudaStream_t stream)
{
  if (L < K) {
    std::cerr << "L must be greater than K, but got L: " << L << ", K: " << K << std::endl;
    return cudaError_t::cudaErrorInvalidValue;
  }

  return cudaGetLastError();
}

cudaError_t polylinePreprocessLauncher(
  const int K, const int P, const int PointDim, const float * in_polyline, const int B,
  const int AgentDim, const float * target_state, float * out_polyline, bool * out_polyline_mask,
  float * out_polyline_center, cudaStream_t stream)
{
  // TODO: update the number of blocks and threads to guard from `cudaErrorIllegalAccess`
  constexpr int threadsPerBlock = 256;
  const dim3 block3d(B, K / threadsPerBlock, P);

  transformPolylineKernel<<<block3d, threadsPerBlock, 0, stream>>>(
    K, P, PointDim, in_polyline, B, AgentDim, target_state, out_polyline, out_polyline_mask);

  setPreviousPositionKernel<<<block3d, threadsPerBlock, 0, stream>>>(
    B, K, P, PointDim, out_polyline_mask, out_polyline);

  const dim3 block2d(B, K / threadsPerBlock);
  calculatePolylineCenterKernel<<<block2d, threadsPerBlock, 0, stream>>>(
    B, K, P, PointDim, out_polyline, out_polyline_mask, out_polyline_center);

  return cudaGetLastError();
}
