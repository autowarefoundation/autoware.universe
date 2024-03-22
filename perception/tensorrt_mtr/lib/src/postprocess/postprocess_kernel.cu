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

#include "postprocess/postprocess_kernel.cuh"

__global__ void transformTrajectoryKernel(
  const int B, const int M, const int T, const int inDim, const float * targetState,
  const int outDim, float * trajectory)
{
  int b = blockIdx.x * blockDim.x + threadIdx.x;
  int m = blockIdx.y * blockDim.y + threadIdx.y;
  int t = blockIdx.z * blockDim.z + threadIdx.z;

  if (b >= B || m >= M || t >= T) {
    return;
  }

  const int pred_idx = (b * M * T + m * T + t) * outDim;
  const float pred_x = trajectory[pred_idx];
  const float pred_y = trajectory[pred_idx + 1];

  const int target_idx = b * inDim;
  const float target_x = targetState[target_idx];
  const float target_y = targetState[target_idx + 1];
  const float target_yaw = targetState[target_idx + 6];
  const float target_cos = cos(target_yaw);
  const float target_sin = sin(target_yaw);

  trajectory[pred_idx] = target_cos * pred_x + target_sin * pred_y + target_x;
  trajectory[pred_idx + 1] = -target_sin * pred_x + target_cos * pred_y + target_y;
}

cudaError_t postprocessLauncher(
  const int B, const int M, const int T, const int inDim, const float * target_state,
  const int outDim, float * pred_score, float * pred_trajectory, cudaStream_t stream)
{
  // TODO: update the number of blocks and threads to guard from `cudaErrorIllegalAccess`
  constexpr int threadsPerBlock = 256;
  dim3 blocks(B, M, T);

  transformTrajectoryKernel<<<blocks, threadsPerBlock, 0, stream>>>(
    B, M, T, inDim, target_state, outDim, pred_trajectory);

  return cudaGetLastError();
}
