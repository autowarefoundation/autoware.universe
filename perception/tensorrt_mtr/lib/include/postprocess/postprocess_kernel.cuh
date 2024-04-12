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

#ifndef POSTPROCESS__POSTPROCESS_KERNEL_CUH__
#define POSTPROCESS__POSTPROCESS_KERNEL_CUH__

#include <cuda_runtime.h>

/**
 * @brief A kernel to transform predicted trajectory from each target coords system to world coords
 * system.
 *
 * @param B The number of target agents.
 * @param M The number of modes.
 * @param T The number of future timestamps.
 * @param inDim The number of input agent state dimensions.
 * @param targetState Source target agent states at latest timestamp, in shape [B*inDim].
 * @param outDim The number of output state dimensions.
 * @param trajectory Output predicted trajectory, in shape [B*M*T*outDim].
 * @return __global__
 */
__global__ void transformTrajectoryKernel(
  const int B, const int M, const int T, const int inDim, const float * targetState,
  const int outDim, float * trajectory);

/**
 * @brief Execute postprocess to predicted score and trajectory.
 *
 * @param B The number of target agents.
 * @param M The number of modes.
 * @param T The number of future timestamps.
 * @param inDim The number of input agent state dimensions.
 * @param target_state Target agent states at the latest timestamp, in shape [B * inDim].
 * @param outDim The number predicted agent state dimensions
 * @param pred_scores Predicted scores, in shape [B*M].
 * @param pred_trajectory Predicted trajectories, in shape [B*M*T*D].
 * @param stream CUDA stream.
 * @return cudaError_t
 */
cudaError_t postprocessLauncher(
  const int B, const int M, const int T, const int inDim, const float * target_state,
  const int outDim, float * pred_score, float * pred_trajectory, cudaStream_t stream);

#endif  // POSTPROCESS__POSTPROCESS_KERNEL_CUH__
