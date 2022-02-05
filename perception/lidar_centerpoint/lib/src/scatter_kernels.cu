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

#include <scatter_kernels.hpp>
#include <stdio.h>

namespace centerpoint
{
__global__ void scatterFeatures_kernel(
  const float * pillar_features, const int * coords, const size_t num_pillars, const int num_pillar_feature,
  const int grid_size_x, const int grid_size_y, float * scattered_features
)
{
    // pillar_features: shape of (max_num_pillars, num_pillar_features)
    // coords: shape of (max_num_pillars, 3)
    // scattered_features: shape of (num_pillars, grid_size_y, grid_size_x)
    int pillar_i = blockIdx.x;
    int feature_i = threadIdx.x;
    int3 coord = ((int3*)coords)[pillar_i]; // (zyx)

    if (coord.x < 0) { return; }

    float features = pillar_features[num_pillar_feature * pillar_i + feature_i];
    scattered_features[grid_size_y * grid_size_x * feature_i + grid_size_x * coord.y + coord.z] = features;
}


cudaError_t scatterFeatures_launch(
  const float * pillar_features, const int * coords, const size_t num_pillars,
  const int max_num_pillar, const int num_pillar_feature, const int grid_size_x,
  const int grid_size_y, float * scattered_features, cudaStream_t stream)
{
    dim3 blocks(max_num_pillar);
    dim3 threads(num_pillar_feature);
    scatterFeatures_kernel<<<blocks, threads, 0, stream>>>(
        pillar_features, coords, num_pillars, num_pillar_feature, grid_size_x,
        grid_size_y, scattered_features);
    cudaError_t err = cudaGetLastError();
    return err;
}

}  // namespace centerpoint
