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

#ifndef NMS_KERNEL_HPP_
#define NMS_KERNEL_HPP_

#include <postprocess_kernel.hpp>

#include <cstdint>

namespace centerpoint
{
cudaError_t circleNMS_launch(
  const thrust::device_vector<Box3D> & boxes3d, const int num_boxes3d,
  const float distance_threshold, uint64_t * mask);

// TODO(yukke42): add description
int circleNMS(
  thrust::device_vector<Box3D> & boxes3d, thrust::device_vector<bool> & keep_mask,
  const float distance_threshold);

}  // namespace centerpoint

#endif  // NMS_KERNEL_HPP_
