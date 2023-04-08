// Copyright 2023 Tier IV, Inc.
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

#ifndef CUDA_UTILS__TENSORRT_PLUGIN_UTILS_HPP_
#define CUDA_UTILS__TENSORRT_PLUGIN_UTILS_HPP_

namespace cuda_utils
{
constexpr size_t CUDA_ALIGN = 256;

template <typename T>
inline size_t get_size_aligned(size_t num_elem)
{
  size_t size = num_elem * sizeof(T);
  size_t extra_align = 0;
  if (size % CUDA_ALIGN != 0) {
    extra_align = CUDA_ALIGN - size % CUDA_ALIGN;
  }
  return size + extra_align;
}

template <typename T>
inline T * get_next_ptr(size_t num_elem, void *& workspace, size_t & workspace_size)
{
  size_t size = get_size_aligned<T>(num_elem);
  if (size > workspace_size) {
    throw std::runtime_error("Workspace is too small!");
  }
  workspace_size -= size;
  T * ptr = reinterpret_cast<T *>(workspace);
  workspace = reinterpret_cast<void *>(reinterpret_cast<uintptr_t>(workspace) + size);
  return ptr;
}
}  // namespace cuda_utils

#endif  // CUDA_UTILS__TENSORRT_PLUGIN_UTILS_HPP_
