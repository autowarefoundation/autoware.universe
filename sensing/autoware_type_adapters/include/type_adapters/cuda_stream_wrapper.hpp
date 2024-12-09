// Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef TYPE_ADAPTERS__CUDA_STREAM_WRAPPER_HPP_
#define TYPE_ADAPTERS__CUDA_STREAM_WRAPPER_HPP_

#include "cuda.h"          // NOLINT
#include "cuda_runtime.h"  // NOLINT
#include "rclcpp/type_adapter.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <memory>
#include <string>

namespace autoware::type_adaptation::type_adapters
{
template <typename T>
struct NotNull
{
  NotNull(const T * pointer_in, const char * msg) : pointer(pointer_in)
  {
    if (pointer == nullptr) {
      throw std::invalid_argument(msg);
    }
  }

  const T * pointer;
};
/**
 * @brief Wrapper class for CUDA stream management
 *
 * Handles creation and destruction of CUDA streams for asynchronous operations
 */
class CUDAStreamWrapper final
{
public:
  CUDAStreamWrapper();
  ~CUDAStreamWrapper();

  /// @brief Get reference to the underlying CUDA stream
  cudaStream_t & stream() { return main_stream_; }

private:
  cudaStream_t main_stream_{};  ///< The main CUDA stream handle
};

/**
 * @brief Wrapper class for CUDA memory allocation and management
 *
 * Handles allocation and deallocation of CUDA device memory and provides
 * methods for memory transfers between host and device
 */
class CUDAMemoryWrapper final
{
public:
  explicit CUDAMemoryWrapper(size_t size_in_bytes);
  ~CUDAMemoryWrapper();

  /// @brief Copy data from host to device memory
  /// @param host_mem Pointer to host memory
  /// @param bytes_to_copy Number of bytes to copy
  /// @param stream CUDA stream for asynchronous operation
  void copy_to_device(uint8_t * host_mem, size_t bytes_to_copy, const cudaStream_t & stream);

  /// @brief Copy data from device to host memory
  /// @param host_mem Pointer to host memory
  /// @param bytes_to_copy Number of bytes to copy
  /// @param stream CUDA stream for asynchronous operation
  void copy_from_device(uint8_t * host_mem, size_t bytes_to_copy, const cudaStream_t & stream);

  /// @brief Get pointer to device memory
  uint8_t * device_memory();

private:
  size_t bytes_allocated_{0};    ///< Size of allocated memory in bytes
  uint8_t * cuda_mem_{nullptr};  ///< Pointer to allocated CUDA memory
};

/**
 * @brief Wrapper class for CUDA events
 *
 * Manages CUDA events for synchronization and timing of CUDA operations
 */
class CUDAEventWrapper final
{
public:
  CUDAEventWrapper();
  ~CUDAEventWrapper();

  /// @brief Record an event in the specified CUDA stream
  void record(std::shared_ptr<CUDAStreamWrapper> cuda_stream);

  /// @brief Get reference to the CUDA event
  cudaEvent_t & event() { return event_; }

private:
  cudaEvent_t event_;  ///< CUDA event handle
};

}  // namespace autoware::type_adaptation::type_adapters

#endif  // TYPE_ADAPTERS__CUDA_STREAM_WRAPPER_HPP_
