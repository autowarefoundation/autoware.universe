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

#include "type_adapters/cuda_stream_wrapper.hpp"

// cspell:ignore nvtx

#include "cuda.h"          // NOLINT
#include "cuda_runtime.h"  // NOLINT

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <nvToolsExt.h>  // NOLINT

#include <memory>
#include <stdexcept>
#include <string>

namespace autoware::type_adaptation::type_adapters
{

CUDAStreamWrapper::CUDAStreamWrapper()
{
  cudaStreamCreate(&main_stream_);
}

CUDAStreamWrapper::~CUDAStreamWrapper()
{
  cudaStreamDestroy(main_stream_);
}

CUDAMemoryWrapper::CUDAMemoryWrapper(size_t bytes_to_allocate) : bytes_allocated_(bytes_to_allocate)
{
  if (cudaMallocManaged(&cuda_mem_, bytes_to_allocate) != cudaSuccess) {
    throw std::runtime_error("Failed to allocate device memory");
  }
}

void CUDAMemoryWrapper::copy_to_device(
  uint8_t * host_mem, size_t bytes_to_copy, const cudaStream_t & stream)
{
  nvtxRangePushA("ImageContainer:CopyToDevice");
  if (bytes_to_copy > bytes_allocated_) {
    throw std::invalid_argument("Tried to copy too many bytes to device");
  }
  if (
    cudaMemcpyAsync(cuda_mem_, host_mem, bytes_to_copy, cudaMemcpyHostToDevice, stream) !=
    cudaSuccess) {
    throw std::runtime_error("Failed to copy memory to the GPU");
  }
  cudaStreamSynchronize(stream);
  nvtxRangePop();
}

void CUDAMemoryWrapper::copy_from_device(
  uint8_t * host_mem, size_t bytes_to_copy, const cudaStream_t & stream)
{
  nvtxRangePushA("ImageContainer:CopyFromDevice");
  if (bytes_to_copy > bytes_allocated_) {
    throw std::invalid_argument("Tried to copy too many bytes from device");
  }
  if (
    cudaMemcpyAsync(host_mem, cuda_mem_, bytes_to_copy, cudaMemcpyDeviceToHost, stream) !=
    cudaSuccess) {
    throw std::runtime_error("Failed to copy memory from the GPU");
  }
  cudaStreamSynchronize(stream);
  nvtxRangePop();
}

uint8_t * CUDAMemoryWrapper::device_memory()
{
  return cuda_mem_;
}

CUDAMemoryWrapper::~CUDAMemoryWrapper()
{
  cudaFree(cuda_mem_);
}

CUDAEventWrapper::CUDAEventWrapper()
{
  cudaEventCreate(&event_);
}

void CUDAEventWrapper::record(std::shared_ptr<CUDAStreamWrapper> cuda_stream)
{
  cudaEventRecord(event_, cuda_stream->stream());
}

CUDAEventWrapper::~CUDAEventWrapper()
{
  cudaEventDestroy(event_);
}

}  //  namespace autoware::type_adaptation::type_adapters
