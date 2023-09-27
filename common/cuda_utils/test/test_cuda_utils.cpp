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

// This code is licensed under CC0 1.0 Universal (Public Domain).
// You can use this without any limitation.
// https://creativecommons.org/publicdomain/zero/1.0/deed.en
// borrowed from https://proc-cpuinfo.fixstars.com/2019/02/cuda_smart_pointer/

#include "cuda_utils/cuda_check_error.hpp"
#include "cuda_utils/cuda_unique_ptr.hpp"
#include "cuda_utils/stream_unique_ptr.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

TEST(CudaCheckErrorTest, TestCudaSuccess)
{
  ::cudaError_t e = ::cudaSuccess;
  EXPECT_NO_THROW(CHECK_CUDA_ERROR(e));
}

TEST(CudaCheckErrorTest, TestCudaError)
{
  ::cudaError_t e = ::cudaErrorMemoryAllocation;
  EXPECT_THROW(CHECK_CUDA_ERROR(e), std::runtime_error);
}

TEST(StreamUniquePtrTest, TestStreamCreation)
{
  // Create a CUDA stream using StreamUniquePtr
  cuda_utils::StreamUniquePtr cudaStream = cuda_utils::makeCudaStream();

  // Perform tests to verify stream behavior
  ASSERT_TRUE(cudaStream.get() != nullptr);
}

TEST(CudaUniquePtrTest, TestDeviceMemoryAllocation)
{
  // Allocate device memory using CudaUniquePtr
  cuda_utils::CudaUniquePtr<int> deviceMemory = cuda_utils::make_unique<int>();

  // Perform tests to verify device memory behavior
  ASSERT_TRUE(deviceMemory.get() != nullptr);
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
