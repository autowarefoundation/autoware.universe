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
#ifndef AUTOWARE__TENSORRT_BEVDET__COMMON_HPP_
#define AUTOWARE__TENSORRT_BEVDET__COMMON_HPP_

#include <NvInfer.h>
#include <assert.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <thrust/sort.h>

#include <iostream>
#include <string>

typedef unsigned char uchar;

#define NUM_THREADS 512

#define CHECK_CUDA(ans)                   \
  {                                       \
    GPUAssert((ans), __FILE__, __LINE__); \
  }

#define DIVUP(m, n) (((m) + (n) - 1) / (n))

inline void GPUAssert(cudaError_t code, const char * file, int line, bool abort = true)
{
  if (code != cudaSuccess) {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
    if (abort) exit(code);
  }
};

#define CUDA_1D_KERNEL_LOOP(i, n) \
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < (n); i += blockDim.x * gridDim.x)

const int MAXTENSORDIMS = 6;
struct TensorDesc
{
  int shape[MAXTENSORDIMS];
  int stride[MAXTENSORDIMS];
  int dim;
};

inline int GET_BLOCKS(const int N)
{
  int optimal_block_num = DIVUP(N, NUM_THREADS);
  int max_block_num = 4096;
  return optimal_block_num < max_block_num ? optimal_block_num : max_block_num;
}

__inline__ std::string dataTypeToString(nvinfer1::DataType dataType)
{
  switch (dataType) {
    case nvinfer1::DataType::kFLOAT:
      return std::string("FP32 ");
    case nvinfer1::DataType::kHALF:
      return std::string("FP16 ");
    case nvinfer1::DataType::kINT8:
      return std::string("INT8 ");
    case nvinfer1::DataType::kINT32:
      return std::string("INT32");
    case nvinfer1::DataType::kBOOL:
      return std::string("BOOL ");
    default:
      return std::string("Unknown");
  }
}

__inline__ size_t dataTypeToSize(nvinfer1::DataType dataType)
{
  switch (dataType) {
    case nvinfer1::DataType::kFLOAT:
      return 4;
    case nvinfer1::DataType::kHALF:
      return 2;
    case nvinfer1::DataType::kINT8:
      return 1;
    case nvinfer1::DataType::kINT32:
      return 4;
    case nvinfer1::DataType::kBOOL:
      return 1;
    default:
      return 4;
  }
}

__inline__ std::string shapeToString(nvinfer1::Dims32 dim)
{
  std::string output("(");
  if (dim.nbDims == 0) {
    return output + std::string(")");
  }
  for (int i = 0; i < dim.nbDims - 1; ++i) {
    output += std::to_string(dim.d[i]) + std::string(", ");
  }
  output += std::to_string(dim.d[dim.nbDims - 1]) + std::string(")");
  return output;
}

class Logger : public nvinfer1::ILogger
{
public:
  explicit Logger(Severity severity = Severity::kWARNING) : reportable_severity(severity) {}

  void log(Severity severity, const char * msg) noexcept override
  {
    // suppress messages with severity enum value greater than the reportable
    if (severity > reportable_severity) return;
    switch (severity) {
      case Severity::kINTERNAL_ERROR:
        std::cerr << "INTERNAL_ERROR: ";
        break;
      case Severity::kERROR:
        std::cerr << "ERROR: ";
        break;
      case Severity::kWARNING:
        std::cerr << "WARNING: ";
        break;
      case Severity::kINFO:
        std::cerr << "INFO: ";
        break;
      default:
        std::cerr << "UNKNOWN: ";
        break;
    }
    std::cerr << msg << std::endl;
  }

  Severity reportable_severity;
};

#endif  // AUTOWARE__TENSORRT_BEVDET__COMMON_HPP_
