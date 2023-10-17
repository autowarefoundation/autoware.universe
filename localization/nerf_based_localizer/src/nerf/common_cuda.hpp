// Copyright 2023 Autoware Foundation
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
//
// This file is derived from the following file.
// https://github.com/Totoro97/f2-nerf/blob/main/src/Common.h
//
// Created by ppwang on 2022/5/8.
//

#ifndef NERF__COMMON_CUDA_HPP_
#define NERF__COMMON_CUDA_HPP_

#include <cuda_runtime.h>

inline unsigned int DivUp(const unsigned int x, const unsigned int y)
{
  return (x + y - 1) / y;
}

constexpr unsigned int THREAD_CAP = 512;
constexpr dim3 LIN_BLOCK_DIM = {THREAD_CAP, 1, 1};

inline dim3 LIN_GRID_DIM(const int x)
{
  return dim3{DivUp(x, THREAD_CAP), 1, 1};
}

#endif  // NERF__COMMON_CUDA_HPP_
