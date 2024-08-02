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
#include "autoware/tensorrt_bevdet/preprocess.hpp"

#include <thrust/fill.h>

#include <fstream>

__global__ void convert_RGBHWC_to_BGRCHW_kernel(
  uchar * input, uchar * output, int channels, int height, int width)
{
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index < channels * height * width) {
    int y = index / 3 / width;
    int x = index / 3 % width;
    int c = 2 - index % 3;  // RGB to BGR

    output[c * height * width + y * width + x] = input[index];
  }
}
// RGBHWC to BGRCHW
void convert_RGBHWC_to_BGRCHW(uchar * input, uchar * output, int channels, int height, int width)
{
  convert_RGBHWC_to_BGRCHW_kernel<<<DIVUP(channels * height * width, NUM_THREADS), NUM_THREADS>>>(
    input, output, channels, height, width);
}
