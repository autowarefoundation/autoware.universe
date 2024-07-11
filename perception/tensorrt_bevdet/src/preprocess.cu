#include "common.hpp"
#include "preprocess.hpp"

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
