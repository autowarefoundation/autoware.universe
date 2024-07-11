#ifndef CPU_JPEGDECODER_HPP_
#define CPU_JPEGDECODER_HPP_

#include "common.hpp"

#include <vector>

int decode_cpu(
  const std::vector<std::vector<char>> & files_data, uchar * out_imgs, size_t width, size_t height);

#endif  // CPU_JPEGDECODER_HPP_
