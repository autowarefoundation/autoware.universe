#ifndef __CPU_JPEGDECODER__
#define __CPU_JPEGDECODER__

#include <vector>
#include "common.hpp"

int decode_cpu(const std::vector<std::vector<char>> &files_data, uchar* out_imgs, size_t width, size_t height);

#endif