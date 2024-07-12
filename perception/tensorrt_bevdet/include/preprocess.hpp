#ifndef PREPROCESS_HPP_
#define PREPROCESS_HPP_

#include "common.hpp"

void convert_RGBHWC_to_BGRCHW(uchar * input, uchar * output, int channels, int height, int width);

#endif  // PREPROCESS_HPP_
