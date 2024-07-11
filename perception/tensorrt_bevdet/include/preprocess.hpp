#ifndef __PREPROCESS_HPP__
#define __PREPROCESS_HPP__

#include "common.hpp"

void convert_RGBHWC_to_BGRCHW(uchar *input, uchar *output, 
                                                        int channels, int height, int width);

#endif