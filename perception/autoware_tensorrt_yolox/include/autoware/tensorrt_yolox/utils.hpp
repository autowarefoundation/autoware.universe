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

#ifndef AUTOWARE__TENSORRT_YOLOX__UTILS_HPP_
#define AUTOWARE__TENSORRT_YOLOX__UTILS_HPP_
#include <opencv2/opencv.hpp>

namespace autoware::tensorrt_yolox
{
std::vector<std::pair<uint8_t, int>> runLengthEncoder(const cv::Mat & mask);
cv::Mat runLengthDecoder(const std::vector<uint8_t> & rle_data, const int rows, const int cols);
}  // namespace autoware::tensorrt_yolox

#endif  // AUTOWARE__TENSORRT_YOLOX__UTILS_HPP_
