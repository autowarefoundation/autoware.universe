// Copyright 2024 AutoCore, Inc.
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
#ifndef AUTOWARE__TENSORRT_BEVDET__CPU_JPEGDECODER_HPP_
#define AUTOWARE__TENSORRT_BEVDET__CPU_JPEGDECODER_HPP_

#include "common.hpp"

#include <vector>

int decode_cpu(
  const std::vector<std::vector<char>> & files_data, uchar * out_imgs, size_t width, size_t height);

#endif  // AUTOWARE__TENSORRT_BEVDET__CPU_JPEGDECODER_HPP_
