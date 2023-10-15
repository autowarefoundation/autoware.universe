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
// Created by ppwang on 2022/5/11.
//

#ifndef NERF_BASED_LOCALIZER__UTILS_HPP_
#define NERF_BASED_LOCALIZER__UTILS_HPP_

#include <torch/torch.h>

#include <string>

namespace utils
{
using Tensor = torch::Tensor;

Tensor read_image_tensor(const std::string & path);
bool write_image_tensor(const std::string & path, Tensor img);
Tensor resize_image(Tensor image, const int resize_height, const int resize_width);
float calc_loss(Tensor pred_image, Tensor gt_image);

}  // namespace utils

#endif  // NERF_BASED_LOCALIZER__UTILS_HPP_
