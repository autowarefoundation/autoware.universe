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
// Created by ppwang on 2022/5/7.
//

#ifndef NERF_BASED_LOCALIZER__DATASET_HPP_
#define NERF_BASED_LOCALIZER__DATASET_HPP_

#include "common.hpp"
#include "rays.hpp"

#include <torch/torch.h>

#include <string>
#include <tuple>
#include <vector>

struct Dataset
{
  using Tensor = torch::Tensor;

public:
  Dataset(const std::string & data_path);

  void save_inference_params(const std::string & train_result_dir) const;

  Rays get_all_rays_of_camera(int idx);

  std::tuple<Rays, Tensor, Tensor> sample_random_rays(int batch_size);

  static std::vector<std::string> glob_image_paths(const std::string & input_dir);

  int n_images;
  Tensor poses, images, intrinsics, dist_params;
  Tensor center;
  float radius;
  int height, width;
};

#endif  // NERF_BASED_LOCALIZER__DATASET_HPP_
