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
// This file is derived from the following file.
// https://github.com/Totoro97/f2-nerf/blob/main/src/Utils/Utils.cpp
//
// Created by ppwang on 2023/4/4.
//

#include "utils.hpp"

#include "common.hpp"

#include <opencv2/opencv.hpp>

#include <torch/torch.h>

using Tensor = torch::Tensor;

Tensor utils::read_image_tensor(const std::string & path)
{
  cv::Mat img = cv::imread(path, cv::IMREAD_UNCHANGED);
  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
  img.convertTo(img, CV_32FC3, 1.0 / 255.0);
  Tensor img_tensor =
    torch::from_blob(img.data, {img.rows, img.cols, img.channels()}, torch::kFloat32).clone();
  return img_tensor;
}

bool utils::write_image_tensor(const std::string & path, Tensor img)
{
  img = img.contiguous();
  img = (img * 255.f).clamp(0, 255).to(torch::kUInt8).to(torch::kCPU);
  cv::Mat img_mat(img.size(0), img.size(1), CV_8UC3, img.data_ptr());
  cv::cvtColor(img_mat, img_mat, cv::COLOR_RGB2BGR);
  cv::imwrite(path, img_mat);
  return true;
}

Tensor utils::resize_image(Tensor image, const int resize_height, const int resize_width)
{
  const int height = image.size(0);
  const int width = image.size(1);
  if (height == resize_height && width == resize_width) {
    return image;
  }

  // change HWC to CHW
  image = image.permute({2, 0, 1});
  image = image.unsqueeze(0);  // add batch dim

  // Resize
  std::vector<int64_t> size = {resize_height, resize_width};
  image = torch::nn::functional::interpolate(
    image, torch::nn::functional::InterpolateFuncOptions().size(size));

  // change CHW to HWC
  image = image.squeeze(0);  // remove batch dim
  image = image.permute({1, 2, 0});
  return image;
}

float utils::calc_loss(Tensor pred_image, Tensor gt_image)
{
  Tensor diff = pred_image - gt_image;
  Tensor loss = (diff * diff).mean(-1);
  Tensor score = loss.numel() / (loss.sum() + 1e-6f);
  return score.mean().item<float>();
}
