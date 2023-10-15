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
#ifndef NERF__LOCALIZER_HPP_
#define NERF__LOCALIZER_HPP_

#include "dataset.hpp"
#include "renderer.hpp"

#include <torch/torch.h>

struct Particle
{
  torch::Tensor pose;  // (3, 4)
  float weight;
};

struct LocalizerParam
{
  std::string train_result_dir;
  int32_t render_pixel_num = 256;
  float noise_position_x = 0.025f;
  float noise_position_y = 0.025f;
  float noise_position_z = 0.025f;
  float noise_rotation_x = 2.5f;
  float noise_rotation_y = 2.5f;
  float noise_rotation_z = 2.5f;
  int32_t resize_factor = 1;
};

class Localizer
{
  using Tensor = torch::Tensor;

public:
  Localizer() = default;
  Localizer(const LocalizerParam & param);

  Tensor render_image(const Tensor & pose);
  std::vector<Particle> optimize_pose_by_random_search(
    Tensor initial_pose, Tensor image_tensor, int64_t particle_num, float noise_coeff);
  std::vector<Tensor> optimize_pose_by_differential(
    Tensor initial_pose, Tensor image_tensor, int64_t iteration_num, float learning_rate);

  torch::Tensor camera2nerf(const torch::Tensor & pose_in_world);
  torch::Tensor nerf2camera(const torch::Tensor & pose_in_camera);

  static Tensor calc_average_pose(const std::vector<Particle> & particles);

  float radius() const { return radius_; }
  int infer_height() const { return infer_height_; }
  int infer_width() const { return infer_width_; }

private:
  std::vector<float> evaluate_poses(const std::vector<Tensor> & poses, const Tensor & image);

  LocalizerParam param_;

  std::shared_ptr<Renderer> renderer_;

  torch::Tensor axis_convert_mat_;

  int infer_height_, infer_width_;
  Tensor intrinsic_;
  Tensor center_;
  float radius_;
};

#endif  // NERF__LOCALIZER_HPP_
