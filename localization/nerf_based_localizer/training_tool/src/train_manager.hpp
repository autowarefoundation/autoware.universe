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
// Created by ppwang on 2022/5/6.
//

#ifndef NERF_BASED_LOCALIZER__TRAIN_MANAGER_HPP_
#define NERF_BASED_LOCALIZER__TRAIN_MANAGER_HPP_

#include "../../src/nerf/dataset.hpp"
#include "../../src/nerf/renderer.hpp"

#include <torch/torch.h>

#include <memory>
#include <string>
#include <tuple>

class TrainManager
{
  using Tensor = torch::Tensor;

public:
  TrainManager(const std::string & train_result_dir, const std::string & dataset_dir);

  void train();

  void update_ada_params();

  // data
  std::string train_result_dir_;

  unsigned iter_step_ = 0;
  unsigned end_iter_;
  unsigned report_freq_, vis_freq_, save_freq_;
  unsigned pts_batch_size_;

  int var_loss_start_, var_loss_end_;
  float learning_rate_, learning_rate_alpha_, learning_rate_warm_up_end_iter_;
  float var_loss_weight_;

  std::shared_ptr<Dataset> dataset_;
  std::shared_ptr<Renderer> renderer_;
  std::shared_ptr<torch::optim::Adam> optimizer_;
};

#endif  // NERF_BASED_LOCALIZER__TRAIN_MANAGER_HPP_
