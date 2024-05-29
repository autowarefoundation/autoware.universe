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
// https://github.com/Totoro97/f2-nerf/blob/main/src/ExpRunner.cpp
//
// Created by ppwang on 2022/5/6.
//

#include "train_manager.hpp"

#include "../../src/nerf/CustomOps/CustomOps.hpp"
#include "../../src/nerf/stop_watch.hpp"
#include "../../src/nerf/utils.hpp"

#include <experimental/filesystem>
#include <opencv2/core.hpp>

#include <fmt/core.h>

#include <fstream>

namespace fs = std::experimental::filesystem::v1;
using Tensor = torch::Tensor;

TrainManager::TrainManager(const std::string & train_result_dir, const std::string & dataset_dir)
{
  const std::string conf_path = train_result_dir + "/train_config.yaml";
  fs::path p(conf_path);
  fs::path canonical_path = fs::canonical(p);
  const std::string path = canonical_path.string();
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("Failed to open " + conf_path);
  }

  train_result_dir_ = train_result_dir;
  fs::create_directories(train_result_dir_);

  const cv::FileNode train_config = fs["train"];
  pts_batch_size_ = (int)train_config["pts_batch_size"];
  end_iter_ = (int)train_config["end_iter"];
  vis_freq_ = (int)train_config["vis_freq"];
  report_freq_ = (int)train_config["report_freq"];
  save_freq_ = (int)train_config["save_freq"];
  learning_rate_ = (float)train_config["learning_rate"];
  learning_rate_alpha_ = (float)train_config["learning_rate_alpha"];
  learning_rate_warm_up_end_iter_ = (int)train_config["learning_rate_warm_up_end_iter"];
  var_loss_weight_ = (float)train_config["var_loss_weight"];
  var_loss_start_ = (int)train_config["var_loss_start"];
  var_loss_end_ = (int)train_config["var_loss_end"];

  // Dataset
  dataset_ = std::make_shared<Dataset>(dataset_dir);
  dataset_->save_inference_params(train_result_dir_);

  // Renderer
  renderer_ = std::make_shared<Renderer>(dataset_->n_images);
  renderer_->to(torch::kCUDA);

  // Optimizer
  optimizer_ = std::make_shared<torch::optim::Adam>(renderer_->optim_param_groups(learning_rate_));
}

void TrainManager::train()
{
  std::ofstream ofs_log(train_result_dir_ + "/train_log.txt");

  Timer timer;
  timer.start();

  float psnr_smooth = -1.0;
  update_ada_params();

  for (; iter_step_ < end_iter_;) {
    constexpr float sampled_pts_per_ray_ = 512.f;
    int cur_batch_size = int(pts_batch_size_ / sampled_pts_per_ray_) >> 4 << 4;
    auto [train_rays, gt_colors, emb_idx] = dataset_->sample_random_rays(cur_batch_size);

    Tensor & rays_o = train_rays.origins;
    Tensor & rays_d = train_rays.dirs;

    auto render_result = renderer_->render(rays_o, rays_d, emb_idx, RunningMode::TRAIN);
    Tensor pred_colors = render_result.colors.index({Slc(0, cur_batch_size)});
    Tensor color_loss = torch::sqrt((pred_colors - gt_colors).square() + 1e-4f).mean();

    Tensor sampled_weights = render_result.weights;
    Tensor idx_start_end = render_result.idx_start_end;
    Tensor sampled_var = CustomOps::WeightVar(sampled_weights, idx_start_end);
    Tensor var_loss = (sampled_var + 1e-2).sqrt().mean();

    float var_loss_weight = 0.f;
    if (iter_step_ > var_loss_end_) {
      var_loss_weight = var_loss_weight_;
    } else if (iter_step_ > var_loss_start_) {
      var_loss_weight = float(iter_step_ - var_loss_start_) /
                        float(var_loss_end_ - var_loss_start_) * var_loss_weight_;
    }

    Tensor loss = color_loss + var_loss * var_loss_weight;

    float mse = (pred_colors - gt_colors).square().mean().item<float>();
    float psnr = 20.f * std::log10(1 / std::sqrt(mse));
    psnr_smooth = psnr_smooth < 0.f ? psnr : psnr * .1f + psnr_smooth * .9f;
    CHECK(!std::isnan(pred_colors.mean().item<float>()));
    CHECK(!std::isnan(gt_colors.mean().item<float>()));
    CHECK(!std::isnan(mse));

    // There can be some cases that the output colors have no grad due to the occupancy grid.
    if (loss.requires_grad()) {
      optimizer_->zero_grad();
      loss.backward();
      optimizer_->step();
    }

    iter_step_++;

    if (iter_step_ % vis_freq_ == 0) {
      torch::NoGradGuard no_grad_guard;

      const int idx = 0;
      const int H = dataset_->height;
      const int W = dataset_->width;
      const int ray_batch_size = 8192;
      auto [pred_colors, pred_depths] = renderer_->render_image(
        dataset_->poses[idx], dataset_->intrinsics[idx], H, W, ray_batch_size);

      Tensor image = dataset_->images[idx].reshape({H, W, 3}).to(torch::kCPU);
      pred_colors = pred_colors.to(torch::kCPU);
      pred_depths = pred_depths.to(torch::kCPU);

      Tensor concat_tensor = torch::cat({image, pred_colors, pred_depths}, 1);
      fs::create_directories(train_result_dir_ + "/images");
      std::stringstream ss;
      ss << std::setw(8) << std::setfill('0') << iter_step_ << "_" << idx << ".png";
      utils::write_image_tensor(train_result_dir_ + "/images/" + ss.str(), concat_tensor);
    }

    if (iter_step_ % save_freq_ == 0) {
      fs::remove_all(train_result_dir_ + "/checkpoints/latest");
      fs::create_directories(train_result_dir_ + "/checkpoints/latest");
      torch::save(renderer_, train_result_dir_ + "/checkpoints/latest/renderer.pt");
    }

    if (iter_step_ % report_freq_ == 0) {
      const int64_t total_sec = timer.elapsed_seconds();
      const int64_t total_m = total_sec / 60;
      const int64_t total_s = total_sec % 60;
      std::stringstream ss;
      ss << std::fixed;
      ss << "Time: " << std::setw(2) << std::setfill('0') << total_m << ":" << std::setw(2)
         << std::setfill('0') << total_s << " ";
      ss << "Iter: " << std::setw(6) << iter_step_ << " ";
      ss << "PSNR: " << psnr_smooth << " ";
      ss << "LOSS: " << color_loss.item<float>() << " ";
      ss << "LR: " << optimizer_->param_groups()[0].options().get_lr();
      const std::string log_str = ss.str();
      std::cout << log_str << std::endl;
      ofs_log << log_str << std::endl;
    }
    update_ada_params();
  }

  std::cout << "Train done" << std::endl;
}

void TrainManager::update_ada_params()
{
  // Update learning rate
  float lr_factor;
  if (iter_step_ >= learning_rate_warm_up_end_iter_) {
    float progress = float(iter_step_ - learning_rate_warm_up_end_iter_) /
                     float(end_iter_ - learning_rate_warm_up_end_iter_);
    lr_factor = (1.f - learning_rate_alpha_) * (std::cos(progress * float(M_PI)) * .5f + .5f) +
                learning_rate_alpha_;
  } else {
    lr_factor = float(iter_step_) / float(learning_rate_warm_up_end_iter_);
  }
  float lr = learning_rate_ * lr_factor;
  for (auto & g : optimizer_->param_groups()) {
    g.options().set_lr(lr);
  }
}
