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
#include "../../src/nerf/localizer.hpp"
#include "../../src/nerf/stop_watch.hpp"
#include "../../src/nerf/utils.hpp"
#include "main_functions.hpp"

#include <experimental/filesystem>
#include <opencv2/core.hpp>

#include <fstream>

namespace fs = std::experimental::filesystem::v1;

enum Dir { kUp, kUpRight, kRight, kDownRight, kDown, kDownLeft, kLeft, kUpLeft, kDirNum };
constexpr int64_t kDx[kDirNum] = {0, 1, 1, 1, 0, -1, -1, -1};
constexpr int64_t kDz[kDirNum] = {1, 1, 0, -1, -1, -1, 0, 1};

void infer(const std::string & train_result_dir, const std::string & dataset_dir)
{
  LocalizerParam param{};
  param.resize_factor = 32;
  param.train_result_dir = train_result_dir;
  Localizer core(param);

  constexpr int32_t iteration_num = 10;

  const std::string save_dir = train_result_dir + "/inference_result/";
  fs::create_directories(save_dir);

  Dataset dataset(dataset_dir);

  Timer timer, timer_local;
  timer.start();

  std::vector<double> optimize_times;

  const float noise = 0.5f / core.radius();
  std::cout << "noise = " << noise << std::endl;

  for (int32_t i = 0; i < dataset.n_images; i++) {
    std::cout << "\rTime " << static_cast<int64_t>(timer.elapsed_seconds()) << " " << i << "/"
              << dataset.n_images << std::flush;
    const std::string curr_dir =
      (std::stringstream() << save_dir << "/" << std::setfill('0') << std::setw(4) << i << "/")
        .str();
    fs::create_directories(curr_dir);

    torch::Tensor initial_pose = dataset.poses[i];
    torch::Tensor image_tensor = dataset.images[i];

    image_tensor = utils::resize_image(image_tensor, core.infer_height(), core.infer_width());
    image_tensor = image_tensor.to(torch::kCUDA);
    utils::write_image_tensor(curr_dir + "image_01_gt.png", image_tensor);

    std::ofstream ofs(curr_dir + "/position.tsv");
    ofs << std::fixed;
    ofs << "name\tx\ty\tz\tscore" << std::endl;
    auto output = [&](const std::string & name, const torch::Tensor & pose, float score) {
      const torch::Tensor out = core.nerf2camera(pose);
      ofs << name << "\t";
      ofs << out[0][3].item<float>() << "\t";
      ofs << out[1][3].item<float>() << "\t";
      ofs << out[2][3].item<float>() << "\t";
      ofs << score << std::endl;
    };

    // Before noise
    torch::Tensor nerf_image_before = core.render_image(initial_pose);
    float score_before = utils::calc_loss(nerf_image_before, image_tensor);
    utils::write_image_tensor(curr_dir + "image_02_before.png", nerf_image_before);
    output("original", initial_pose, score_before);

    // Added noise
    for (int32_t d = 0; d < kDirNum; d++) {
      torch::Tensor curr_pose = initial_pose.clone();
      curr_pose[0][3] += noise * kDx[d];
      curr_pose[2][3] += noise * kDz[d];
      torch::Tensor nerf_image_noised = core.render_image(curr_pose);
      float score_noised = utils::calc_loss(nerf_image_noised, image_tensor);
      utils::write_image_tensor(
        curr_dir + "image_03_noised" + std::to_string(d) + ".png", nerf_image_noised);
      output("noised_" + std::to_string(d), curr_pose, score_noised);

      // Optimize
      timer_local.start();
      std::vector<torch::Tensor> optimized_poses =
        core.optimize_pose_by_differential(curr_pose, image_tensor, iteration_num, 1e-4f);
      optimize_times.push_back(timer_local.elapsed_seconds());
      for (int32_t itr = 0; itr < optimized_poses.size(); itr++) {
        torch::Tensor optimized_pose = optimized_poses[itr];
        torch::Tensor nerf_image_after = core.render_image(optimized_pose);
        float score_after = utils::calc_loss(nerf_image_after, image_tensor);
        const std::string suffix =
          (std::stringstream() << d << "_" << std::setfill('0') << std::setw(2) << itr).str();
        utils::write_image_tensor(curr_dir + "image_04_after_" + suffix + ".png", nerf_image_after);
        output("optimized_" + suffix, optimized_pose, score_after);
      }
    }
  }

  torch::Tensor optimize_time_tensor = torch::tensor(optimize_times, torch::kFloat);

  std::cout << "\nAverage Time = " << optimize_time_tensor.mean().item<float>() << " sec"
            << std::endl;
}
