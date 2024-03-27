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
// https://github.com/Totoro97/f2-nerf/blob/main/src/Dataset/Dataset.cpp
//
// Created by ppwang on 2022/5/7.
//
#include "dataset.hpp"

#include "stop_watch.hpp"
#include "utils.hpp"

#include <experimental/filesystem>

#include <fmt/core.h>
#include <glob.h>

#include <fstream>
#include <iostream>

using Tensor = torch::Tensor;

namespace fs = std::experimental::filesystem::v1;

Dataset::Dataset(const std::string & data_path)
{
  ScopeWatch dataset_watch("Dataset::Dataset");

  std::cout << "data_path = " << data_path << std::endl;

  // Load camera pose
  CHECK(fs::exists(data_path + "/cams_meta.tsv"));
  {
    std::ifstream ifs(data_path + "/cams_meta.tsv");
    std::string line;
    std::getline(ifs, line);  // header
    std::vector<Tensor> poses_vec, intrinsics_vec, dist_params_vec;
    while (std::getline(ifs, line)) {
      std::istringstream iss(line);
      std::vector<std::string> tokens;
      std::string token;
      while (std::getline(iss, token, '\t')) {
        tokens.push_back(token);
      }
      const int POSE_NUM = 12;       //(3, 4)
      const int INTRINSIC_NUM = 9;   //(3, 3)
      const int DISTORTION_NUM = 4;  //(k1, k2, p1, p2)
      CHECK(tokens.size() == POSE_NUM + INTRINSIC_NUM + DISTORTION_NUM);
      Tensor pose = torch::zeros({3, 4}, torch::kFloat32);
      for (int i = 0; i < POSE_NUM; i++) {
        pose.index_put_({i / 4, i % 4}, std::stof(tokens[i]));
      }
      pose = pose.reshape({3, 4});
      poses_vec.push_back(pose);

      Tensor intrinsic = torch::zeros({3, 3}, torch::kFloat32);
      for (int i = 0; i < INTRINSIC_NUM; i++) {
        intrinsic.index_put_({i / 3, i % 3}, std::stof(tokens[POSE_NUM + i]));
      }
      intrinsic = intrinsic.reshape({3, 3});
      intrinsics_vec.push_back(intrinsic);

      Tensor dist_param = torch::zeros({4}, torch::kFloat32);
      for (int i = 0; i < DISTORTION_NUM; i++) {
        dist_param.index_put_({i}, std::stof(tokens[POSE_NUM + INTRINSIC_NUM + i]));
      }
      dist_params_vec.push_back(dist_param);
    }

    n_images = poses_vec.size();
    poses = torch::stack(poses_vec, 0).contiguous().to(torch::kCUDA);
    intrinsics = torch::stack(intrinsics_vec, 0).contiguous().to(torch::kCUDA);
    dist_params = torch::stack(dist_params_vec, 0).contiguous().to(torch::kCUDA);
  }

  // Normalize scene
  {
    Tensor cam_pos = poses.index({Slc(), Slc(0, 3), 3}).clone();
    center = cam_pos.mean(0, false);
    Tensor bias = cam_pos - center.unsqueeze(0);
    radius = torch::linalg_norm(bias, 2, -1, false).max().item<float>();
    cam_pos = (cam_pos - center.unsqueeze(0)) / radius;
    poses.index_put_({Slc(), Slc(0, 3), 3}, cam_pos);
    poses = poses.contiguous();
  }

  std::vector<Tensor> images_vec;
  // Load images
  {
    ScopeWatch watch("LoadImages");
    const std::vector<std::string> image_paths = glob_image_paths(data_path + "/images/");
    for (int i = 0; i < n_images; i++) {
      const std::string image_path = image_paths[i];
      images_vec.push_back(utils::read_image_tensor(image_path).to(torch::kCPU));
    }
  }

  std::cout << "Number of images: " << n_images << std::endl;

  height = images_vec[0].size(0);
  width = images_vec[0].size(1);
  images = torch::stack(images_vec, 0).contiguous();
}

void Dataset::save_inference_params(const std::string & train_result_dir) const
{
  std::ofstream ofs(train_result_dir + "/inference_params.yaml");
  ofs << std::fixed;
  ofs << "%YAML 1.2" << std::endl;
  ofs << "---" << std::endl;
  ofs << "n_images: " << n_images << std::endl;
  ofs << "height: " << height << std::endl;
  ofs << "width: " << width << std::endl;

  ofs << "intrinsic: [";
  ofs << intrinsics[0][0][0].item() << ", ";
  ofs << intrinsics[0][0][1].item() << ", ";
  ofs << intrinsics[0][0][2].item() << "," << std::endl;
  ofs << "            ";
  ofs << intrinsics[0][1][0].item() << ", ";
  ofs << intrinsics[0][1][1].item() << ", ";
  ofs << intrinsics[0][1][2].item() << "," << std::endl;
  ofs << "            ";
  ofs << intrinsics[0][2][0].item() << ", ";
  ofs << intrinsics[0][2][1].item() << ", ";
  ofs << intrinsics[0][2][2].item() << "]" << std::endl;

  ofs << "normalizing_center: [" << center[0].item();
  ofs << ", " << center[1].item();
  ofs << ", " << center[2].item() << "]" << std::endl;
  ofs << "normalizing_radius: " << radius << std::endl;
}

Rays Dataset::get_all_rays_of_camera(int idx)
{
  int H = height;
  int W = width;
  Tensor ii = torch::linspace(0.f, H - 1.f, H, CUDAFloat);
  Tensor jj = torch::linspace(0.f, W - 1.f, W, CUDAFloat);
  auto ij = torch::meshgrid({ii, jj}, "ij");
  Tensor i = ij[0].reshape({-1});
  Tensor j = ij[1].reshape({-1});

  auto [rays_o, rays_d] = get_rays_from_pose(
    poses[idx].unsqueeze(0), intrinsics[idx].unsqueeze(0), torch::stack({i, j}, -1));
  return {rays_o, rays_d};
}

std::tuple<Rays, Tensor, Tensor> Dataset::sample_random_rays(int batch_size)
{
  const auto CPULong = torch::TensorOptions().dtype(torch::kLong).device(torch::kCPU);
  Tensor cam_indices = torch::randint(n_images, {batch_size}, CPULong);
  Tensor i = torch::randint(0, height, batch_size, CPULong);
  Tensor j = torch::randint(0, width, batch_size, CPULong);
  Tensor ij = torch::stack({i, j}, -1).to(torch::kCUDA).contiguous();

  Tensor gt_colors = images.view({-1, 3})
                       .index({(cam_indices * height * width + i * width + j).to(torch::kLong)})
                       .to(torch::kCUDA)
                       .contiguous();
  cam_indices = cam_indices.to(torch::kCUDA);
  cam_indices = cam_indices.to(torch::kInt32);
  ij = ij.to(torch::kInt32);

  Tensor selected_poses = torch::index_select(poses, 0, cam_indices);
  Tensor selected_intrinsics = torch::index_select(intrinsics, 0, cam_indices);
  auto [rays_o, rays_d] = get_rays_from_pose(selected_poses, selected_intrinsics, ij);

  return {{rays_o, rays_d}, gt_colors, cam_indices.to(torch::kInt32).contiguous()};
}

std::vector<std::string> Dataset::glob_image_paths(const std::string & input_dir)
{
  glob_t buffer;
  std::vector<std::string> files;
  glob((input_dir + "*.png").c_str(), 0, NULL, &buffer);
  for (size_t i = 0; i < buffer.gl_pathc; i++) {
    files.push_back(buffer.gl_pathv[i]);
  }
  globfree(&buffer);
  std::sort(files.begin(), files.end());
  return files;
}
