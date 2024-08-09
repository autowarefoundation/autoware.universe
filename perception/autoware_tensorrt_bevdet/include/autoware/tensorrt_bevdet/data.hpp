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
#ifndef AUTOWARE__TENSORRT_BEVDET__DATA_HPP_
#define AUTOWARE__TENSORRT_BEVDET__DATA_HPP_

#include "common.hpp"
#include "nvjpegdecoder.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

struct camParams
{
  camParams() = default;
  camParams(const YAML::Node & config, int n, std::vector<std::string> & cams_name);

  int N_img;

  Eigen::Quaternion<float> ego2global_rot;
  Eigen::Translation3f ego2global_trans;

  Eigen::Quaternion<float> lidar2ego_rot;
  Eigen::Translation3f lidar2ego_trans;
  //
  std::vector<Eigen::Matrix3f> cams_intrin;
  std::vector<Eigen::Quaternion<float>> cams2ego_rot;
  std::vector<Eigen::Translation3f> cams2ego_trans;
  //
  std::vector<std::string> imgs_file;

  std::int64_t timestamp;
  std::string scene_token;
};

struct camsData
{
  camsData() = default;
  explicit camsData(const camParams & _param) : param(_param), imgs_dev(nullptr) {}
  camParams param;
  uchar * imgs_dev;
};

Eigen::Translation3f fromYamlTrans(YAML::Node x);
Eigen::Quaternion<float> fromYamlQuater(YAML::Node x);
Eigen::Matrix3f fromYamlMatrix3f(YAML::Node x);

int read_image(const std::string & image_names, std::vector<char> & raw_data);

int read_sample(
  const std::vector<std::string> & imgs_file, std::vector<std::vector<char>> & imgs_data);

#endif  // AUTOWARE__TENSORRT_BEVDET__DATA_HPP_
