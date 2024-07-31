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
#include "data.hpp"

#include "cpu_jpegdecoder.hpp"

#include <stdlib.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>

using std::chrono::duration;
using std::chrono::high_resolution_clock;

camParams::camParams(const YAML::Node & config, int n, std::vector<std::string> & cams_name)
: N_img(n)
{
  if (static_cast<size_t>(n) != cams_name.size()) {
    std::cerr << "Error! Need " << n << " camera param, bug given " << cams_name.size()
              << " camera names!" << std::endl;
  }
  ego2global_rot = fromYamlQuater(config["ego2global_rotation"]);
  ego2global_trans = fromYamlTrans(config["ego2global_translation"]);

  lidar2ego_rot = fromYamlQuater(config["lidar2ego_rotation"]);
  lidar2ego_trans = fromYamlTrans(config["lidar2ego_translation"]);

  timestamp = config["timestamp"].as<std::int64_t>();
  scene_token = config["scene_token"].as<std::string>();

  imgs_file.clear();

  cams_intrin.clear();
  cams2ego_rot.clear();
  cams2ego_trans.clear();

  for (const std::string & name : cams_name) {
    imgs_file.push_back("." + config["cams"][name]["data_path"].as<std::string>());

    //
    cams_intrin.push_back(fromYamlMatrix3f(config["cams"][name]["cam_intrinsic"]));
    cams2ego_rot.push_back(fromYamlQuater(config["cams"][name]["sensor2ego_rotation"]));
    cams2ego_trans.push_back(fromYamlTrans(config["cams"][name]["sensor2ego_translation"]));
    //
  }
}

DataLoader::DataLoader(
  int _n_img, int _h, int _w, const std::string & _data_infos_path,
  const std::vector<std::string> & _cams_name, bool _sep)
: n_img(_n_img),
  cams_intrin(_n_img),
  cams2ego_rot(_n_img),
  cams2ego_trans(_n_img),
#ifdef __HAVE_NVJPEG__
  nvdecoder(_n_img, DECODE_BGR),
#endif
  img_h(_h),
  img_w(_w),
  cams_name(_cams_name),
  data_infos_path(_data_infos_path),
  separate(_sep)
{
  YAML::Node temp_seq = YAML::LoadFile(data_infos_path + "/time_sequence.yaml");
  time_sequence = temp_seq["time_sequence"].as<std::vector<int>>();
  sample_num = time_sequence.size();

  cams_param.resize(sample_num);

  if (separate == false) {
    YAML::Node infos = YAML::LoadFile(data_infos_path + "/samples_info.yaml");

    for (size_t i = 0; i < cams_name.size(); i++) {
      cams_intrin[i] = fromYamlMatrix3f(infos[0]["cams"][cams_name[i]]["cam_intrinsic"]);
      cams2ego_rot[i] = fromYamlQuater(infos[0]["cams"][cams_name[i]]["sensor2ego_rotation"]);
      cams2ego_trans[i] = fromYamlTrans(infos[0]["cams"][cams_name[i]]["sensor2ego_translation"]);
    }
    lidar2ego_rot = fromYamlQuater(infos[0]["lidar2ego_rotation"]);
    lidar2ego_trans = fromYamlTrans(infos[0]["lidar2ego_translation"]);

    for (int i = 0; i < sample_num; i++) {
      cams_param[i] = camParams(infos[i], n_img, cams_name);
    }
  } else {
    YAML::Node config0 = YAML::LoadFile(data_infos_path + "/samples_info/sample0000.yaml");

    for (size_t i = 0; i < cams_name.size(); i++) {
      cams_intrin[i] = fromYamlMatrix3f(config0["cams"][cams_name[i]]["cam_intrinsic"]);
      cams2ego_rot[i] = fromYamlQuater(config0["cams"][cams_name[i]]["sensor2ego_rotation"]);
      cams2ego_trans[i] = fromYamlTrans(config0["cams"][cams_name[i]]["sensor2ego_translation"]);
    }
    lidar2ego_rot = fromYamlQuater(config0["lidar2ego_rotation"]);
    lidar2ego_trans = fromYamlTrans(config0["lidar2ego_translation"]);
  }

  CHECK_CUDA(cudaMalloc(reinterpret_cast<void **>(&imgs_dev), n_img * img_h * img_w * 3 * sizeof(uchar)));
}

const camsData & DataLoader::data(int idx, bool time_order)
{
  if (time_order) {
    idx = time_sequence[idx];
  }
  printf("------time_sequence idx : %d ---------\n", idx);
  if (separate == false) {
    cams_data.param = cams_param[idx];
  } else {
    char str_idx[50];
    sprintf(str_idx, "/samples_info/sample%04d.yaml", idx);
    YAML::Node config_idx = YAML::LoadFile(data_infos_path + str_idx);
    cams_data.param = camParams(config_idx, n_img, cams_name);
  }
  imgs_data.clear();
  if (read_sample(cams_data.param.imgs_file, imgs_data)) {
    exit(1);
  }
#ifdef __HAVE_NVJPEG__
  nvdecoder.decode(imgs_data, imgs_dev);
#else
  decode_cpu(imgs_data, imgs_dev, img_w, img_h);
  printf("decode on cpu!\n");
#endif
  cams_data.imgs_dev = imgs_dev;
  return cams_data;
}

DataLoader::~DataLoader()
{
  CHECK_CUDA(cudaFree(imgs_dev));
}

int read_image(std::string & image_names, std::vector<char> & raw_data)
{
  std::ifstream input(image_names.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

  if (!(input.is_open())) {
    std::cerr << "Cannot open image: " << image_names << std::endl;
    return EXIT_FAILURE;
  }

  std::streamsize file_size = input.tellg();
  input.seekg(0, std::ios::beg);
  if (raw_data.size() < static_cast<size_t>(file_size)) {
    raw_data.resize(file_size);
  }
  if (!input.read(raw_data.data(), file_size)) {
    std::cerr << "Cannot read from file: " << image_names << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

int read_sample(std::vector<std::string> & imgs_file, std::vector<std::vector<char>> & imgs_data)
{
  imgs_data.resize(imgs_file.size());

  for (size_t i = 0; i < imgs_data.size(); i++) {
    if (read_image(imgs_file[i], imgs_data[i])) {
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}

Eigen::Translation3f fromYamlTrans(YAML::Node x)
{
  std::vector<float> trans = x.as<std::vector<float>>();
  return Eigen::Translation3f(trans[0], trans[1], trans[2]);
}

Eigen::Quaternion<float> fromYamlQuater(YAML::Node x)
{
  std::vector<float> quater = x.as<std::vector<float>>();
  return Eigen::Quaternion<float>(quater[0], quater[1], quater[2], quater[3]);
}

Eigen::Matrix3f fromYamlMatrix3f(YAML::Node x)
{
  std::vector<std::vector<float>> m = x.as<std::vector<std::vector<float>>>();
  Eigen::Matrix3f mat;
  for (size_t i = 0; i < m.size(); i++) {
    for (size_t j = 0; j < m[0].size(); j++) {
      mat(i, j) = m[i][j];
    }
  }
  return mat;
}
