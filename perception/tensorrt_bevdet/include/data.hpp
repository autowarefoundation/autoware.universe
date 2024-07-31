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
#ifndef DATA_HPP_
#define DATA_HPP_

#include "common.hpp"
#include "nvjpegdecoder.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <yaml-cpp/yaml.h>

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

class DataLoader
{
public:
  DataLoader() = default;
  DataLoader(
    int _n_img, int _h, int _w, const std::string & _data_infos_path,
    const std::vector<std::string> & _cams_name, bool _sep = true);

  const std::vector<Eigen::Matrix3f> & get_cams_intrin() const { return cams_intrin; }
  const std::vector<Eigen::Quaternion<float>> & get_cams2ego_rot() const { return cams2ego_rot; }
  const std::vector<Eigen::Translation3f> & get_cams2ego_trans() const { return cams2ego_trans; }

  const Eigen::Quaternion<float> & get_lidar2ego_rot() const { return lidar2ego_rot; }

  const Eigen::Translation3f & get_lidar2ego_trans() const { return lidar2ego_trans; }

  int size() { return sample_num; }

  const camsData & data(int idx, bool time_order = true);
  ~DataLoader();

private:
  std::vector<int> time_sequence;
  std::string data_infos_path;
  int sample_num;

  std::vector<std::string> cams_name;
  int n_img;
  int img_h;
  int img_w;

  std::vector<camParams> cams_param;
  camsData cams_data;

  std::vector<Eigen::Matrix3f> cams_intrin;
  std::vector<Eigen::Quaternion<float>> cams2ego_rot;
  std::vector<Eigen::Translation3f> cams2ego_trans;
  Eigen::Quaternion<float> lidar2ego_rot;
  Eigen::Translation3f lidar2ego_trans;

#ifdef __HAVE_NVJPEG__
  nvjpegDecoder nvdecoder;
#endif
  uchar * imgs_dev;
  std::vector<std::vector<char>> imgs_data;
  bool separate;
};

Eigen::Translation3f fromYamlTrans(YAML::Node x);
Eigen::Quaternion<float> fromYamlQuater(YAML::Node x);
Eigen::Matrix3f fromYamlMatrix3f(YAML::Node x);

int read_image(std::string & image_names, std::vector<char> & raw_data);

int read_sample(std::vector<std::string> & imgs_file, std::vector<std::vector<char>> & imgs_data);

#endif  // DATA_HPP_
