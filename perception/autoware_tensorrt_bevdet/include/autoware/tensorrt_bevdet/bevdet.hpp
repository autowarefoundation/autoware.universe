// Copyright 2024 AutoCore, Inc.
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
#ifndef AUTOWARE__TENSORRT_BEVDET__BEVDET_HPP_
#define AUTOWARE__TENSORRT_BEVDET__BEVDET_HPP_

#include "NvInfer.h"
#include "common.hpp"
#include "data.hpp"
#include "postprocess.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <NvOnnxParser.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::tensorrt_bevdet
{
class AdjFrame
{
public:
  AdjFrame() {}
  AdjFrame(int n, size_t buf_size)
  : n_(n), buf_size_(buf_size), scenes_token_(n), ego2global_rot_(n), ego2global_trans_(n)
  {
    CHECK_CUDA(cudaMalloc(reinterpret_cast<void **>(&adj_buffer), n * buf_size));
    CHECK_CUDA(cudaMemset(adj_buffer, 0, n * buf_size));
    last_ = 0;
    buffer_num_ = 0;
    init_ = false;

    for (auto & rot : ego2global_rot_) {
      rot = Eigen::Quaternion<float>(0.f, 0.f, 0.f, 0.f);
    }
    for (auto & trans : ego2global_trans_) {
      trans = Eigen::Translation3f(0.f, 0.f, 0.f);
    }
  }
  const std::string & lastScenesToken() const { return scenes_token_[last_]; }

  void reset()
  {
    last_ = 0;  // origin -1
    buffer_num_ = 0;
    init_ = false;
  }

  void saveFrameBuffer(
    const void * curr_buffer, const std::string & curr_token,
    const Eigen::Quaternion<float> & ego2global_rot, const Eigen::Translation3f & ego2global_trans)
  {
    int iters = init_ ? 1 : n_;
    while (iters--) {
      last_ = (last_ + 1) % n_;
      CHECK_CUDA(cudaMemcpy(
        reinterpret_cast<char *>(adj_buffer) + last_ * buf_size_, curr_buffer, buf_size_,
        cudaMemcpyDeviceToDevice));
      scenes_token_[last_] = curr_token;
      ego2global_rot_[last_] = ego2global_rot;
      ego2global_trans_[last_] = ego2global_trans;
      buffer_num_ = std::min(buffer_num_ + 1, n_);
    }
    init_ = true;
  }
  int havingBuffer(int idx) { return static_cast<int>(idx < buffer_num_); }

  const void * getFrameBuffer(int idx)
  {
    idx = (-idx + last_ + n_) % n_;
    return reinterpret_cast<char *>(adj_buffer) + idx * buf_size_;
  }
  void getEgo2Global(
    int idx, Eigen::Quaternion<float> & adj_ego2global_rot,
    Eigen::Translation3f & adj_ego2global_trans)
  {
    idx = (-idx + last_ + n_) % n_;
    adj_ego2global_rot = ego2global_rot_[idx];
    adj_ego2global_trans = ego2global_trans_[idx];
  }

  ~AdjFrame() { CHECK_CUDA(cudaFree(adj_buffer)); }

private:
  int n_;
  size_t buf_size_;

  int last_;
  int buffer_num_;
  bool init_;

  std::vector<std::string> scenes_token_;
  std::vector<Eigen::Quaternion<float>> ego2global_rot_;
  std::vector<Eigen::Translation3f> ego2global_trans_;

  void * adj_buffer;
};

class BEVDet
{
public:
  BEVDet() {}
  BEVDet(
    const std::string & config_file, int n_img, std::vector<Eigen::Matrix3f> _cams_intrin,
    std::vector<Eigen::Quaternion<float>> _cams2ego_rot,
    std::vector<Eigen::Translation3f> _cams2ego_trans, const std::string & onnx_file,
    const std::string & engine_file);

  int doInfer(
    const camsData & cam_data, std::vector<Box> & out_detections, float & cost_time, int idx = -1);
  ~BEVDet();

protected:
  void initParams(const std::string & config_file);

  void initViewTransformer(
    std::shared_ptr<int> & ranks_bev_ptr, std::shared_ptr<int> & ranks_depth_ptr,
    std::shared_ptr<int> & ranks_feat_ptr, std::shared_ptr<int> & interval_starts_ptr,
    std::shared_ptr<int> & interval_lengths_ptr);
  void exportEngine(const std::string & onnxFile, const std::string & trtFile);
  int initEngine(const std::string & engine_file);

  int deserializeTRTEngine(const std::string & engine_file, nvinfer1::ICudaEngine ** engine_ptr);

  void mallocDeviceMemory();

  void initCamParams(
    const std::vector<Eigen::Quaternion<float>> & curr_cams2ego_rot,
    const std::vector<Eigen::Translation3f> & curr_cams2ego_trans,
    const std::vector<Eigen::Matrix3f> & cams_intrin);

  void getAdjBEVFeature(
    const std::string & curr_scene_token, const Eigen::Quaternion<float> & ego2global_rot,
    const Eigen::Translation3f & ego2global_trans);

  void getCurr2AdjTransform(
    const Eigen::Quaternion<float> & curr_ego2global_rot,
    const Eigen::Quaternion<float> & adj_ego2global_rot,
    const Eigen::Translation3f & curr_ego2global_trans,
    const Eigen::Translation3f & adj_ego2global_trans, float * transform_dev);

private:
  int N_img;

  int src_img_h;
  int src_img_w;
  int input_img_h;
  int input_img_w;
  int crop_h;
  int crop_w;
  float resize_radio;
  int down_sample;
  int feat_h;
  int feat_w;
  int bev_h;
  int bev_w;
  int bevpool_channel;

  float depth_start;
  float depth_end;
  float depth_step;
  int depth_num;

  float x_start;
  float x_end;
  float x_step;
  int xgrid_num;

  float y_start;
  float y_end;
  float y_step;
  int ygrid_num;

  float z_start;
  float z_end;
  float z_step;
  int zgrid_num;

  std::vector<float> mean;
  std::vector<float> std;

  bool use_depth;
  bool use_adj;
  int adj_num;

  int class_num;
  float score_thresh;
  float nms_overlap_thresh;
  int nms_pre_maxnum;
  int nms_post_maxnum;
  std::vector<float> nms_rescale_factor;
  std::vector<int> class_num_pre_task;
  std::map<std::string, int> out_num_task_head;

  std::vector<Eigen::Matrix3f> cams_intrin;
  std::vector<Eigen::Quaternion<float>> cams2ego_rot;
  std::vector<Eigen::Translation3f> cams2ego_trans;

  Eigen::Matrix3f post_rot;
  Eigen::Translation3f post_trans;

  std::vector<size_t> trt_buffer_sizes;
  void ** trt_buffer_dev;
  float * cam_params_host;
  void ** post_buffer;

  std::map<std::string, int> buffer_map;

  int valid_feat_num;
  int unique_bev_num;

  int transform_size;
  int cam_params_size;

  Logger g_logger;

  nvinfer1::ICudaEngine * trt_engine;
  nvinfer1::IExecutionContext * trt_context;

  std::unique_ptr<PostprocessGPU> postprocess_ptr;
  std::unique_ptr<AdjFrame> adj_frame_ptr;
};
}  // namespace autoware::tensorrt_bevdet
#endif  // AUTOWARE__TENSORRT_BEVDET__BEVDET_HPP_
