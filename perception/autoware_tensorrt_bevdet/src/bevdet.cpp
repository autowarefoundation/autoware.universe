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

// cspell:ignore BEVDET, thre, TRTBEV, bevdet, caminfo, intrin, Ncams, bevfeat, bevpool, maxnum,
// zgrid cspell:ignore gridbev, egobev, adjgrid, currgrid
#include "autoware/tensorrt_bevdet/bevdet.hpp"

#include "autoware/tensorrt_bevdet/alignbev_plugin.hpp"
#include "autoware/tensorrt_bevdet/bevpool_plugin.hpp"
#include "autoware/tensorrt_bevdet/gatherbev_plugin.hpp"
#include "autoware/tensorrt_bevdet/preprocess_plugin.hpp"

#include <thrust/functional.h>
#include <thrust/sort.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>

using std::chrono::duration;
using std::chrono::high_resolution_clock;

namespace autoware::tensorrt_bevdet
{
BEVDet::BEVDet(
  const std::string & config_file, int n_img, std::vector<Eigen::Matrix3f> cams_intrin,
  std::vector<Eigen::Quaternion<float>> cams2ego_rot,
  std::vector<Eigen::Translation3f> cams2ego_trans, const std::string & onnx_file,
  const std::string & engine_file)
: cams_intrin_(cams_intrin), cams2ego_rot_(cams2ego_rot), cams2ego_trans_(cams2ego_trans)
{
  initParams(config_file);
  if (n_img != n_img_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BEVDet"), "BEVDet need %d images, but given %d images!", n_img_, n_img);
  }
  auto start = high_resolution_clock::now();

  std::shared_ptr<int> ranks_bev_ptr = nullptr;
  std::shared_ptr<int> ranks_depth_ptr = nullptr;
  std::shared_ptr<int> ranks_feat_ptr = nullptr;
  std::shared_ptr<int> interval_starts_ptr = nullptr;
  std::shared_ptr<int> interval_lengths_ptr = nullptr;

  initViewTransformer(
    ranks_bev_ptr, ranks_depth_ptr, ranks_feat_ptr, interval_starts_ptr, interval_lengths_ptr);
  auto end = high_resolution_clock::now();
  duration<float> t = end - start;
  RCLCPP_INFO(
    rclcpp::get_logger("BEVDet"), "InitVewTransformer cost time : %.4lf ms", t.count() * 1000);

  if (access(engine_file.c_str(), F_OK) == 0) {
    RCLCPP_INFO(rclcpp::get_logger("BEVDet"), "Inference engine prepared.");
  } else {
    // onnx to engine
    RCLCPP_WARN(
      rclcpp::get_logger("BEVDet"), "Could not find %s, try making TensorRT engine from onnx",
      engine_file.c_str());
    exportEngine(onnx_file, engine_file);
  }
  initEngine(engine_file);  // FIXME
  mallocDeviceMemory();

  if (use_adj_) {
    adj_frame_ptr_.reset(new AdjFrame(adj_num_, trt_buffer_sizes_[buffer_map_["curr_bevfeat"]]));
  }

  cam_params_host_ = new float[n_img_ * cam_params_size_];

  CHECK_CUDA(cudaMemcpy(
    trt_buffer_dev_[buffer_map_["ranks_bev"]], ranks_bev_ptr.get(), valid_feat_num_ * sizeof(int),
    cudaMemcpyHostToDevice));
  CHECK_CUDA(cudaMemcpy(
    trt_buffer_dev_[buffer_map_["ranks_depth"]], ranks_depth_ptr.get(),
    valid_feat_num_ * sizeof(int), cudaMemcpyHostToDevice));
  CHECK_CUDA(cudaMemcpy(
    trt_buffer_dev_[buffer_map_["ranks_feat"]], ranks_feat_ptr.get(), valid_feat_num_ * sizeof(int),
    cudaMemcpyHostToDevice));
  CHECK_CUDA(cudaMemcpy(
    trt_buffer_dev_[buffer_map_["interval_starts"]], interval_starts_ptr.get(),
    unique_bev_num_ * sizeof(int), cudaMemcpyHostToDevice));
  CHECK_CUDA(cudaMemcpy(
    trt_buffer_dev_[buffer_map_["interval_lengths"]], interval_lengths_ptr.get(),
    unique_bev_num_ * sizeof(int), cudaMemcpyHostToDevice));

  CHECK_CUDA(cudaMemcpy(
    trt_buffer_dev_[buffer_map_["mean"]], mean_.data(), 3 * sizeof(float), cudaMemcpyHostToDevice));
  CHECK_CUDA(cudaMemcpy(
    trt_buffer_dev_[buffer_map_["std"]], std_.data(), 3 * sizeof(float), cudaMemcpyHostToDevice));
}

void BEVDet::initCamParams(
  const std::vector<Eigen::Quaternion<float>> & curr_cams2ego_rot,
  const std::vector<Eigen::Translation3f> & curr_cams2ego_trans,
  const std::vector<Eigen::Matrix3f> & curr_cams_intrin)
{
  for (int i = 0; i < n_img_; i++) {
    cam_params_host_[i * cam_params_size_ + 0] = curr_cams_intrin[i](0, 0);
    cam_params_host_[i * cam_params_size_ + 1] = curr_cams_intrin[i](1, 1);
    cam_params_host_[i * cam_params_size_ + 2] = curr_cams_intrin[i](0, 2);
    cam_params_host_[i * cam_params_size_ + 3] = curr_cams_intrin[i](1, 2);
    cam_params_host_[i * cam_params_size_ + 4] = post_rot_(0, 0);
    cam_params_host_[i * cam_params_size_ + 5] = post_rot_(0, 1);
    cam_params_host_[i * cam_params_size_ + 6] = post_trans_.translation()(0);
    cam_params_host_[i * cam_params_size_ + 7] = post_rot_(1, 0);
    cam_params_host_[i * cam_params_size_ + 8] = post_rot_(1, 1);
    cam_params_host_[i * cam_params_size_ + 9] = post_trans_.translation()(1);
    cam_params_host_[i * cam_params_size_ + 10] = 1.f;  // bda 0 0
    cam_params_host_[i * cam_params_size_ + 11] = 0.f;  // bda 0 1
    cam_params_host_[i * cam_params_size_ + 12] = 0.f;  // bda 1 0
    cam_params_host_[i * cam_params_size_ + 13] = 1.f;  // bda 1 1
    cam_params_host_[i * cam_params_size_ + 14] = 1.f;  // bda 2 2
    cam_params_host_[i * cam_params_size_ + 15] = curr_cams2ego_rot[i].matrix()(0, 0);
    cam_params_host_[i * cam_params_size_ + 16] = curr_cams2ego_rot[i].matrix()(0, 1);
    cam_params_host_[i * cam_params_size_ + 17] = curr_cams2ego_rot[i].matrix()(0, 2);
    cam_params_host_[i * cam_params_size_ + 18] = curr_cams2ego_trans[i].translation()(0);
    cam_params_host_[i * cam_params_size_ + 19] = curr_cams2ego_rot[i].matrix()(1, 0);
    cam_params_host_[i * cam_params_size_ + 20] = curr_cams2ego_rot[i].matrix()(1, 1);
    cam_params_host_[i * cam_params_size_ + 21] = curr_cams2ego_rot[i].matrix()(1, 2);
    cam_params_host_[i * cam_params_size_ + 22] = curr_cams2ego_trans[i].translation()(1);
    cam_params_host_[i * cam_params_size_ + 23] = curr_cams2ego_rot[i].matrix()(2, 0);
    cam_params_host_[i * cam_params_size_ + 24] = curr_cams2ego_rot[i].matrix()(2, 1);
    cam_params_host_[i * cam_params_size_ + 25] = curr_cams2ego_rot[i].matrix()(2, 2);
    cam_params_host_[i * cam_params_size_ + 26] = curr_cams2ego_trans[i].translation()(2);
  }
  CHECK_CUDA(cudaMemcpy(
    trt_buffer_dev_[buffer_map_["cam_params"]], cam_params_host_,
    trt_buffer_sizes_[buffer_map_["cam_params"]], cudaMemcpyHostToDevice));
}

void BEVDet::initParams(const std::string & config_file)
{
  mean_ = std::vector<float>(3);
  std_ = std::vector<float>(3);

  YAML::Node model_config = YAML::LoadFile(config_file);
  n_img_ = model_config["data_config"]["Ncams"].as<int>();
  src_img_h_ = model_config["data_config"]["src_size"][0].as<int>();
  src_img_w_ = model_config["data_config"]["src_size"][1].as<int>();
  input_img_h_ = model_config["data_config"]["input_size"][0].as<int>();
  input_img_w_ = model_config["data_config"]["input_size"][1].as<int>();
  resize_radio_ = model_config["data_config"]["resize_radio"].as<float>();
  crop_h_ = model_config["data_config"]["crop"][0].as<int>();
  crop_w_ = model_config["data_config"]["crop"][1].as<int>();
  mean_[0] = model_config["mean"][0].as<float>();
  mean_[1] = model_config["mean"][1].as<float>();
  mean_[2] = model_config["mean"][2].as<float>();
  std_[0] = model_config["std"][0].as<float>();
  std_[1] = model_config["std"][1].as<float>();
  std_[2] = model_config["std"][2].as<float>();
  down_sample_ = model_config["model"]["down_sample"].as<int>();
  depth_start_ = model_config["grid_config"]["depth"][0].as<float>();
  depth_end_ = model_config["grid_config"]["depth"][1].as<float>();
  depth_step_ = model_config["grid_config"]["depth"][2].as<float>();
  x_start_ = model_config["grid_config"]["x"][0].as<float>();
  x_end_ = model_config["grid_config"]["x"][1].as<float>();
  x_step_ = model_config["grid_config"]["x"][2].as<float>();
  y_start_ = model_config["grid_config"]["y"][0].as<float>();
  y_end_ = model_config["grid_config"]["y"][1].as<float>();
  y_step_ = model_config["grid_config"]["y"][2].as<float>();
  z_start_ = model_config["grid_config"]["z"][0].as<float>();
  z_end_ = model_config["grid_config"]["z"][1].as<float>();
  z_step_ = model_config["grid_config"]["z"][2].as<float>();
  bevpool_channel_ = model_config["model"]["bevpool_channels"].as<int>();
  nms_pre_maxnum_ = model_config["test_cfg"]["max_per_img"].as<int>();
  nms_post_maxnum_ = model_config["test_cfg"]["post_max_size"].as<int>();
  score_thresh_ = model_config["test_cfg"]["score_threshold"].as<float>();
  nms_overlap_thresh_ = model_config["test_cfg"]["nms_thr"][0].as<float>();
  use_depth_ = model_config["use_depth"].as<bool>();
  use_adj_ = model_config["use_adj"].as<bool>();
  transform_size_ = model_config["transform_size"].as<int>();
  cam_params_size_ = model_config["cam_params_size"].as<int>();

  std::vector<std::vector<float>> nms_factor_temp =
    model_config["test_cfg"]["nms_rescale_factor"].as<std::vector<std::vector<float>>>();
  nms_rescale_factor_.clear();
  for (const auto & task_factors : nms_factor_temp) {
    for (float factor : task_factors) {
      nms_rescale_factor_.push_back(factor);
    }
  }

  std::vector<std::vector<std::string>> class_name_pre_task;
  class_num_ = 0;
  YAML::Node tasks = model_config["model"]["tasks"];
  class_num_pre_task_ = std::vector<int>();
  for (auto it : tasks) {
    int num = it["num_class"].as<int>();
    class_num_pre_task_.push_back(num);
    class_num_ += num;
    class_name_pre_task.push_back(it["class_names"].as<std::vector<std::string>>());
  }

  YAML::Node common_head_channel = model_config["model"]["common_head"]["channels"];
  YAML::Node common_head_name = model_config["model"]["common_head"]["names"];
  for (size_t i = 0; i < common_head_channel.size(); i++) {
    out_num_task_head_[common_head_name[i].as<std::string>()] = common_head_channel[i].as<int>();
  }

  feat_h_ = input_img_h_ / down_sample_;
  feat_w_ = input_img_w_ / down_sample_;
  depth_num_ = (depth_end_ - depth_start_) / depth_step_;
  xgrid_num_ = (x_end_ - x_start_) / x_step_;
  ygrid_num_ = (y_end_ - y_start_) / y_step_;
  zgrid_num_ = (z_end_ - z_start_) / z_step_;
  bev_h_ = ygrid_num_;
  bev_w_ = xgrid_num_;

  post_rot_ << resize_radio_, 0, 0, 0, resize_radio_, 0, 0, 0, 1;
  post_trans_.translation() << -crop_w_, -crop_h_, 0;

  adj_num_ = 0;
  if (use_adj_) {
    adj_num_ = model_config["adj_num"].as<int>();
  }

  postprocess_ptr_.reset(new PostprocessGPU(
    class_num_, score_thresh_, nms_overlap_thresh_, nms_pre_maxnum_, nms_post_maxnum_, down_sample_,
    bev_h_, bev_w_, x_step_, y_step_, x_start_, y_start_, class_num_pre_task_,
    nms_rescale_factor_));
}

void BEVDet::mallocDeviceMemory()
{
  trt_buffer_sizes_.resize(trt_engine_->getNbBindings());
  trt_buffer_dev_ = reinterpret_cast<void **>(new void *[trt_engine_->getNbBindings()]);
  for (int i = 0; i < trt_engine_->getNbBindings(); i++) {
    nvinfer1::Dims32 dim = trt_context_->getBindingDimensions(i);
    size_t size = 1;
    for (int j = 0; j < dim.nbDims; j++) {
      size *= dim.d[j];
    }
    size *= dataTypeToSize(trt_engine_->getBindingDataType(i));
    trt_buffer_sizes_[i] = size;
    CHECK_CUDA(cudaMalloc(&trt_buffer_dev_[i], size));
  }

  RCLCPP_INFO(rclcpp::get_logger("BEVDet"), "img num binding : %d", trt_engine_->getNbBindings());

  post_buffer_ = reinterpret_cast<void **>(new void *[class_num_pre_task_.size() * 6]);
  for (size_t i = 0; i < class_num_pre_task_.size(); i++) {
    post_buffer_[i * 6 + 0] = trt_buffer_dev_[buffer_map_["reg_" + std::to_string(i)]];
    post_buffer_[i * 6 + 1] = trt_buffer_dev_[buffer_map_["height_" + std::to_string(i)]];
    post_buffer_[i * 6 + 2] = trt_buffer_dev_[buffer_map_["dim_" + std::to_string(i)]];
    post_buffer_[i * 6 + 3] = trt_buffer_dev_[buffer_map_["rot_" + std::to_string(i)]];
    post_buffer_[i * 6 + 4] = trt_buffer_dev_[buffer_map_["vel_" + std::to_string(i)]];
    post_buffer_[i * 6 + 5] = trt_buffer_dev_[buffer_map_["heatmap_" + std::to_string(i)]];
  }

  return;
}

void BEVDet::initViewTransformer(
  std::shared_ptr<int> & ranks_bev_ptr, std::shared_ptr<int> & ranks_depth_ptr,
  std::shared_ptr<int> & ranks_feat_ptr, std::shared_ptr<int> & interval_starts_ptr,
  std::shared_ptr<int> & interval_lengths_ptr)
{
  int num_points = n_img_ * depth_num_ * feat_h_ * feat_w_;
  Eigen::Vector3f * frustum = new Eigen::Vector3f[num_points];

  for (int i = 0; i < n_img_; i++) {
    for (int d_ = 0; d_ < depth_num_; d_++) {
      for (int h_ = 0; h_ < feat_h_; h_++) {
        for (int w_ = 0; w_ < feat_w_; w_++) {
          int offset =
            i * depth_num_ * feat_h_ * feat_w_ + d_ * feat_h_ * feat_w_ + h_ * feat_w_ + w_;
          (frustum + offset)->x() = static_cast<float>(w_) * (input_img_w_ - 1) / (feat_w_ - 1);
          (frustum + offset)->y() = static_cast<float>(h_) * (input_img_h_ - 1) / (feat_h_ - 1);
          (frustum + offset)->z() = static_cast<float>(d_) * depth_step_ + depth_start_;

          // eliminate post transformation
          *(frustum + offset) -= post_trans_.translation();
          *(frustum + offset) = post_rot_.inverse() * *(frustum + offset);
          //
          (frustum + offset)->x() *= (frustum + offset)->z();
          (frustum + offset)->y() *= (frustum + offset)->z();
          // img to ego -> rot -> trans
          *(frustum + offset) = cams2ego_rot_[i] * cams_intrin_[i].inverse() * *(frustum + offset) +
                                cams2ego_trans_[i].translation();

          // voxelization
          *(frustum + offset) -= Eigen::Vector3f(x_start_, y_start_, z_start_);
          (frustum + offset)->x() = static_cast<int>((frustum + offset)->x() / x_step_);
          (frustum + offset)->y() = static_cast<int>((frustum + offset)->y() / y_step_);
          (frustum + offset)->z() = static_cast<int>((frustum + offset)->z() / z_step_);
        }
      }
    }
  }

  int * _ranks_depth = new int[num_points];
  int * _ranks_feat = new int[num_points];

  for (int i = 0; i < num_points; i++) {
    _ranks_depth[i] = i;
  }
  for (int i = 0; i < n_img_; i++) {
    for (int d_ = 0; d_ < depth_num_; d_++) {
      for (int u = 0; u < feat_h_ * feat_w_; u++) {
        int offset = i * (depth_num_ * feat_h_ * feat_w_) + d_ * (feat_h_ * feat_w_) + u;
        _ranks_feat[offset] = i * feat_h_ * feat_w_ + u;
      }
    }
  }

  std::vector<int> kept;
  for (int i = 0; i < num_points; i++) {
    if (
      static_cast<int>((frustum + i)->x()) >= 0 &&
      static_cast<int>((frustum + i)->x()) < xgrid_num_ &&
      static_cast<int>((frustum + i)->y()) >= 0 &&
      static_cast<int>((frustum + i)->y()) < ygrid_num_ &&
      static_cast<int>((frustum + i)->z()) >= 0 &&
      static_cast<int>((frustum + i)->z()) < zgrid_num_) {
      kept.push_back(i);
    }
  }

  valid_feat_num_ = kept.size();
  int * ranks_depth_host = new int[valid_feat_num_];
  int * ranks_feat_host = new int[valid_feat_num_];
  int * ranks_bev_host = new int[valid_feat_num_];
  int * order = new int[valid_feat_num_];

  for (int i = 0; i < valid_feat_num_; i++) {
    Eigen::Vector3f & p = frustum[kept[i]];
    ranks_bev_host[i] = static_cast<int>(p.z()) * xgrid_num_ * ygrid_num_ +
                        static_cast<int>(p.y()) * xgrid_num_ + static_cast<int>(p.x());
    order[i] = i;
  }

  thrust::sort_by_key(ranks_bev_host, ranks_bev_host + valid_feat_num_, order);
  for (int i = 0; i < valid_feat_num_; i++) {
    ranks_depth_host[i] = _ranks_depth[kept[order[i]]];
    ranks_feat_host[i] = _ranks_feat[kept[order[i]]];
  }

  delete[] _ranks_depth;
  delete[] _ranks_feat;
  delete[] frustum;
  delete[] order;

  std::vector<int> interval_starts_host;
  std::vector<int> interval_lengths_host;

  interval_starts_host.push_back(0);
  int len = 1;
  for (int i = 1; i < valid_feat_num_; i++) {
    if (ranks_bev_host[i] != ranks_bev_host[i - 1]) {
      interval_starts_host.push_back(i);
      interval_lengths_host.push_back(len);
      len = 1;
    } else {
      len++;
    }
  }

  interval_lengths_host.push_back(len);
  unique_bev_num_ = interval_lengths_host.size();

  int * interval_starts_host_ptr = new int[interval_starts_host.size()];
  int * interval_lengths_host_ptr = new int[interval_lengths_host.size()];

  memcpy(
    interval_starts_host_ptr, interval_starts_host.data(),
    interval_starts_host.size() * sizeof(int));
  memcpy(
    interval_lengths_host_ptr, interval_lengths_host.data(),
    interval_lengths_host.size() * sizeof(int));

  ranks_bev_ptr.reset(ranks_bev_host);
  ranks_depth_ptr.reset(ranks_depth_host);
  ranks_feat_ptr.reset(ranks_feat_host);
  interval_starts_ptr.reset(interval_starts_host_ptr);
  interval_lengths_ptr.reset(interval_lengths_host_ptr);

  RCLCPP_INFO(rclcpp::get_logger("BEVDet"), "valid_feat_num: %d", valid_feat_num_);
  RCLCPP_INFO(rclcpp::get_logger("BEVDet"), "unique_bev_num: %d", unique_bev_num_);
}

void print_dim(nvinfer1::Dims dim, const std::string & name)
{
  std::ostringstream oss;
  oss << name << " : ";
  for (auto i = 0; i < dim.nbDims; i++) {
    oss << dim.d[i] << ' ';
  }
  RCLCPP_INFO(rclcpp::get_logger("BEVDet"), "%s", oss.str().c_str());
}

int BEVDet::initEngine(const std::string & engine_file)
{
  if (deserializeTRTEngine(engine_file, &trt_engine_)) {
    return EXIT_FAILURE;
  }

  if (trt_engine_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("BEVDet"), "Failed to deserialize engine file!");
    return EXIT_FAILURE;
  }
  trt_context_ = trt_engine_->createExecutionContext();

  if (trt_context_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("BEVDet"), "Failed to create TensorRT Execution Context!");
    return EXIT_FAILURE;
  }

  // set bindings
  std::vector<nvinfer1::Dims32> shapes{
    {4, {n_img_, 3, src_img_h_, src_img_w_ / 4}},
    {1, {3}},
    {1, {3}},
    {3, {1, n_img_, cam_params_size_}},
    {1, {valid_feat_num_}},
    {1, {valid_feat_num_}},
    {1, {valid_feat_num_}},
    {1, {unique_bev_num_}},
    {1, {unique_bev_num_}},
    {5, {1, adj_num_, bevpool_channel_, bev_h_, bev_w_}},
    {3, {1, adj_num_, transform_size_}},
    {2, {1, 1}}};

  for (size_t i = 0; i < shapes.size(); i++) {
    trt_context_->setBindingDimensions(i, shapes[i]);
  }

  buffer_map_.clear();
  for (auto i = 0; i < trt_engine_->getNbBindings(); i++) {
    auto dim = trt_context_->getBindingDimensions(i);
    auto name = trt_engine_->getBindingName(i);
    buffer_map_[name] = i;
    print_dim(dim, name);
  }

  return EXIT_SUCCESS;
}

int BEVDet::deserializeTRTEngine(
  const std::string & engine_file, nvinfer1::ICudaEngine ** engine_ptr)
{
  std::stringstream engine_stream;
  engine_stream.seekg(0, engine_stream.beg);

  std::ifstream file(engine_file);
  engine_stream << file.rdbuf();
  file.close();

  nvinfer1::IRuntime * runtime = nvinfer1::createInferRuntime(g_logger_);
  if (runtime == nullptr) {
    std::string msg("Failed to build runtime parser!");
    g_logger_.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
    return EXIT_FAILURE;
  }
  engine_stream.seekg(0, std::ios::end);
  const int engine_size = engine_stream.tellg();

  engine_stream.seekg(0, std::ios::beg);
  void * engine_str = malloc(engine_size);
  engine_stream.read(reinterpret_cast<char *>(engine_str), engine_size);

  nvinfer1::ICudaEngine * engine = runtime->deserializeCudaEngine(engine_str, engine_size, NULL);
  if (engine == nullptr) {
    std::string msg("Failed to build engine parser!");
    g_logger_.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
    return EXIT_FAILURE;
  }
  *engine_ptr = engine;
  for (int bi = 0; bi < engine->getNbBindings(); bi++) {
    if (engine->bindingIsInput(bi) == true) {
      RCLCPP_INFO(
        rclcpp::get_logger("BEVDet"), "Binding %d (%s): Input.", bi, engine->getBindingName(bi));
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("BEVDet"), "Binding %d (%s): Output.", bi, engine->getBindingName(bi));
    }
  }
  return EXIT_SUCCESS;
}

void BEVDet::getAdjBEVFeature(
  const std::string & curr_scene_token, const Eigen::Quaternion<float> & ego2global_rot,
  const Eigen::Translation3f & ego2global_trans)
{
  int flag = 1;
  if (adj_frame_ptr_->lastScenesToken() != curr_scene_token) {
    adj_frame_ptr_->reset();
    flag = 0;
  }

  // the smaller the idx, the newer th adj_bevfeat
  for (int i = 0; i < adj_num_; i++) {
    const void * adj_buffer = adj_frame_ptr_->getFrameBuffer(i);

    size_t buf_size = trt_buffer_sizes_[buffer_map_["adj_feats"]] / adj_num_;

    CHECK_CUDA(cudaMemcpy(
      reinterpret_cast<char *>(trt_buffer_dev_[buffer_map_["adj_feats"]]) + i * buf_size,
      adj_buffer, buf_size, cudaMemcpyDeviceToDevice));

    Eigen::Quaternion<float> adj_ego2global_rot;
    Eigen::Translation3f adj_ego2global_trans;
    adj_frame_ptr_->getEgo2Global(i, adj_ego2global_rot, adj_ego2global_trans);

    getCurr2AdjTransform(
      ego2global_rot, adj_ego2global_rot, ego2global_trans, adj_ego2global_trans,
      reinterpret_cast<float *>(trt_buffer_dev_[buffer_map_["transforms"]]) + i * transform_size_);
  }
  CHECK_CUDA(cudaMemcpy(
    trt_buffer_dev_[buffer_map_["flag"]], &flag, trt_buffer_sizes_[buffer_map_["flag"]],
    cudaMemcpyHostToDevice));
}

void BEVDet::getCurr2AdjTransform(
  const Eigen::Quaternion<float> & curr_ego2global_rot,
  const Eigen::Quaternion<float> & adj_ego2global_rot,
  const Eigen::Translation3f & curr_ego2global_trans,
  const Eigen::Translation3f & adj_ego2global_trans, float * transform_dev)
{
  Eigen::Matrix4f curr_e2g_transform;
  Eigen::Matrix4f adj_e2g_transform;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      curr_e2g_transform(i, j) = curr_ego2global_rot.matrix()(i, j);
      adj_e2g_transform(i, j) = adj_ego2global_rot.matrix()(i, j);
    }
  }
  for (int i = 0; i < 3; i++) {
    curr_e2g_transform(i, 3) = curr_ego2global_trans.vector()(i);
    adj_e2g_transform(i, 3) = adj_ego2global_trans.vector()(i);

    curr_e2g_transform(3, i) = 0.f;
    adj_e2g_transform(3, i) = 0.f;
  }
  curr_e2g_transform(3, 3) = 1.f;
  adj_e2g_transform(3, 3) = 1.f;

  Eigen::Matrix4f currEgo2adjEgo = adj_e2g_transform.inverse() * curr_e2g_transform;
  Eigen::Matrix3f currEgo2adjEgo_2d;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      currEgo2adjEgo_2d(i, j) = currEgo2adjEgo(i, j);
    }
  }
  currEgo2adjEgo_2d(2, 0) = 0.f;
  currEgo2adjEgo_2d(2, 1) = 0.f;
  currEgo2adjEgo_2d(2, 2) = 1.f;
  currEgo2adjEgo_2d(0, 2) = currEgo2adjEgo(0, 3);
  currEgo2adjEgo_2d(1, 2) = currEgo2adjEgo(1, 3);

  Eigen::Matrix3f gridbev2egobev;
  gridbev2egobev(0, 0) = x_step_;
  gridbev2egobev(1, 1) = y_step_;
  gridbev2egobev(0, 2) = x_start_;
  gridbev2egobev(1, 2) = y_start_;
  gridbev2egobev(2, 2) = 1.f;

  gridbev2egobev(0, 1) = 0.f;
  gridbev2egobev(1, 0) = 0.f;
  gridbev2egobev(2, 0) = 0.f;
  gridbev2egobev(2, 1) = 0.f;

  Eigen::Matrix3f currgrid2adjgrid = gridbev2egobev.inverse() * currEgo2adjEgo_2d * gridbev2egobev;

  CHECK_CUDA(cudaMemcpy(
    transform_dev, Eigen::Matrix3f(currgrid2adjgrid.transpose()).data(),
    transform_size_ * sizeof(float), cudaMemcpyHostToDevice));
}

int BEVDet::doInfer(
  const camsData & cam_data, std::vector<Box> & out_detections, float & cost_time, int idx)
{
  RCLCPP_DEBUG(rclcpp::get_logger("BEVDet"), "---------%d---------", idx + 1);

  auto start = high_resolution_clock::now();
  CHECK_CUDA(cudaMemcpy(
    trt_buffer_dev_[buffer_map_["images"]], cam_data.imgs_dev,
    trt_buffer_sizes_[buffer_map_["images"]], cudaMemcpyDeviceToDevice));

  initCamParams(
    cam_data.param.cams2ego_rot, cam_data.param.cams2ego_trans, cam_data.param.cams_intrin);

  getAdjBEVFeature(
    cam_data.param.scene_token, cam_data.param.ego2global_rot, cam_data.param.ego2global_trans);

  if (!trt_context_->executeV2(trt_buffer_dev_)) {
    RCLCPP_ERROR(rclcpp::get_logger("BEVDet"), "BEVDet forward failing!");
  }

  adj_frame_ptr_->saveFrameBuffer(
    trt_buffer_dev_[buffer_map_["curr_bevfeat"]], cam_data.param.scene_token,
    cam_data.param.ego2global_rot, cam_data.param.ego2global_trans);

  auto end = high_resolution_clock::now();

  postprocess_ptr_->DoPostprocess(post_buffer_, out_detections);
  CHECK_CUDA(cudaDeviceSynchronize());

  auto post_end = high_resolution_clock::now();

  duration<double> infer_t = post_end - start;
  duration<double> engine_t = end - start;
  duration<double> post_t = post_end - end;

  cost_time = infer_t.count() * 1000;
  RCLCPP_DEBUG(rclcpp::get_logger("BEVDet"), "Inference time: %.5lf ms", engine_t.count() * 1000);
  RCLCPP_DEBUG(rclcpp::get_logger("BEVDet"), "Postprocess time: %.5lf ms", post_t.count() * 1000);
  RCLCPP_DEBUG(rclcpp::get_logger("BEVDet"), "Total time: %.5lf ms", infer_t.count() * 1000);
  RCLCPP_DEBUG(rclcpp::get_logger("BEVDet"), "Detect %ld objects", out_detections.size());

  return EXIT_SUCCESS;
}

void BEVDet::exportEngine(const std::string & onnxFile, const std::string & trtFile)
{
  CHECK_CUDA(cudaSetDevice(0));
  nvinfer1::ICudaEngine * engine = nullptr;
  std::vector<nvinfer1::Dims32> min_shapes{
    {4, {6, 3, 900, 400}}, {1, {3}},      {1, {3}},    {3, {1, 6, 27}}, {1, {200000}},
    {1, {200000}},         {1, {200000}}, {1, {8000}}, {1, {8000}},     {5, {1, 8, 80, 128, 128}},
    {3, {1, 8, 6}},        {2, {1, 1}}};

  std::vector<nvinfer1::Dims32> opt_shapes{
    {4, {6, 3, 900, 400}}, {1, {3}},      {1, {3}},     {3, {1, 6, 27}}, {1, {356760}},
    {1, {356760}},         {1, {356760}}, {1, {13360}}, {1, {13360}},    {5, {1, 8, 80, 128, 128}},
    {3, {1, 8, 6}},        {2, {1, 1}}};

  std::vector<nvinfer1::Dims32> max_shapes{
    {4, {6, 3, 900, 400}}, {1, {3}},      {1, {3}},     {3, {1, 6, 27}}, {1, {370000}},
    {1, {370000}},         {1, {370000}}, {1, {14000}}, {1, {14000}},    {5, {1, 8, 80, 128, 128}},
    {3, {1, 8, 6}},        {2, {1, 1}}};
  nvinfer1::IBuilder * builder = nvinfer1::createInferBuilder(g_logger_);
  nvinfer1::INetworkDefinition * network =
    builder->createNetworkV2(1U << int(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));
  nvinfer1::IOptimizationProfile * profile = builder->createOptimizationProfile();
  nvinfer1::IBuilderConfig * config = builder->createBuilderConfig();
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 3UL << 32UL);

  config->setFlag(nvinfer1::BuilderFlag::kFP16);

  nvonnxparser::IParser * parser = nvonnxparser::createParser(*network, g_logger_);

  if (!parser->parseFromFile(onnxFile.c_str(), static_cast<int>(g_logger_.reportable_severity))) {
    RCLCPP_ERROR(rclcpp::get_logger("BEVDet"), "Failed parsing .onnx file!");
    for (int i = 0; i < parser->getNbErrors(); ++i) {
      auto * error = parser->getError(i);
      RCLCPP_ERROR(
        rclcpp::get_logger("BEVDet"), "Error code: %d, Description: %s",
        static_cast<int>(error->code()), error->desc());
    }

    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("BEVDet"), "Succeeded parsing .onnx file!");

  for (size_t i = 0; i < min_shapes.size(); i++) {
    nvinfer1::ITensor * it = network->getInput(i);
    profile->setDimensions(it->getName(), nvinfer1::OptProfileSelector::kMIN, min_shapes[i]);
    profile->setDimensions(it->getName(), nvinfer1::OptProfileSelector::kOPT, opt_shapes[i]);
    profile->setDimensions(it->getName(), nvinfer1::OptProfileSelector::kMAX, max_shapes[i]);
  }
  config->addOptimizationProfile(profile);

  nvinfer1::IHostMemory * engineString = builder->buildSerializedNetwork(*network, *config);
  if (engineString == nullptr || engineString->size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("BEVDet"), "Failed building serialized engine!");
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("BEVDet"), "Succeeded building serialized engine!");

  nvinfer1::IRuntime * runtime{nvinfer1::createInferRuntime(g_logger_)};
  engine = runtime->deserializeCudaEngine(engineString->data(), engineString->size());
  if (engine == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("BEVDet"), "Failed building engine!");
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("BEVDet"), "Succeeded building engine!");

  std::ofstream engineFile(trtFile, std::ios::binary);
  if (!engineFile) {
    RCLCPP_ERROR(rclcpp::get_logger("BEVDet"), "Failed opening file to write");
    return;
  }
  engineFile.write(static_cast<char *>(engineString->data()), engineString->size());
  if (engineFile.fail()) {
    RCLCPP_ERROR(rclcpp::get_logger("BEVDet"), "Failed saving .engine file!");
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("BEVDet"), "Succeeded saving .engine file!");
}

BEVDet::~BEVDet()
{
  for (int i = 0; i < trt_engine_->getNbBindings(); i++) {
    CHECK_CUDA(cudaFree(trt_buffer_dev_[i]));
  }
  delete[] trt_buffer_dev_;
  delete[] post_buffer_;

  delete[] cam_params_host_;

  trt_context_->destroy();
  trt_engine_->destroy();
}
}  // namespace autoware::tensorrt_bevdet
