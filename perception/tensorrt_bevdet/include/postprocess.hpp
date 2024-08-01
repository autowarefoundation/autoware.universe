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
#ifndef POSTPROCESS_HPP_
#define POSTPROCESS_HPP_

#include "iou3d_nms.hpp"
#include "common.hpp"

#include <memory>
#include <vector>

class PostprocessGPU
{
public:
  PostprocessGPU() {}
  PostprocessGPU(
    const int _class_num, const float _score_thresh, const float _nms_thresh,
    const int _nms_pre_maxnum, const int _nms_post_maxnum, const int _down_sample,
    const int _output_h, const int _output_w, const float _x_step, const float _y_step,
    const float _x_start, const float _y_start, const std::vector<int> & _class_num_pre_task,
    const std::vector<float> & _nms_rescale_factor);

  void DoPostprocess(void ** const bev_buffer, std::vector<Box> & out_detections);
  ~PostprocessGPU();

private:
  int class_num;
  float score_thresh;
  float nms_thresh;
  int nms_pre_maxnum;
  int nms_post_maxnum;
  int down_sample;
  int output_h;
  int output_w;
  float x_step;
  float y_step;
  float x_start;
  float y_start;
  int map_size;
  int task_num;

  std::vector<int> class_num_pre_task;
  std::vector<float> nms_rescale_factor;

  std::unique_ptr<Iou3dNmsCuda> iou3d_nms;

  float * boxes_dev = nullptr;
  float * score_dev = nullptr;
  int * cls_dev = nullptr;
  int * valid_box_num = nullptr;
  int * sorted_indices_dev = nullptr;
  std::int64_t * keep_data_host = nullptr;
  int * sorted_indices_host = nullptr;
  float * boxes_host = nullptr;
  float * score_host = nullptr;
  int * cls_host = nullptr;

  float * nms_rescale_factor_dev = nullptr;
};

#endif  // POSTPROCESS_HPP_
