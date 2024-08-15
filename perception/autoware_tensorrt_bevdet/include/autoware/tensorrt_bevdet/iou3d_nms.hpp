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
/*
3D IoU Calculation and Rotated NMS(modified from 2D NMS written by others)
Written by Shaoshuai Shi
All Rights Reserved 2019-2022.
*/
#pragma once
#ifndef AUTOWARE__TENSORRT_BEVDET__IOU3D_NMS_HPP_
#define AUTOWARE__TENSORRT_BEVDET__IOU3D_NMS_HPP_

#include "common.hpp"

#include <stdio.h>
#include <thrust/count.h>
#include <thrust/device_vector.h>
#include <thrust/gather.h>
#include <thrust/host_vector.h>
#include <thrust/sequence.h>
#include <thrust/sort.h>
#include <thrust/transform.h>

#include <iomanip>
#include <iostream>
#include <vector>

const int THREADS_PER_BLOCK = 16;
const int THREADS_PER_BLOCK_NMS = sizeof(std::int64_t) * 8;
const float EPS = 1e-8;

class Iou3dNmsCuda
{
public:
  Iou3dNmsCuda(const int head_x_size, const int head_y_size, const float nms_overlap_thresh);
  ~Iou3dNmsCuda() = default;

  int DoIou3dNms(
    const int box_num_pre, const float * res_box, const int * res_sorted_indices,
    std::int64_t * host_keep_data);

private:
  const int head_x_size_;
  const int head_y_size_;
  const float nms_overlap_thresh_;
};

static const int kBoxBlockSize = 9;

struct Box
{
  float x;
  float y;
  float z;
  float l;
  float w;
  float h;
  float r;
  float vx = 0.0f;  // optional
  float vy = 0.0f;  // optional
  float score;
  int label;
  bool is_drop;  // for nms
};
#endif  // AUTOWARE__TENSORRT_BEVDET__IOU3D_NMS_HPP_
