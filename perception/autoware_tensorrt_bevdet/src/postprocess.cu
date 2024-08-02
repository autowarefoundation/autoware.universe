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
#include "autoware/tensorrt_bevdet/postprocess.hpp"

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/sequence.h>
#include <thrust/sort.h>
#include <thrust/transform.h>

__device__ float sigmoid_gpu(const float x)
{
  return 1.0f / (1.0f + expf(-x));
}

__global__ void BEVDecodeObjectKernel(
  const int map_size,        // 40000
  const float score_thresh,  // 0.1
                             //    const int nms_pre_max_size, // 4096
  const float x_start, const float y_start, const float x_step, const float y_step,
  const int output_h, const int output_w, const int downsample_size, const int num_class_in_task,
  const int cls_range, const float * reg, const float * hei, const float * dim, const float * rot,
  const float * vel, const float * cls, float * res_box, float * res_conf, int * res_cls,
  int * res_box_num, float * rescale_factor)
{  // According to the confidence level, the initial screening showed that there were res-box_num
   // boxes after screening, not exceeding nms_pre_max_size 4096
  int idx = threadIdx.x + blockDim.x * blockIdx.x;
  if (idx >= map_size) return;

  float max_score = cls[idx];  // Initialize as the first class of the task
  int label = cls_range;       // Initialize as the first class of the task
  for (int i = 1; i < num_class_in_task; ++i) {
    float cur_score = cls[idx + i * map_size];
    if (cur_score > max_score) {
      max_score = cur_score;
      label = i + cls_range;
    }
  }

  int coor_x = idx % output_h;  //
  int coor_y = idx / output_w;  //

  float conf = sigmoid_gpu(max_score);  // Calculate confidence level
  if (conf > score_thresh) {
    int cur_valid_box_id = atomicAdd(res_box_num, 1);
    res_box[cur_valid_box_id * kBoxBlockSize + 0] =
      (reg[idx + 0 * map_size] + coor_x) * x_step + x_start;
    res_box[cur_valid_box_id * kBoxBlockSize + 1] =
      (reg[idx + 1 * map_size] + coor_y) * y_step + y_start;
    res_box[cur_valid_box_id * kBoxBlockSize + 2] = hei[idx];
    res_box[cur_valid_box_id * kBoxBlockSize + 3] =
      expf(dim[idx + 0 * map_size]) * rescale_factor[label];  // nms scale
    res_box[cur_valid_box_id * kBoxBlockSize + 4] =
      expf(dim[idx + 1 * map_size]) * rescale_factor[label];
    res_box[cur_valid_box_id * kBoxBlockSize + 5] =
      expf(dim[idx + 2 * map_size]) * rescale_factor[label];
    res_box[cur_valid_box_id * kBoxBlockSize + 6] = atan2f(rot[idx], rot[idx + map_size]);
    res_box[cur_valid_box_id * kBoxBlockSize + 7] = vel[idx];
    res_box[cur_valid_box_id * kBoxBlockSize + 8] = vel[idx + map_size];

    res_conf[cur_valid_box_id] = conf;
    res_cls[cur_valid_box_id] = label;
  }
}

PostprocessGPU::PostprocessGPU(
  const int _class_num, const float _score_thresh, const float _nms_thresh,
  const int _nms_pre_maxnum, const int _nms_post_maxnum, const int _down_sample,
  const int _output_h, const int _output_w, const float _x_step, const float _y_step,
  const float _x_start, const float _y_start, const std::vector<int> & _class_num_pre_task,
  const std::vector<float> & _nms_rescale_factor)
: class_num(_class_num),
  score_thresh(_score_thresh),
  nms_thresh(_nms_thresh),
  nms_pre_maxnum(_nms_pre_maxnum),
  nms_post_maxnum(_nms_post_maxnum),
  down_sample(_down_sample),
  output_h(_output_h),
  output_w(_output_w),
  x_step(_x_step),
  y_step(_y_step),
  x_start(_x_start),
  y_start(_y_start),
  map_size(output_h * output_w),
  class_num_pre_task(_class_num_pre_task),
  nms_rescale_factor(_nms_rescale_factor),
  task_num(_class_num_pre_task.size())
{
  CHECK_CUDA(cudaMalloc((void **)&boxes_dev, kBoxBlockSize * map_size * sizeof(float)));
  CHECK_CUDA(cudaMalloc((void **)&score_dev, map_size * sizeof(float)));
  CHECK_CUDA(cudaMalloc((void **)&cls_dev, map_size * sizeof(int)));
  CHECK_CUDA(cudaMalloc((void **)&sorted_indices_dev, map_size * sizeof(int)));
  CHECK_CUDA(cudaMalloc((void **)&valid_box_num, sizeof(int)));
  CHECK_CUDA(cudaMalloc((void **)&nms_rescale_factor_dev, class_num * sizeof(float)));

  CHECK_CUDA(cudaMallocHost((void **)&boxes_host, kBoxBlockSize * map_size * sizeof(float)));
  CHECK_CUDA(cudaMallocHost((void **)&score_host, nms_pre_maxnum * sizeof(float)));
  CHECK_CUDA(cudaMallocHost((void **)&cls_host, map_size * sizeof(float)));
  CHECK_CUDA(cudaMallocHost((void **)&sorted_indices_host, nms_pre_maxnum * sizeof(int)));
  CHECK_CUDA(cudaMallocHost((void **)&keep_data_host, nms_pre_maxnum * sizeof(std::int64_t)));

  CHECK_CUDA(cudaMemcpy(
    nms_rescale_factor_dev, nms_rescale_factor.data(), class_num * sizeof(float),
    cudaMemcpyHostToDevice));

  iou3d_nms.reset(new Iou3dNmsCuda(output_h, output_w, nms_thresh));

  for (auto i = 0; i < nms_rescale_factor.size(); i++) {
    printf("%.2f%c", nms_rescale_factor[i], i == nms_rescale_factor.size() - 1 ? '\n' : ' ');
  }
}
PostprocessGPU::~PostprocessGPU()
{
  CHECK_CUDA(cudaFree(boxes_dev));
  CHECK_CUDA(cudaFree(score_dev));
  CHECK_CUDA(cudaFree(cls_dev));
  CHECK_CUDA(cudaFree(sorted_indices_dev));
  CHECK_CUDA(cudaFree(valid_box_num));
  CHECK_CUDA(cudaFree(nms_rescale_factor_dev));

  CHECK_CUDA(cudaFreeHost(boxes_host));
  CHECK_CUDA(cudaFreeHost(score_host));
  CHECK_CUDA(cudaFreeHost(cls_host));
  CHECK_CUDA(cudaFreeHost(sorted_indices_host));
  CHECK_CUDA(cudaFreeHost(keep_data_host));
}

void PostprocessGPU::DoPostprocess(void ** const bev_buffer, std::vector<Box> & out_detections)
{
  // bev_buffer : BEV_feat, reg_0, hei_0, dim_0, rot_0, vel_0, heatmap_0, reg_1 ...
  const int task_num = class_num_pre_task.size();
  int cur_start_label = 0;
  for (int i = 0; i < task_num; i++) {
    float * reg = (float *)bev_buffer[i * 6 + 0];      // 2 x 128 x 128
    float * hei = (float *)bev_buffer[i * 6 + 1];      // 1 x 128 x 128
    float * dim = (float *)bev_buffer[i * 6 + 2];      // 3 x 128 x 128
    float * rot = (float *)bev_buffer[i * 6 + 3];      // 2 x 128 x 128
    float * vel = (float *)bev_buffer[i * 6 + 4];      // 2 x 128 x 128
    float * heatmap = (float *)bev_buffer[i * 6 + 5];  // c x 128 x 128

    dim3 grid(DIVUP(map_size, NUM_THREADS));
    CHECK_CUDA(cudaMemset(valid_box_num, 0, sizeof(int)));
    BEVDecodeObjectKernel<<<grid, NUM_THREADS>>>(
      map_size, score_thresh, x_start, y_start, x_step, y_step, output_h, output_w, down_sample,
      class_num_pre_task[i], cur_start_label, reg, hei, dim, rot, vel, heatmap, boxes_dev,
      score_dev, cls_dev, valid_box_num, nms_rescale_factor_dev);

    /*
    此时 boxes_dev, score_dev, cls_dev 有 valid_box_num 个元素，可能大于nms_pre_maxnum,
    而且是无序排列的
    */
    int box_num_pre = 0;
    CHECK_CUDA(cudaMemcpy(&box_num_pre, valid_box_num, sizeof(int), cudaMemcpyDeviceToHost));

    thrust::sequence(thrust::device, sorted_indices_dev, sorted_indices_dev + box_num_pre);
    thrust::sort_by_key(
      thrust::device, score_dev, score_dev + box_num_pre, sorted_indices_dev,
      thrust::greater<float>());
    // score_dev is sorted in descending order, while sorted_indices_dev indexes the original
    // order,， sorted_indices_dev[i] = j; i:current，j:pre-index;  j:[0, map_size]

    box_num_pre = std::min(box_num_pre, nms_pre_maxnum);

    int box_num_post =
      iou3d_nms->DoIou3dNms(box_num_pre, boxes_dev, sorted_indices_dev, keep_data_host);

    box_num_post = std::min(box_num_post, nms_post_maxnum);

    CHECK_CUDA(cudaMemcpy(
      sorted_indices_host, sorted_indices_dev, box_num_pre * sizeof(int), cudaMemcpyDeviceToHost));
    CHECK_CUDA(cudaMemcpy(
      boxes_host, boxes_dev, kBoxBlockSize * map_size * sizeof(float), cudaMemcpyDeviceToHost));
    CHECK_CUDA(
      cudaMemcpy(score_host, score_dev, box_num_pre * sizeof(float), cudaMemcpyDeviceToHost));
    CHECK_CUDA(cudaMemcpy(cls_host, cls_dev, map_size * sizeof(float), cudaMemcpyDeviceToHost));

    for (auto j = 0; j < box_num_post; j++) {
      int k = keep_data_host[j];
      int idx = sorted_indices_host[k];
      Box box;
      box.x = boxes_host[idx * kBoxBlockSize + 0];
      box.y = boxes_host[idx * kBoxBlockSize + 1];
      box.z = boxes_host[idx * kBoxBlockSize + 2];
      box.l = boxes_host[idx * kBoxBlockSize + 3] / nms_rescale_factor[cls_host[idx]];
      box.w = boxes_host[idx * kBoxBlockSize + 4] / nms_rescale_factor[cls_host[idx]];
      box.h = boxes_host[idx * kBoxBlockSize + 5] / nms_rescale_factor[cls_host[idx]];
      box.r = boxes_host[idx * kBoxBlockSize + 6];
      box.vx = boxes_host[idx * kBoxBlockSize + 7];
      box.vy = boxes_host[idx * kBoxBlockSize + 8];

      box.label = cls_host[idx];
      box.score = score_host[k];
      box.z -= box.h * 0.5;  // bottom height
      out_detections.push_back(box);
    }

    cur_start_label += class_num_pre_task[i];
  }
}
