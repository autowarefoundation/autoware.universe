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

#include "autoware/tensorrt_bevdet/iou3d_nms.hpp"

struct Point
{
  float x, y;
  __device__ Point() {}
  __device__ Point(double _x, double _y) { x = _x, y = _y; }

  __device__ void set(float _x, float _y)
  {
    x = _x;
    y = _y;
  }

  __device__ Point operator+(const Point & b) const { return Point(x + b.x, y + b.y); }

  __device__ Point operator-(const Point & b) const { return Point(x - b.x, y - b.y); }
};

__device__ inline float cross(const Point & a, const Point & b)
{
  return a.x * b.y - a.y * b.x;
}

__device__ inline float cross(const Point & p1, const Point & p2, const Point & p0)
{
  return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
}

__device__ int check_rect_cross(
  const Point & p1, const Point & p2, const Point & q1, const Point & q2)
{
  int ret = min(p1.x, p2.x) <= max(q1.x, q2.x) && min(q1.x, q2.x) <= max(p1.x, p2.x) &&
            min(p1.y, p2.y) <= max(q1.y, q2.y) && min(q1.y, q2.y) <= max(p1.y, p2.y);
  return ret;
}

__device__ inline int check_in_box2d(const float * box, const Point & p)
{
  // params: (7) [x, y, z, dx, dy, dz, heading]
  const float kMargin = 1e-2;

  float center_x = box[0], center_y = box[1];
  // rotate the point in the opposite direction of box
  float angle_cos = cos(-box[6]);
  float angle_sin = sin(-box[6]);
  float rot_x = (p.x - center_x) * angle_cos + (p.y - center_y) * (-angle_sin);
  float rot_y = (p.x - center_x) * angle_sin + (p.y - center_y) * angle_cos;

  return (fabs(rot_x) < box[3] / 2 + kMargin && fabs(rot_y) < box[4] / 2 + kMargin);
}

__device__ inline int intersection(
  const Point & p1, const Point & p0, const Point & q1, const Point & q0, Point & ans)
{
  // fast exclusion
  if (check_rect_cross(p0, p1, q0, q1) == 0) return 0;

  // check cross standing
  float s1 = cross(q0, p1, p0);
  float s2 = cross(p1, q1, p0);
  float s3 = cross(p0, q1, q0);
  float s4 = cross(q1, p1, q0);

  if (!(s1 * s2 > 0 && s3 * s4 > 0)) return 0;

  // calculate intersection of two lines
  float s5 = cross(q1, p1, p0);
  if (fabs(s5 - s1) > EPS) {
    ans.x = (s5 * q0.x - s1 * q1.x) / (s5 - s1);
    ans.y = (s5 * q0.y - s1 * q1.y) / (s5 - s1);

  } else {
    float a0 = p0.y - p1.y, b0 = p1.x - p0.x, c0 = p0.x * p1.y - p1.x * p0.y;
    float a1 = q0.y - q1.y, b1 = q1.x - q0.x, c1 = q0.x * q1.y - q1.x * q0.y;
    float D = a0 * b1 - a1 * b0;

    ans.x = (b0 * c1 - b1 * c0) / D;
    ans.y = (a1 * c0 - a0 * c1) / D;
  }

  return 1;
}

__device__ inline void rotate_around_center(
  const Point & center, const float angle_cos, const float angle_sin, Point & p)
{
  float new_x = (p.x - center.x) * angle_cos + (p.y - center.y) * (-angle_sin) + center.x;
  float new_y = (p.x - center.x) * angle_sin + (p.y - center.y) * angle_cos + center.y;
  p.set(new_x, new_y);
}

__device__ inline int point_cmp(const Point & a, const Point & b, const Point & center)
{
  return atan2(a.y - center.y, a.x - center.x) > atan2(b.y - center.y, b.x - center.x);
}

__device__ inline float box_overlap(const float * box_a, const float * box_b)
{
  // params box_a: [x, y, z, dx, dy, dz, heading]
  // params box_b: [x, y, z, dx, dy, dz, heading]

  float a_angle = box_a[6], b_angle = box_b[6];
  float a_dx_half = box_a[3] / 2, b_dx_half = box_b[3] / 2, a_dy_half = box_a[4] / 2,
        b_dy_half = box_b[4] / 2;
  float a_x1 = box_a[0] - a_dx_half, a_y1 = box_a[1] - a_dy_half;
  float a_x2 = box_a[0] + a_dx_half, a_y2 = box_a[1] + a_dy_half;
  float b_x1 = box_b[0] - b_dx_half, b_y1 = box_b[1] - b_dy_half;
  float b_x2 = box_b[0] + b_dx_half, b_y2 = box_b[1] + b_dy_half;

  Point center_a(box_a[0], box_a[1]);
  Point center_b(box_b[0], box_b[1]);

#ifdef DEBUG
  std::cout << "a: (" << std::fixed << std::setprecision(3) << a_x1 << ", "
          << a_y1 << ", " << a_x2 << ", " << a_y2 << ", " << a_angle
          << "), b: (" << b_x1 << ", " << b_y1 << ", " << b_x2 << ", "
          << b_y2 << ", " << b_angle << ")" << std::endl;
  std::cout << "center a: (" << center_a.x << ", " << center_a.y
          << "), b: (" << center_b.x << ", " << center_b.y << ")" << std::endl;
#endif

  Point box_a_corners[5];
  box_a_corners[0].set(a_x1, a_y1);
  box_a_corners[1].set(a_x2, a_y1);
  box_a_corners[2].set(a_x2, a_y2);
  box_a_corners[3].set(a_x1, a_y2);

  Point box_b_corners[5];
  box_b_corners[0].set(b_x1, b_y1);
  box_b_corners[1].set(b_x2, b_y1);
  box_b_corners[2].set(b_x2, b_y2);
  box_b_corners[3].set(b_x1, b_y2);

  // get oriented corners
  float a_angle_cos = cos(a_angle), a_angle_sin = sin(a_angle);
  float b_angle_cos = cos(b_angle), b_angle_sin = sin(b_angle);

  for (int k = 0; k < 4; k++) {
#ifdef DEBUG
    std::cout << "before corner " << k << ": a("
          << std::fixed << std::setprecision(3) << box_a_corners[k].x << ", "
          << box_a_corners[k].y << "), b("
          << box_b_corners[k].x << ", " << box_b_corners[k].y << ")" << std::endl;
#endif
    rotate_around_center(center_a, a_angle_cos, a_angle_sin, box_a_corners[k]);
    rotate_around_center(center_b, b_angle_cos, b_angle_sin, box_b_corners[k]);
#ifdef DEBUG
    std::cout << "corner " << k << ": a("
          << std::fixed << std::setprecision(3) << box_a_corners[k].x << ", "
          << box_a_corners[k].y << "), b("
          << box_b_corners[k].x << ", " << box_b_corners[k].y << ")" << std::endl;
#endif
  }

  box_a_corners[4] = box_a_corners[0];
  box_b_corners[4] = box_b_corners[0];

  // get intersection of lines
  Point cross_points[16];
  Point poly_center;
  int cnt = 0, flag = 0;

  poly_center.set(0, 0);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      flag = intersection(
        box_a_corners[i + 1], box_a_corners[i], box_b_corners[j + 1], box_b_corners[j],
        cross_points[cnt]);
      if (flag) {
        poly_center = poly_center + cross_points[cnt];
        cnt++;
#ifdef DEBUG
        std::cout << "Cross points (" 
          << std::fixed << std::setprecision(3) << cross_points[cnt - 1].x << ", " 
          << cross_points[cnt - 1].y << "): a("
          << box_a_corners[i].x << ", " << box_a_corners[i].y << ")->("
          << box_a_corners[i + 1].x << ", " << box_a_corners[i + 1].y << "), b("
          << box_b_corners[i].x << ", " << box_b_corners[i].y << ")->("
          << box_b_corners[i + 1].x << ", " << box_b_corners[i + 1].y << ")" 
          << std::endl;
#endif
      }
    }
  }

  // check corners
  for (int k = 0; k < 4; k++) {
    if (check_in_box2d(box_a, box_b_corners[k])) {
      poly_center = poly_center + box_b_corners[k];
      cross_points[cnt] = box_b_corners[k];
      cnt++;
#ifdef DEBUG
      std::cout << "b corners in a: corner_b("
          << std::fixed << std::setprecision(3) << cross_points[cnt - 1].x << ", "
          << cross_points[cnt - 1].y << ")" << std::endl;
#endif
    }
    if (check_in_box2d(box_b, box_a_corners[k])) {
      poly_center = poly_center + box_a_corners[k];
      cross_points[cnt] = box_a_corners[k];
      cnt++;
#ifdef DEBUG
      std::cout << "a corners in b: corner_a("
          << std::fixed << std::setprecision(3) << cross_points[cnt - 1].x << ", "
          << cross_points[cnt - 1].y << ")" << std::endl;
#endif
    }
  }

  poly_center.x /= cnt;
  poly_center.y /= cnt;

  // sort the points of polygon
  Point temp;
  for (int j = 0; j < cnt - 1; j++) {
    for (int i = 0; i < cnt - j - 1; i++) {
      if (point_cmp(cross_points[i], cross_points[i + 1], poly_center)) {
        temp = cross_points[i];
        cross_points[i] = cross_points[i + 1];
        cross_points[i + 1] = temp;
      }
    }
  }

#ifdef DEBUG
  std::cout << "cnt=" << cnt << std::endl;
  for (int i = 0; i < cnt; i++) {
    std::cout << "All cross point " << i << ": ("
          << std::fixed << std::setprecision(3) << cross_points[i].x << ", "
          << cross_points[i].y << ")" << std::endl;
  }
#endif

  // get the overlap areas
  float area = 0;
  for (int k = 0; k < cnt - 1; k++) {
    area += cross(cross_points[k] - cross_points[0], cross_points[k + 1] - cross_points[0]);
  }

  return fabs(area) / 2.0;
}

__device__ inline float iou_bev(const float * box_a, const float * box_b)
{
  // params box_a: [x, y, z, dx, dy, dz, heading]
  // params box_b: [x, y, z, dx, dy, dz, heading]
  float sa = box_a[3] * box_a[4];
  float sb = box_b[3] * box_b[4];
  float s_overlap = box_overlap(box_a, box_b);
  return s_overlap / fmaxf(sa + sb - s_overlap, EPS);
}

__device__ inline float iou_normal(float const * const a, float const * const b)
{
  // params: a: [x, y, z, dx, dy, dz, heading]
  // params: b: [x, y, z, dx, dy, dz, heading]
  float left = fmaxf(a[0] - a[3] / 2, b[0] - b[3] / 2),
        right = fminf(a[0] + a[3] / 2, b[0] + b[3] / 2);
  float top = fmaxf(a[1] - a[4] / 2, b[1] - b[4] / 2),
        bottom = fminf(a[1] + a[4] / 2, b[1] + b[4] / 2);
  float width = fmaxf(right - left, 0.f), height = fmaxf(bottom - top, 0.f);
  float interS = width * height;
  float Sa = a[3] * a[4];
  float Sb = b[3] * b[4];
  return interS / fmaxf(Sa + Sb - interS, EPS);
}

__global__ void boxes_overlap_kernel(
  const int num_a, const float * boxes_a, const int num_b, const float * boxes_b,
  float * ans_overlap)
{
  // params boxes_a: (N, 7) [x, y, z, dx, dy, dz, heading]
  // params boxes_b: (M, 7) [x, y, z, dx, dy, dz, heading]
  const int a_idx = blockIdx.y * THREADS_PER_BLOCK + threadIdx.y;
  const int b_idx = blockIdx.x * THREADS_PER_BLOCK + threadIdx.x;

  if (a_idx >= num_a || b_idx >= num_b) {
    return;
  }
  const float * cur_box_a = boxes_a + a_idx * kBoxBlockSize;
  const float * cur_box_b = boxes_b + b_idx * kBoxBlockSize;
  float s_overlap = box_overlap(cur_box_a, cur_box_b);
  ans_overlap[a_idx * num_b + b_idx] = s_overlap;
}

__global__ void boxes_iou_bev_kernel(
  const int num_a, const float * boxes_a, const int num_b, const float * boxes_b, float * ans_iou)
{
  // params boxes_a: (N, 7) [x, y, z, dx, dy, dz, heading]
  // params boxes_b: (M, 7) [x, y, z, dx, dy, dz, heading]
  const int a_idx = blockIdx.y * THREADS_PER_BLOCK + threadIdx.y;
  const int b_idx = blockIdx.x * THREADS_PER_BLOCK + threadIdx.x;

  if (a_idx >= num_a || b_idx >= num_b) {
    return;
  }

  const float * cur_box_a = boxes_a + a_idx * kBoxBlockSize;
  const float * cur_box_b = boxes_b + b_idx * kBoxBlockSize;
  float cur_iou_bev = iou_bev(cur_box_a, cur_box_b);
  ans_iou[a_idx * num_b + b_idx] = cur_iou_bev;
}

__global__ void RotatedNmsKernel(
  const int boxes_num, const float nms_overlap_thresh, const float * boxes, std::int64_t * mask)
{
  // params: boxes (N, 7) [x, y, z, dx, dy, dz, heading]
  // params: mask (N, N/THREADS_PER_BLOCK_NMS)

  const int row_start = blockIdx.y;
  const int col_start = blockIdx.x;

  // if (row_start > col_start) return;

  const int row_size = fminf(boxes_num - row_start * THREADS_PER_BLOCK_NMS, THREADS_PER_BLOCK_NMS);
  const int col_size = fminf(boxes_num - col_start * THREADS_PER_BLOCK_NMS, THREADS_PER_BLOCK_NMS);

  __shared__ float block_boxes[THREADS_PER_BLOCK_NMS * kBoxBlockSize];

  if (threadIdx.x < col_size) {
    block_boxes[threadIdx.x * kBoxBlockSize + 0] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 0];
    block_boxes[threadIdx.x * kBoxBlockSize + 1] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 1];
    block_boxes[threadIdx.x * kBoxBlockSize + 2] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 2];
    block_boxes[threadIdx.x * kBoxBlockSize + 3] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 3];
    block_boxes[threadIdx.x * kBoxBlockSize + 4] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 4];
    block_boxes[threadIdx.x * kBoxBlockSize + 5] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 5];
    block_boxes[threadIdx.x * kBoxBlockSize + 6] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 6];
  }
  __syncthreads();

  if (threadIdx.x < row_size) {
    const int cur_box_idx = THREADS_PER_BLOCK_NMS * row_start + threadIdx.x;
    const float * cur_box = boxes + cur_box_idx * kBoxBlockSize;

    int i = 0;
    std::int64_t t = 0;
    int start = 0;
    if (row_start == col_start) {
      start = threadIdx.x + 1;
    }
    for (i = start; i < col_size; i++) {
      if (iou_bev(cur_box, block_boxes + i * kBoxBlockSize) > nms_overlap_thresh) {
        t |= 1ULL << i;
      }
    }
    const int col_blocks = DIVUP(boxes_num, THREADS_PER_BLOCK_NMS);
    mask[cur_box_idx * col_blocks + col_start] = t;
  }
}

__global__ void NmsKernel(
  const int boxes_num, const float nms_overlap_thresh, const float * boxes, std::int64_t * mask)
{
  // params: boxes (N, 7) [x, y, z, dx, dy, dz, heading]
  // params: mask (N, N/THREADS_PER_BLOCK_NMS)

  const int row_start = blockIdx.y;
  const int col_start = blockIdx.x;

  // if (row_start > col_start) return;

  const int row_size = fminf(boxes_num - row_start * THREADS_PER_BLOCK_NMS, THREADS_PER_BLOCK_NMS);
  const int col_size = fminf(boxes_num - col_start * THREADS_PER_BLOCK_NMS, THREADS_PER_BLOCK_NMS);

  __shared__ float block_boxes[THREADS_PER_BLOCK_NMS * kBoxBlockSize];

  if (threadIdx.x < col_size) {
    block_boxes[threadIdx.x * kBoxBlockSize + 0] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 0];
    block_boxes[threadIdx.x * kBoxBlockSize + 1] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 1];
    block_boxes[threadIdx.x * kBoxBlockSize + 2] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 2];
    block_boxes[threadIdx.x * kBoxBlockSize + 3] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 3];
    block_boxes[threadIdx.x * kBoxBlockSize + 4] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 4];
    block_boxes[threadIdx.x * kBoxBlockSize + 5] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 5];
    block_boxes[threadIdx.x * kBoxBlockSize + 6] =
      boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * kBoxBlockSize + 6];
  }
  __syncthreads();

  if (threadIdx.x < row_size) {
    const int cur_box_idx = THREADS_PER_BLOCK_NMS * row_start + threadIdx.x;
    const float * cur_box = boxes + cur_box_idx * kBoxBlockSize;

    int i = 0;
    std::int64_t t = 0;
    int start = 0;
    if (row_start == col_start) {
      start = threadIdx.x + 1;
    }
    for (i = start; i < col_size; i++) {
      if (iou_normal(cur_box, block_boxes + i * kBoxBlockSize) > nms_overlap_thresh) {
        t |= 1ULL << i;
      }
    }
    const int col_blocks = DIVUP(boxes_num, THREADS_PER_BLOCK_NMS);
    mask[cur_box_idx * col_blocks + col_start] = t;
  }
}

__global__ void RotatedNmsWithIndicesKernel(
  const int boxes_num, const float nms_overlap_thresh, const float * res_box,
  const int * res_sorted_indices, std::int64_t * mask)
{
  const int row_start = blockIdx.y;
  const int col_start = blockIdx.x;

  const int row_size = fminf(boxes_num - row_start * THREADS_PER_BLOCK_NMS, THREADS_PER_BLOCK_NMS);
  const int col_size = fminf(boxes_num - col_start * THREADS_PER_BLOCK_NMS, THREADS_PER_BLOCK_NMS);

  __shared__ float block_boxes[THREADS_PER_BLOCK_NMS * kBoxBlockSize];

  if (threadIdx.x < col_size) {  // 419, 420 have two situations to discuss separately
    const int col_actual_idx = res_sorted_indices[THREADS_PER_BLOCK_NMS * col_start + threadIdx.x];
    block_boxes[threadIdx.x * kBoxBlockSize + 0] = res_box[col_actual_idx * kBoxBlockSize + 0];
    block_boxes[threadIdx.x * kBoxBlockSize + 1] = res_box[col_actual_idx * kBoxBlockSize + 1];
    block_boxes[threadIdx.x * kBoxBlockSize + 2] = res_box[col_actual_idx * kBoxBlockSize + 2];
    block_boxes[threadIdx.x * kBoxBlockSize + 3] = res_box[col_actual_idx * kBoxBlockSize + 3];
    block_boxes[threadIdx.x * kBoxBlockSize + 4] = res_box[col_actual_idx * kBoxBlockSize + 4];
    block_boxes[threadIdx.x * kBoxBlockSize + 5] = res_box[col_actual_idx * kBoxBlockSize + 5];
    block_boxes[threadIdx.x * kBoxBlockSize + 6] = res_box[col_actual_idx * kBoxBlockSize + 6];
  }
  __syncthreads();  // Waiting for threads within the block

  if (threadIdx.x < row_size) {  //
    const int cur_box_idx = THREADS_PER_BLOCK_NMS * row_start + threadIdx.x;
    const int row_actual_idx = res_sorted_indices[cur_box_idx];

    float cur_box[kBoxBlockSize];
    cur_box[0] = res_box[row_actual_idx * kBoxBlockSize + 0];
    cur_box[1] = res_box[row_actual_idx * kBoxBlockSize + 1];
    cur_box[2] = res_box[row_actual_idx * kBoxBlockSize + 2];
    cur_box[3] = res_box[row_actual_idx * kBoxBlockSize + 3];
    cur_box[4] = res_box[row_actual_idx * kBoxBlockSize + 4];
    cur_box[5] = res_box[row_actual_idx * kBoxBlockSize + 5];
    cur_box[6] = res_box[row_actual_idx * kBoxBlockSize + 6];

    int i = 0;
    std::int64_t t = 0;
    int start = 0;
    if (row_start == col_start) {
      start = threadIdx.x + 1;  // Isn't it right now to compare with [0~threadIdx. x)? FIXME
    }  // I think this is correct, and there is no need to compare it with numbers smaller than my
       // own. If col is smaller than row, there is also no need to compare it
    for (i = start; i < col_size; i++) {
      if (iou_bev(cur_box, block_boxes + i * kBoxBlockSize) > nms_overlap_thresh) {
        t |= 1ULL << i;
      }
    }

    const int col_blocks = DIVUP(boxes_num, THREADS_PER_BLOCK_NMS);
    // assume cur_box_idx = 21, col_start = 0, row_start = 0 , threadIdx = 21,
    // mark 21 th box and top 64 boxes
    mask[cur_box_idx * col_blocks + col_start] = t;
    // or col_start * boxes_num + cur_box_idx
  }
}

__global__ void box_assign_kernel(
  float * reg, float * height, float * dim, float * rot, float * boxes, float * score, int * label,
  float * out_score, int * out_label, int * validIndexs, int head_x_size, int head_y_size)
{
  int boxId = blockIdx.x;
  int channel = threadIdx.x;
  int idx = validIndexs[boxId];
  if (channel == 0) {
    boxes[boxId * kBoxBlockSize + 0] = reg[idx];
  } else if (channel == 1) {
    boxes[boxId * kBoxBlockSize + 1] = reg[idx + head_x_size * head_y_size];
  } else if (channel == 2) {
    boxes[boxId * kBoxBlockSize + 2] = height[idx];
  } else if (channel == 3) {
    boxes[boxId * kBoxBlockSize + 3] = dim[idx];
  } else if (channel == 4) {
    boxes[boxId * kBoxBlockSize + 4] = dim[idx + head_x_size * head_y_size];
  } else if (channel == 5) {
    boxes[boxId * kBoxBlockSize + 5] = dim[idx + 2 * head_x_size * head_y_size];
  } else if (channel == 6) {
    float theta =
      atan2f(rot[0 * head_x_size * head_y_size + idx], rot[1 * head_x_size * head_y_size + idx]);
    theta = -theta - 3.1415926 / 2;
    boxes[boxId * kBoxBlockSize + 6] = theta;
  }
  // else if(channel == 7)
  // out_score[boxId] = score[idx];
  else if (channel == 8) {
    out_label[boxId] = label[idx];
  }
}

void box_assign_launcher(
  float * reg, float * height, float * dim, float * rot, float * boxes, float * score, int * label,
  float * out_score, int * out_label, int * validIndexs, int boxSize, int head_x_size,
  int head_y_size)
{
  box_assign_kernel<<<boxSize, 9>>>(
    reg, height, dim, rot, boxes, score, label, out_score, out_label, validIndexs, head_x_size,
    head_y_size);
}

__global__ void index_assign(int * indexs)
{
  int yIdx = blockIdx.x;
  int xIdx = threadIdx.x;
  int idx = yIdx * blockDim.x + xIdx;
  indexs[idx] = idx;
}

void index_assign_launcher(int * indexs, int head_x_size, int head_y_size)
{
  index_assign<<<head_x_size, head_y_size>>>(indexs);
}

void boxes_overlap_launcher(
  const int num_a, const float * boxes_a, const int num_b, const float * boxes_b,
  float * ans_overlap)
{
  dim3 blocks(
    DIVUP(num_b, THREADS_PER_BLOCK),
    DIVUP(num_a, THREADS_PER_BLOCK));  // blockIdx.x(col), blockIdx.y(row)
  dim3 threads(THREADS_PER_BLOCK, THREADS_PER_BLOCK);

  boxes_overlap_kernel<<<blocks, threads>>>(num_a, boxes_a, num_b, boxes_b, ans_overlap);
#ifdef DEBUG
  cudaDeviceSynchronize();  // for using printf in kernel function
#endif
}

void boxes_iou_bev_launcher(
  const int num_a, const float * boxes_a, const int num_b, const float * boxes_b, float * ans_iou)
{
  dim3 blocks(
    DIVUP(num_b, THREADS_PER_BLOCK),
    DIVUP(num_a, THREADS_PER_BLOCK));  // blockIdx.x(col), blockIdx.y(row)
  dim3 threads(THREADS_PER_BLOCK, THREADS_PER_BLOCK);

  boxes_iou_bev_kernel<<<blocks, threads>>>(num_a, boxes_a, num_b, boxes_b, ans_iou);
#ifdef DEBUG
  cudaDeviceSynchronize();  // for using printf in kernel function
#endif
}

void nms_launcher(const float * boxes, std::int64_t * mask, int boxes_num, float nms_overlap_thresh)
{
  dim3 blocks(DIVUP(boxes_num, THREADS_PER_BLOCK_NMS), DIVUP(boxes_num, THREADS_PER_BLOCK_NMS));
  dim3 threads(THREADS_PER_BLOCK_NMS);

  RotatedNmsKernel<<<blocks, threads>>>(boxes_num, nms_overlap_thresh, boxes, mask);
}

void nms_normal_launcher(
  const float * boxes, std::int64_t * mask, int boxes_num, float nms_overlap_thresh)
{
  dim3 blocks(DIVUP(boxes_num, THREADS_PER_BLOCK_NMS), DIVUP(boxes_num, THREADS_PER_BLOCK_NMS));
  dim3 threads(THREADS_PER_BLOCK_NMS);
  NmsKernel<<<blocks, threads>>>(boxes_num, nms_overlap_thresh, boxes, mask);
}

Iou3dNmsCuda::Iou3dNmsCuda(
  const int head_x_size, const int head_y_size, const float nms_overlap_thresh)
: head_x_size_(head_x_size), head_y_size_(head_y_size), nms_overlap_thresh_(nms_overlap_thresh)
{
}

int Iou3dNmsCuda::DoIou3dNms(
  const int box_num_pre, const float * res_box, const int * res_sorted_indices,
  std::int64_t * host_keep_data)
{
  const int col_blocks = DIVUP(box_num_pre, THREADS_PER_BLOCK_NMS);  // How many blocks are divided
  std::int64_t * dev_mask = NULL;
  cudaMalloc((void **)&dev_mask, box_num_pre * col_blocks * sizeof(std::int64_t));  //
  // THREADS_PER_BLOCK_NMS 掩码的长度
  dim3 blocks(DIVUP(box_num_pre, THREADS_PER_BLOCK_NMS), DIVUP(box_num_pre, THREADS_PER_BLOCK_NMS));
  dim3 threads(THREADS_PER_BLOCK_NMS);
  RotatedNmsWithIndicesKernel<<<blocks, threads>>>(
    box_num_pre, nms_overlap_thresh_, res_box, res_sorted_indices, dev_mask);

  std::int64_t host_mask[box_num_pre * col_blocks];
  cudaMemcpy(
    host_mask, dev_mask, box_num_pre * col_blocks * sizeof(std::int64_t), cudaMemcpyDeviceToHost);
  cudaFree(dev_mask);

  std::int64_t host_remv[col_blocks];
  memset(host_remv, 0, col_blocks * sizeof(std::int64_t));
  int num_to_keep = 0;
  for (int i = 0; i < box_num_pre; i++) {
    int nblock = i / THREADS_PER_BLOCK_NMS;
    int inblock = i % THREADS_PER_BLOCK_NMS;
    if (!(host_remv[nblock] &
          (1ULL << inblock))) {  // Meeting this requirement indicates that it does not conflict
                                 // with any of the boxes selected earlier
      host_keep_data[num_to_keep++] = i;  // Add the satisfied box
      for (int j = nblock; j < col_blocks;
           j++) {  // Consider boxes with index i that overlap beyond the threshold
        host_remv[j] |= host_mask[i * col_blocks + j];
      }
    }
  }

  if (cudaSuccess != cudaGetLastError()) {
    std::cerr << "Error!" << std::endl;
  }
  return num_to_keep;
}
