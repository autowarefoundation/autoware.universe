// Copyright (c) OpenMMLab. All rights reserved.
// Modified from
// https://github.com/open-mmlab/mmdetection3d/blob/main/projects/BEVFusion/bevfusion/ops/bev_pool/src/bev_pool_cuda.cu
// https://github.com/mit-han-lab/bevfusion/blob/main/mmdet3d/ops/bev_pool/src/bev_pool_cuda.cu
// Available under Apache-2.0 license

#ifndef AUTOWARE__BEV_OPS__BEV_POOL_CUDA_HPP_
#define AUTOWARE__BEV_OPS__BEV_POOL_CUDA_HPP_

#include <cuda_runtime_api.h>

void bev_pool(
  int b, int d, int h, int w, int n, int c, int n_intervals, const float * x,
  const int * geom_feats, const int * interval_starts, const int * interval_lengths, float * out,
  cudaStream_t & stream);

#endif  // AUTOWARE__BEV_OPS__BEV_POOL_CUDA_HPP_
