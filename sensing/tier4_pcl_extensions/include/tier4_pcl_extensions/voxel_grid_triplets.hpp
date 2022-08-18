// Copyright 2020 Tier IV, Inc.
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
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef TIER4_PCL_EXTENSIONS__VOXEL_GRID_TRIPLETS_HPP_
#define TIER4_PCL_EXTENSIONS__VOXEL_GRID_TRIPLETS_HPP_

#include <pcl/pcl_config.h>

#if PCL_VERSION < PCL_VERSION_CALC(1, 12, 0)
#include <pcl/filters/boost.h>
#endif

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <map>
#include <vector>

namespace pcl
{

struct cloud_point_index_idx_triplets
{
  unsigned int idx0;
  unsigned int idx1;
  unsigned int idx2;
  unsigned int cloud_point_index;

  cloud_point_index_idx_triplets() = default;
  cloud_point_index_idx_triplets(
    unsigned int idx0_, unsigned int idx1_, unsigned int idx2_, unsigned int cloud_point_index_)
  : idx0(idx0_), idx1(idx1_), idx2(idx2_), cloud_point_index(cloud_point_index_)
  {
  }
  bool operator<(const cloud_point_index_idx_triplets & p) const
  {
    return (idx0 < p.idx0) || (idx0 == p.idx0 && idx1 < p.idx1) ||
           (idx0 == p.idx0 && idx1 == p.idx1 && idx2 < p.idx2);
  }  // test brancheless agains branched version with a dataset at some point \ along with some sort
     // implementations
};

template <typename PointT>
class VoxelGridTriplets : public VoxelGrid<PointT>
{
public:
  using Filter<PointT>::filter_name_;
  using Filter<PointT>::getClassName;
  using Filter<PointT>::input_;
  using Filter<PointT>::indices_;
  using VoxelGrid<PointT>::filter_field_name_;
  using VoxelGrid<PointT>::filter_limit_min_;
  using VoxelGrid<PointT>::filter_limit_max_;
  using VoxelGrid<PointT>::filter_limit_negative_;
  using VoxelGrid<PointT>::inverse_leaf_size_;
  using VoxelGrid<PointT>::min_b_;
  using VoxelGrid<PointT>::max_b_;
  using VoxelGrid<PointT>::div_b_;
  using VoxelGrid<PointT>::divb_mul_;
  using VoxelGrid<PointT>::min_points_per_voxel_;
  using VoxelGrid<PointT>::save_leaf_layout_;
  using VoxelGrid<PointT>::leaf_layout_;
  using VoxelGrid<PointT>::downsample_all_data_;

  using PointCloud = typename Filter<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

  typedef pcl::shared_ptr<VoxelGrid<PointT>> Ptr;
  typedef pcl::shared_ptr<const VoxelGrid<PointT>> ConstPtr;

protected:
  /** \brief Filter cloud and initializes voxel structure.
   * \param[out] output cloud containing centroids of voxels containing a sufficient
   *                    number of points
   */
  void applyFilter(PointCloud & output);
};
}  // namespace pcl

#ifdef PCL_NO_PRECOMPILE
//  #include <pcl/filters/impl/voxel_grid_covariance.hpp>
#include <voxel_grid_approxi.hpp>
#endif

#endif  // TIER4_PCL_EXTENSIONS__VOXEL_GRID_TRIPLETS_HPP_
