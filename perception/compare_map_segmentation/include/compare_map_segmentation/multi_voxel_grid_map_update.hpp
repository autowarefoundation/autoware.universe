// Copyright 2023 Autoware Foundation
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

#ifndef COMPARE_MAP_SEGMENTATION__MULTI_VOXEL_GRID_MAP_UPDATE_HPP_
#define COMPARE_MAP_SEGMENTATION__MULTI_VOXEL_GRID_MAP_UPDATE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <boost/sort/spreadsort/integer_sort.hpp>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <cfloat>  // for FLT_MAX
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace compare_map_segmentation
{
template <typename PointT>
// TODO(badai-nguyen): when map loader I/F is updated, remove this class since
// boundary point calculation become unnecessary
class MultiVoxelGrid : public pcl::VoxelGrid<PointT>
{
  typedef std::map<std::string, pcl::PointCloud<PointT>> MapCellDict;
  struct cloud_point_index_idx
  {
    unsigned int idx;
    unsigned int cloud_point_index;

    cloud_point_index_idx() = default;
    cloud_point_index_idx(unsigned int idx_, unsigned int cloud_point_index_)
    : idx(idx_), cloud_point_index(cloud_point_index_)
    {
    }
    bool operator<(const cloud_point_index_idx & p) const { return (idx < p.idx); }
  };

protected:
  using pcl::VoxelGrid<PointT>::filter_name_;
  using pcl::VoxelGrid<PointT>::downsample_all_data_;
  using pcl::VoxelGrid<PointT>::input_;
  using pcl::VoxelGrid<PointT>::save_leaf_layout_;
  using pcl::VoxelGrid<PointT>::min_b_;
  using pcl::VoxelGrid<PointT>::max_b_;
  using pcl::VoxelGrid<PointT>::divb_mul_;
  using pcl::VoxelGrid<PointT>::div_b_;
  using pcl::VoxelGrid<PointT>::inverse_leaf_size_;

  using pcl::VoxelGrid<PointT>::filter_limit_negative_;
  using pcl::VoxelGrid<PointT>::filter_limit_max_;
  using pcl::VoxelGrid<PointT>::filter_limit_min_;
  using pcl::VoxelGrid<PointT>::indices_;
  using pcl::VoxelGrid<PointT>::min_points_per_voxel_;
  using pcl::VoxelGrid<PointT>::filter_field_name_;

  using PointCloud = typename pcl::Filter<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  using pcl::VoxelGrid<PointT>::leaf_layout_;

  inline void set_voxel_grid(
    std::vector<int> * leaf_layout, const Eigen::Vector4i & min_b, const Eigen::Vector4i & max_b,
    const Eigen::Vector4i & div_b, const Eigen::Vector4i & divb_mul,
    const Eigen::Array4f & inverse_leaf_size)
  {
    leaf_layout_ = std::move(*leaf_layout);
    min_b_ = min_b;
    max_b_ = max_b;
    div_b_ = div_b;
    divb_mul_ = divb_mul;
    inverse_leaf_size_ = inverse_leaf_size;
  }

  inline Eigen::Vector4i get_min_b() const { return min_b_; }
  inline Eigen::Vector4i get_divb_mul() const { return divb_mul_; }
  inline Eigen::Vector4i get_max_b() const { return max_b_; }
  inline Eigen::Vector4i get_div_b() const { return div_b_; }
  inline Eigen::Array4f get_inverse_leaf_size() const { return inverse_leaf_size_; }
  inline std::vector<int> getLeafLayout() { return (leaf_layout_); }
};

};  // namespace compare_map_segmentation

#endif  // COMPARE_MAP_SEGMENTATION__MULTI_VOXEL_GRID_MAP_UPDATE_HPP_
