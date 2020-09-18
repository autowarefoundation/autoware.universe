/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>
#include "pointcloud_preprocessor/VoxelGridOutlierFilterConfig.h"
#include "pointcloud_preprocessor/filter.h"

namespace pointcloud_preprocessor
{
class VoxelGridOutlierFilterNodelet : public pointcloud_preprocessor::Filter
{
protected:
  boost::shared_ptr<
    dynamic_reconfigure::Server<pointcloud_preprocessor::VoxelGridOutlierFilterConfig> >
    srv_;
  virtual void filter(
    const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);
  virtual void subscribe();
  virtual void unsubscribe();

  bool child_init(ros::NodeHandle & nh, bool & has_service);
  void config_callback(
    pointcloud_preprocessor::VoxelGridOutlierFilterConfig & config, uint32_t level);

private:
  double voxel_size_x_;
  double voxel_size_y_;
  double voxel_size_z_;
  int voxel_points_threshold_;

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pointcloud_preprocessor
