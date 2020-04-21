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

#include "pointcloud_preprocessor/outlier_filter/voxel_grid_outlier_filter_nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

namespace pointcloud_preprocessor
{
bool VoxelGridOutlierFilterNodelet::child_init(ros::NodeHandle & nh, bool & has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared<
    dynamic_reconfigure::Server<pointcloud_preprocessor::VoxelGridOutlierFilterConfig> >(nh);
  dynamic_reconfigure::Server<pointcloud_preprocessor::VoxelGridOutlierFilterConfig>::CallbackType
    f = boost::bind(&VoxelGridOutlierFilterNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
  return (true);
}

void VoxelGridOutlierFilterNodelet::filter(
  const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_voxelized_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_voxelized_input->points.reserve(pcl_input->points.size());
  voxel_filter.setInputCloud(pcl_input);
  voxel_filter.setSaveLeafLayout(true);
  voxel_filter.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
  voxel_filter.setMinimumPointsNumberPerVoxel(voxel_points_threshold_);
  voxel_filter.filter(*pcl_voxelized_input);

  pcl_output->points.reserve(pcl_input->points.size());
  for (size_t i = 0; i < pcl_input->points.size(); ++i) {
    const int index = voxel_filter.getCentroidIndexAt(voxel_filter.getGridCoordinates(
      pcl_input->points.at(i).x, pcl_input->points.at(i).y, pcl_input->points.at(i).z));
    if (index != -1)  // not empty voxel
    {
      pcl_output->points.push_back(pcl_input->points.at(i));
    }
  }

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

void VoxelGridOutlierFilterNodelet::subscribe() { Filter::subscribe(); }

void VoxelGridOutlierFilterNodelet::unsubscribe() { Filter::unsubscribe(); }

void VoxelGridOutlierFilterNodelet::config_callback(
  pointcloud_preprocessor::VoxelGridOutlierFilterConfig & config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (voxel_size_x_ != config.voxel_size_x) {
    voxel_size_x_ = config.voxel_size_x;
    NODELET_DEBUG(
      "[%s::config_callback] Setting new distance threshold to: %f.", getName().c_str(),
      config.voxel_size_x);
  }
  if (voxel_size_y_ != config.voxel_size_y) {
    voxel_size_y_ = config.voxel_size_y;
    NODELET_DEBUG(
      "[%s::config_callback] Setting new distance threshold to: %f.", getName().c_str(),
      config.voxel_size_y);
  }
  if (voxel_size_z_ != config.voxel_size_z) {
    voxel_size_z_ = config.voxel_size_z;
    NODELET_DEBUG(
      "[%s::config_callback] Setting new distance threshold to: %f.", getName().c_str(),
      config.voxel_size_z);
  }
  if (voxel_points_threshold_ != config.voxel_points_threshold) {
    voxel_points_threshold_ = config.voxel_points_threshold;
    NODELET_DEBUG(
      "[%s::config_callback] Setting new distance threshold to: %d.", getName().c_str(),
      config.voxel_points_threshold);
  }
  // ---[ These really shouldn't be here, and as soon as dynamic_reconfigure improves, we'll remove them and inherit
  // from Filter
  if (tf_input_frame_ != config.input_frame) {
    tf_input_frame_ = config.input_frame;
    NODELET_DEBUG("[config_callback] Setting the input TF frame to: %s.", tf_input_frame_.c_str());
  }
  if (tf_output_frame_ != config.output_frame) {
    tf_output_frame_ = config.output_frame;
    NODELET_DEBUG(
      "[config_callback] Setting the output TF frame to: %s.", tf_output_frame_.c_str());
  }
  // ]---
}

}  // namespace pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::VoxelGridOutlierFilterNodelet, nodelet::Nodelet);