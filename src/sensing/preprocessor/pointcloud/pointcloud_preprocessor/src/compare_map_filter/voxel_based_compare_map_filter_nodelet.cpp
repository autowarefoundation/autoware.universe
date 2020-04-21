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

#include "pointcloud_preprocessor/compare_map_filter/voxel_based_compare_map_filter_nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

namespace pointcloud_preprocessor
{
bool VoxelBasedCompareMapFilterNodelet::child_init(ros::NodeHandle & nh, bool & has_service)
{
  set_map_in_voxel_grid_ = false;
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared<
    dynamic_reconfigure::Server<pointcloud_preprocessor::CompareMapFilterConfig> >(nh);
  dynamic_reconfigure::Server<pointcloud_preprocessor::CompareMapFilterConfig>::CallbackType f =
    boost::bind(&VoxelBasedCompareMapFilterNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
  return (true);
}

void VoxelBasedCompareMapFilterNodelet::filter(
  const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (voxel_map_ptr_ == NULL) {
    output = *input;
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_output->points.reserve(pcl_input->points.size());
  for (size_t i = 0; i < pcl_input->points.size(); ++i) {
    const pcl::PointXYZ point = pcl_input->points.at(i);
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y, point.z), point, distance_threshold_, voxel_map_ptr_,
          voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y - distance_threshold_, point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y - distance_threshold_, point.z), point,
          distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y - distance_threshold_, point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y, point.z - distance_threshold_), point,
          distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y, point.z + distance_threshold_), point,
          distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y + distance_threshold_, point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y + distance_threshold_, point.z), point,
          distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y + distance_threshold_, point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;

    if (is_in_voxel(
          pcl::PointXYZ(
            point.x - distance_threshold_, point.y - distance_threshold_,
            point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x - distance_threshold_, point.y - distance_threshold_, point.z),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(
            point.x - distance_threshold_, point.y - distance_threshold_,
            point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x - distance_threshold_, point.y, point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x - distance_threshold_, point.y, point.z), point,
          distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x - distance_threshold_, point.y, point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(
            point.x - distance_threshold_, point.y + distance_threshold_,
            point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x - distance_threshold_, point.y + distance_threshold_, point.z),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(
            point.x - distance_threshold_, point.y + distance_threshold_,
            point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;

    if (is_in_voxel(
          pcl::PointXYZ(
            point.x + distance_threshold_, point.y - distance_threshold_,
            point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x + distance_threshold_, point.y - distance_threshold_, point.z),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(
            point.x + distance_threshold_, point.y - distance_threshold_,
            point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x + distance_threshold_, point.y, point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x + distance_threshold_, point.y, point.z), point,
          distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x + distance_threshold_, point.y, point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(
            point.x + distance_threshold_, point.y + distance_threshold_,
            point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(point.x + distance_threshold_, point.y + distance_threshold_, point.z),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(
          pcl::PointXYZ(
            point.x + distance_threshold_, point.y + distance_threshold_,
            point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;

    pcl_output->points.push_back(pcl_input->points.at(i));
  }
  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

bool VoxelBasedCompareMapFilterNodelet::is_in_voxel(
  const pcl::PointXYZ & src_point, const pcl::PointXYZ & target_point,
  const double distance_threshold, const PointCloudPtr & map,
  pcl::VoxelGrid<pcl::PointXYZ> & voxel) const
{
  int voxel_index =
    voxel.getCentroidIndexAt(voxel.getGridCoordinates(src_point.x, src_point.y, src_point.z));
  if (voxel_index != -1)  // not empty voxel
  {
    const double dist_x = map->points.at(voxel_index).x - target_point.x;
    const double dist_y = map->points.at(voxel_index).y - target_point.y;
    const double dist_z = map->points.at(voxel_index).z - target_point.z;
    const double sqr_distance = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;
    if (sqr_distance < distance_threshold * distance_threshold) {
      return true;
    }
  }
  return false;
}

void VoxelBasedCompareMapFilterNodelet::input_target_callback(const PointCloudConstPtr & map)
{
  boost::mutex::scoped_lock lock(mutex_);
  set_map_in_voxel_grid_ = true;
  tf_input_frame_ = map->header.frame_id;
  voxel_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(distance_threshold_, distance_threshold_, distance_threshold_);
  voxel_grid_.setInputCloud(map);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr_);
}

void VoxelBasedCompareMapFilterNodelet::subscribe()
{
  Filter::subscribe();
  sub_map_ =
    pnh_->subscribe("map", 1, &VoxelBasedCompareMapFilterNodelet::input_target_callback, this);
}

void VoxelBasedCompareMapFilterNodelet::unsubscribe()
{
  Filter::unsubscribe();
  sub_map_.shutdown();
}

void VoxelBasedCompareMapFilterNodelet::config_callback(
  pointcloud_preprocessor::CompareMapFilterConfig & config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (distance_threshold_ != config.distance_threshold) {
    distance_threshold_ = config.distance_threshold;
    voxel_grid_.setLeafSize(distance_threshold_, distance_threshold_, distance_threshold_);
    voxel_grid_.setSaveLeafLayout(true);
    if (set_map_in_voxel_grid_) voxel_grid_.filter(*voxel_map_ptr_);
    NODELET_DEBUG(
      "[%s::config_callback] Setting new distance threshold to: %f.", getName().c_str(),
      config.distance_threshold);
  }
  // ---[ These really shouldn't be here, and as soon as dynamic_reconfigure improves, we'll remove them and inherit
  // from Filter
  if (tf_output_frame_ != config.output_frame) {
    tf_output_frame_ = config.output_frame;
    NODELET_DEBUG(
      "[config_callback] Setting the output TF frame to: %s.", tf_output_frame_.c_str());
  }
  // ]---
}

}  // namespace pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(
  pointcloud_preprocessor::VoxelBasedCompareMapFilterNodelet, nodelet::Nodelet);