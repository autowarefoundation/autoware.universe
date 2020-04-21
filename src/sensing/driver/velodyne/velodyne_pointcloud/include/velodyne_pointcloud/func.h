/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include <deque>
#include <vector>

#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <velodyne_pointcloud/point_types.h>

namespace velodyne_pointcloud
{
pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractValidPoints(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const double min_range, const double max_range);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractInvalidPoints(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractInvalidNearPoints(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::vector<float> & invalid_intensity_array, const size_t num_lasers);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractInvalidNearPointsFiltered(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::vector<float> & invalid_intensity_array, const size_t num_lasers,
  const size_t points_size_threshold);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::deque<geometry_msgs::TwistStamped> & twist_queue,
  const tf2::Transform & tf2_base_link_to_sensor);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr sortRingNumber(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const size_t num_lasers);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr sortZeroIndex(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const size_t num_lasers);

pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr convert(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud);

}  // namespace velodyne_pointcloud
