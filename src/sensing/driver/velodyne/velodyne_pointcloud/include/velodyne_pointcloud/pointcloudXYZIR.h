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

#ifndef __POINTCLOUDXYZIR_H
#define __POINTCLOUDXYZIR_H

#include <pcl_ros/point_cloud.h>
#include <velodyne_pointcloud/datacontainerbase.h>

#include <velodyne_pointcloud/point_types.h>

namespace velodyne_pointcloud
{
class PointcloudXYZIR : public velodyne_rawdata::DataContainerBase
{
public:
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr pc;

  PointcloudXYZIR() : pc(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>) {}

  virtual void addPoint(
    const float & x, const float & y, const float & z, const uint16_t & ring,
    const uint16_t & azimuth, const float & distance, const float & intensity,
    const double & time_stamp) override;
};
}  // namespace velodyne_pointcloud
#endif  //__POINTCLOUDXYZIR_H
