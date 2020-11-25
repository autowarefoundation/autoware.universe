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
#include "pointcloud_preprocessor/filter.h"

namespace pointcloud_preprocessor
{
class RingOutlierFilterComponent : public pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

private:
  double distance_ratio_;
  double object_length_threshold_;
  int num_points_threshold_;

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RingOutlierFilterComponent(const rclcpp::NodeOptions & options);
};

}  // namespace pointcloud_preprocessor

namespace pcl
{
struct PointXYZIRADT
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  float azimuth;
  float distance;
  double time_stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}  // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(
  pcl::PointXYZIRADT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(
    float, azimuth, azimuth)(float, distance, distance)(double, time_stamp, time_stamp))
