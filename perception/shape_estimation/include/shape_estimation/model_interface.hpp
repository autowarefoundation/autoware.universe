/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 *
 * v1.0 Yukihiro Saito
 */

#pragma once

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "geometry_msgs/msg/pose.hpp"
#include <string>
#include "autoware_perception_msgs/msg/shape.hpp"

class ShapeEstimationModelInterface
{
public:
  ShapeEstimationModelInterface() {}

  virtual ~ShapeEstimationModelInterface() {}

  virtual bool estimate(
    const pcl::PointCloud<pcl::PointXYZ> & cluster,
    autoware_perception_msgs::msg::Shape & shape_output,
    geometry_msgs::msg::Pose & pose_output) = 0;
};
