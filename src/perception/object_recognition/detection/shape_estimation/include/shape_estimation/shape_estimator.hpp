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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include "autoware_perception_msgs/Shape.h"
#include "geometry_msgs/Pose.h"

class ShapeEstimator
{
private:
  bool estimateShape(
    const int type, const pcl::PointCloud<pcl::PointXYZ> & cluster,
    autoware_perception_msgs::Shape & shape_output, geometry_msgs::Pose & pose_output,
    bool & orientaion_output);
  bool applyFilter(
    const int type, const autoware_perception_msgs::Shape & shape_output,
    const geometry_msgs::Pose & pose_output, const bool & orientaion_output);
  bool applyCorrector(
    const int type, autoware_perception_msgs::Shape & shape_output,
    geometry_msgs::Pose & pose_output, bool & orientaion_output);

public:
  ShapeEstimator();

  ~ShapeEstimator(){};

  bool getShapeAndPose(
    const int type, const pcl::PointCloud<pcl::PointXYZ> & cluster,
    autoware_perception_msgs::Shape & shape_output, geometry_msgs::Pose & pose_output,
    bool & orientaion_output);
};