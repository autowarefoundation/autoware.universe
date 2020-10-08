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

#include <ros/ros.h>
#include "shape_estimation/shape_estimator.hpp"
// #include "shape_estimation/map_corrector_node.hpp"
#include "autoware_perception_msgs/DynamicObjectWithFeatureArray.h"

class ShapeEstimationNode
{
private:  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  // bool use_map_correct_;

  void callback(
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_msg);

private:
  std::unique_ptr<ShapeEstimator> estimator_;
  // std::shared_ptr<MapCorrectorNode> map_corrector_node_ptr_;

public:
  ShapeEstimationNode();

  ~ShapeEstimationNode(){};
};
