/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
#include "autoware_perception_msgs/DynamicObjectArray.h"

class NaivePathPredictionNode
{
private:  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  void callback(const autoware_perception_msgs::DynamicObjectArray::ConstPtr & input_msg);

public:
  NaivePathPredictionNode();

  ~NaivePathPredictionNode(){};
};
