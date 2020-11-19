/*
 * Copyright 2020 TierIV. All rights reserved.
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
#include <rclcpp/rclcpp.hpp>
#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"

class Debugger
{
public:
  Debugger(rclcpp::Node * node);
  ~Debugger(){};
  void publishColoredPointCloud(
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & input);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr instance_pointcloud_pub_;
};