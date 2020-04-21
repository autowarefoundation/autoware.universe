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
#include <ros/ros.h>
#include "autoware_perception_msgs/DynamicObjectArray.h"
#include "autoware_perception_msgs/DynamicObjectWithFeatureArray.h"
#include "autoware_perception_msgs/PredictedPath.h"
#include "autoware_perception_msgs/Shape.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/ColorRGBA.h"

class DynamicObjectVisualizer
{
private:  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  void dynamicObjectWithFeatureCallback(
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_msg);
  void dynamicObjectCallback(
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr & input_msg);
  bool calcBoundingBoxLineList(
    const autoware_perception_msgs::Shape & shape, std::vector<geometry_msgs::Point> & points);
  bool calcCylinderLineList(
    const autoware_perception_msgs::Shape & shape, std::vector<geometry_msgs::Point> & points);
  bool calcCircleLineList(
    const geometry_msgs::Point center, const double radius,
    std::vector<geometry_msgs::Point> & points, const int n = 20);
  bool calcPolygonLineList(
    const autoware_perception_msgs::Shape & shape, std::vector<geometry_msgs::Point> & points);
  bool calcPathLineList(
    const autoware_perception_msgs::PredictedPath & path,
    std::vector<geometry_msgs::Point> & points);
  bool getLabel(const autoware_perception_msgs::Semantic & semantic, std::string & label);
  void getColor(
    const autoware_perception_msgs::DynamicObject & object, std_msgs::ColorRGBA & color);
  void initColorList(std::vector<std_msgs::ColorRGBA> & colors);
  void initPose(geometry_msgs::Pose & pose);

  bool only_known_objects_;
  std::vector<std_msgs::ColorRGBA> colors_;

public:
  DynamicObjectVisualizer();
  virtual ~DynamicObjectVisualizer() {}
};