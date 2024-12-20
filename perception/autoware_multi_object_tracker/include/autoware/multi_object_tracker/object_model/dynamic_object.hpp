// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//
// Author: v1.0 Taekjin Lee
//

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__DYNAMIC_OBJECT_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__DYNAMIC_OBJECT_HPP_

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>

#include <vector>

namespace autoware::multi_object_tracker
{
namespace types
{
enum OrientationAvailability : uint8_t {
  UNAVAILABLE = 0,
  SIGN_UNKNOWN = 1,
  AVAILABLE = 2,
};

enum ShapeType : uint8_t {
  BOUNDING_BOX = 0,
  CYLINDER = 1,
  POLYGON = 2,
};

struct ObjectKinematics
{
  geometry_msgs::msg::PoseWithCovariance pose_with_covariance;
  geometry_msgs::msg::TwistWithCovariance twist_with_covariance;
  bool has_position_covariance = false;
  OrientationAvailability orientation_availability;
  bool has_twist = false;
  bool has_twist_covariance = false;
};

struct ObjectShape
{
  ShapeType type;
  geometry_msgs::msg::Polygon footprint;
  geometry_msgs::msg::Vector3 dimensions;
};

struct DynamicObject
{
  float_t existence_probability;
  std::vector<autoware_perception_msgs::msg::ObjectClassification> classification;
  ObjectKinematics kinematics;
  ObjectShape shape;
};

struct DynamicObjects
{
  std_msgs::msg::Header header;
  std::vector<DynamicObject> objects;
};

DynamicObject getDynamicObject(autoware_perception_msgs::msg::DetectedObject object)
{
  DynamicObject dynamic_object;
  dynamic_object.existence_probability = object.existence_probability;
  dynamic_object.classification = object.classification;

  dynamic_object.kinematics.pose_with_covariance = object.kinematics.pose_with_covariance;
  dynamic_object.kinematics.twist_with_covariance = object.kinematics.twist_with_covariance;
  dynamic_object.kinematics.has_position_covariance = object.kinematics.has_position_covariance;
  if (
    object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE) {
    dynamic_object.kinematics.orientation_availability = OrientationAvailability::UNAVAILABLE;
  } else if (
    object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN) {
    dynamic_object.kinematics.orientation_availability = OrientationAvailability::SIGN_UNKNOWN;
  } else if (
    object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE) {
    dynamic_object.kinematics.orientation_availability = OrientationAvailability::AVAILABLE;
  }
  dynamic_object.kinematics.has_twist = object.kinematics.has_twist;
  dynamic_object.kinematics.has_twist_covariance = object.kinematics.has_twist_covariance;

  if (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    dynamic_object.shape.type = ShapeType::BOUNDING_BOX;
  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    dynamic_object.shape.type = ShapeType::CYLINDER;
  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    dynamic_object.shape.type = ShapeType::POLYGON;
  }

  dynamic_object.shape.footprint = object.shape.footprint;
  dynamic_object.shape.dimensions = object.shape.dimensions;
  return dynamic_object;
}

DynamicObjects getDynamicObject(autoware_perception_msgs::msg::DetectedObjects objects)
{
  DynamicObjects dynamic_objects;
  dynamic_objects.header = objects.header;
  dynamic_objects.objects.reserve(objects.objects.size());
  for (const auto & object : objects.objects) {
    dynamic_objects.objects.emplace_back(getDynamicObject(object));
  }
  return dynamic_objects;
}

}  // namespace types
}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__DYNAMIC_OBJECT_HPP_
