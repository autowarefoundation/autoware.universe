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

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <vector>

namespace autoware::multi_object_tracker
{
namespace types
{
struct ObjectKinematics
{
  uint8_t UNAVAILABLE = 0;
  uint8_t SIGN_UNKNOWN = 1;
  uint8_t AVAILABLE = 2;

  geometry_msgs::msg::PoseWithCovariance pose_with_covariance;
  geometry_msgs::msg::TwistWithCovariance twist_with_covariance;
  bool has_position_covariance = false;
  uint8_t orientation_availability;
  bool has_twist = false;
  bool has_twist_covariance = false;
};

struct ObjectShape
{
  uint8_t BOUNDING_BOX = 0;
  uint8_t CYLINDER = 1;
  uint8_t POLYGON = 2;

  uint8_t type;
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

}  // namespace types
}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__DYNAMIC_OBJECT_HPP_
