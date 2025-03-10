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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__SHAPES_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__SHAPES_HPP_

#include "autoware/multi_object_tracker/object_model/types.hpp"

#include <Eigen/Core>

#include <tf2_ros/buffer.h>

#include <string>

namespace autoware::multi_object_tracker
{
namespace shapes
{
double get2dIoU(
  const types::DynamicObject & source_object, const types::DynamicObject & target_object,
  const double min_union_area = 0.01);

bool convertConvexHullToBoundingBox(
  const types::DynamicObject & input_object, types::DynamicObject & output_object);

bool getMeasurementYaw(
  const types::DynamicObject & object, const double & predicted_yaw, double & measurement_yaw);

Eigen::Vector2d getNearestCornerOrSurface(
  const types::DynamicObject & object, const geometry_msgs::msg::Transform & self_transform);

void calcAnchorPointOffset(
  const types::DynamicObject & this_object, const types::DynamicObject & input_object,
  const Eigen::Vector2d anchor_vector, Eigen::Vector2d & tracking_offset,
  types::DynamicObject & offset_object);
}  // namespace shapes
}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__SHAPES_HPP_
