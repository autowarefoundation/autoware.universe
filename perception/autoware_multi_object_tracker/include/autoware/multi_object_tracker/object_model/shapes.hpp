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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__SHAPES_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__SHAPES_HPP_

#include "autoware/multi_object_tracker/object_model/dynamic_object.hpp"

#include <tf2_ros/buffer.h>

#include <string>

namespace autoware::multi_object_tracker
{
namespace shapes
{
bool transformObjects(
  const types::DynamicObjectList & input_msg, const std::string & target_frame_id,
  const tf2_ros::Buffer & tf_buffer, types::DynamicObjectList & output_msg);

double get2dIoU(
  const types::DynamicObject & source_object, const types::DynamicObject & target_object,
  const double min_union_area = 0.01);
}  // namespace shapes
}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__SHAPES_HPP_
