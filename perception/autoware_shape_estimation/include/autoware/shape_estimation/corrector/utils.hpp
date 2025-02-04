// Copyright 2021 TierIV. All rights reserved.
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

#ifndef AUTOWARE__SHAPE_ESTIMATION__CORRECTOR__UTILS_HPP_
#define AUTOWARE__SHAPE_ESTIMATION__CORRECTOR__UTILS_HPP_

#include "autoware/shape_estimation/shape_estimator.hpp"

#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace autoware::shape_estimation
{

namespace corrector_utils
{
struct CorrectionBBParameters
{
  double min_width;
  double max_width;
  double default_width;
  double min_length;
  double max_length;
  double default_length;
};

bool correctWithDefaultValue(
  const CorrectionBBParameters & param, autoware_perception_msgs::msg::Shape & shape_output,
  geometry_msgs::msg::Pose & pose_output);

bool correctWithReferenceYawAndShapeSize(
  const ReferenceShapeSizeInfo & ref_shape_size_info,
  autoware_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output);

bool correctWithReferenceYaw(
  const CorrectionBBParameters & param, autoware_perception_msgs::msg::Shape & shape_output,
  geometry_msgs::msg::Pose & pose_output);

bool correctWithReferenceShapeAndPose(
  const ReferenceShapeSizeInfo & ref_shape_size_info, const geometry_msgs::msg::Pose & ref_pose,
  autoware_perception_msgs::msg::Shape & shape, geometry_msgs::msg::Pose & pose);

}  // namespace corrector_utils

}  // namespace autoware::shape_estimation

#endif  // AUTOWARE__SHAPE_ESTIMATION__CORRECTOR__UTILS_HPP_
