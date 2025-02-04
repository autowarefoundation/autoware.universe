// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include "autoware/shape_estimation/corrector/reference_object_based_corrector.hpp"

namespace autoware::shape_estimation
{
namespace corrector
{
bool ReferenceObjectBasedVehicleCorrector::correct(
  autoware_perception_msgs::msg::Shape & shape, geometry_msgs::msg::Pose & pose)
{
  return corrector_utils::correctWithReferenceShapeAndPose(
    ref_shape_size_info_, ref_pose_, shape, pose);
}

}  // namespace corrector
}  // namespace autoware::shape_estimation
