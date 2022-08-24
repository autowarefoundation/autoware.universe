// Copyright 2018 Autoware Foundation. All rights reserved.
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

#include "shape_estimation/corrector/car_corrector.hpp"

bool CarCorrector::correct(
  autoware_auto_perception_msgs::msg::Shape & shape, geometry_msgs::msg::Pose & pose)
{
  const bool use_reference_shape_size = ref_shape_size_info_ ? true : false;
  const bool use_reference_shape = use_reference_yaw_ && use_reference_shape_size;

  if (use_reference_shape) {  // use reference yaw and shape size
    return corrector_utils::correctVehicleBoundingBoxWithReferenceShape(
      ref_shape_size_info_.get(), shape, pose);
  } else if (use_reference_yaw_) {  // use reference yaw
    return corrector_utils::correctVehicleBoundingBoxWithReferenceYaw(params_, shape, pose);
  } else if (use_reference_shape_size) {  // use reference shape size
    RCLCPP_WARN(
      rclcpp::get_logger("shape_estimation"),
      "Currently, not support reference yaw is false, reference size is true");
    return corrector_utils::correctVehicleBoundingBox(
      params_, shape, pose);  // TODO(yukkysaito) implement only reference shape size version
  }
  return correctVehicleBoundingBox(params_, shape, pose);
}
