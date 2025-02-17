// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__PERCEPTION_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__PERCEPTION_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

namespace autoware::component_interface_specs_universe::perception
{

struct ObjectRecognition
{
  using Message = autoware_perception_msgs::msg::PredictedObjects;
  static constexpr char name[] = "/perception/object_recognition/objects";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

}  // namespace autoware::component_interface_specs_universe::perception

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__PERCEPTION_HPP_
