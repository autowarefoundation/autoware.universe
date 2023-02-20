// Copyright 2023 TIER IV, Inc.
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

#include "vehicle_info.hpp"

#include <vehicle_info_util/vehicle_info_util.hpp>

namespace default_ad_api
{

VehicleInfoNode::VehicleInfoNode(const rclcpp::NodeOptions & options)
: Node("vehicle_info", options)
{
  const auto on_vehicle_dimensions = [this](auto, auto res) {
    res->status.success = true;
    res->dimensions = dimensions_;
  };

  const auto vehicle = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  dimensions_.wheel_radius = vehicle.wheel_radius_m;
  dimensions_.wheel_width = vehicle.wheel_width_m;
  dimensions_.wheel_base = vehicle.wheel_base_m;
  dimensions_.wheel_tread = vehicle.wheel_tread_m;
  dimensions_.front_overhang = vehicle.front_overhang_m;
  dimensions_.rear_overhang = vehicle.rear_overhang_m;
  dimensions_.left_overhang = vehicle.left_overhang_m;
  dimensions_.right_overhang = vehicle.right_overhang_m;
  dimensions_.height = vehicle.vehicle_height_m;

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_srv(srv_dimensions_, on_vehicle_dimensions);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::VehicleInfoNode)
