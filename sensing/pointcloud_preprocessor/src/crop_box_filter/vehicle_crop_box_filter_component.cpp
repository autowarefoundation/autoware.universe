// Copyright 2022 Tier IV, Inc.
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

#include "pointcloud_preprocessor/crop_box_filter/crop_box_filter_nodelet.hpp"

#include <vehicle_info_util/vehicle_info_util.hpp>

namespace pointcloud_preprocessor
{
class VehicleCropBoxFilterComponent final : public CropBoxFilterComponent
{
public:
  explicit VehicleCropBoxFilterComponent(const rclcpp::NodeOptions & node_options)
  : CropBoxFilterComponent(node_options)
  {
    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
    set_parameter(rclcpp::Parameter{"min_x", vehicle_info.min_longitudinal_offset_m});
    set_parameter(rclcpp::Parameter{"min_y", vehicle_info.min_lateral_offset_m});
    set_parameter(rclcpp::Parameter{"min_z", vehicle_info.min_height_offset_m});
    set_parameter(rclcpp::Parameter{"max_x", vehicle_info.max_longitudinal_offset_m});
    set_parameter(rclcpp::Parameter{"max_y", vehicle_info.max_lateral_offset_m});
    set_parameter(rclcpp::Parameter{"max_z", vehicle_info.max_height_offset_m});
  }
};
}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::VehicleCropBoxFilterComponent)
