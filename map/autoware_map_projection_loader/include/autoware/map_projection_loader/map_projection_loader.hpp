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

#ifndef AUTOWARE__MAP_PROJECTION_LOADER__MAP_PROJECTION_LOADER_HPP_
#define AUTOWARE__MAP_PROJECTION_LOADER__MAP_PROJECTION_LOADER_HPP_

#include "rclcpp/rclcpp.hpp"

#include <autoware/component_interface_specs_universe/map.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>

#include <string>

namespace autoware::map_projection_loader
{
autoware_map_msgs::msg::MapProjectorInfo load_info_from_yaml(const std::string & filename);
autoware_map_msgs::msg::MapProjectorInfo load_map_projector_info(
  const std::string & yaml_filename, const std::string & lanelet2_map_filename);

class MapProjectionLoader : public rclcpp::Node
{
public:
  explicit MapProjectionLoader(const rclcpp::NodeOptions & options);

private:
  using MapProjectorInfo = autoware::component_interface_specs_universe::map::MapProjectorInfo;
  autoware::component_interface_utils::Publisher<MapProjectorInfo>::SharedPtr publisher_;
};
}  // namespace autoware::map_projection_loader

#endif  // AUTOWARE__MAP_PROJECTION_LOADER__MAP_PROJECTION_LOADER_HPP_
