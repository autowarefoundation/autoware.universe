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

#include "map_projection_loader/map_projection_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>

MapProjectionLoader::MapProjectionLoader() : Node("map_projection_loader")
{
  tier4_map_msgs::msg::MapProjectorInfo msg;
  if (this->declare_parameter<bool>("use_local_projector")) {
    RCLCPP_INFO(
      this->get_logger(),
      "Use default value for map_projector_info instead of loading file. NOTE: Please consider "
      "providing map_projector_info.yaml and setting `load_info_from_file` to false");
    msg.type = "local";
  } else {
    std::string filename = this->declare_parameter<std::string>("map_projector_info_path");
    std::ifstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s.", filename.c_str());
      return;
    }
    YAML::Node data = YAML::LoadFile(filename);

    // Assume the YAML file has a "type" key for map_projector_info.
    msg.type = data["type"].as<std::string>();
    msg.mgrs_grid = data["mgrs_grid"].as<std::string>();
  }

  // Publish the message
  publisher_ = this->create_publisher<tier4_map_msgs::msg::MapProjectorInfo>(
    "~/map_projector_info", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  publisher_->publish(msg);
}
