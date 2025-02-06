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

#include "autoware/map_projection_loader/map_projection_loader.hpp"

#include "autoware/map_projection_loader/load_info_from_lanelet2_map.hpp"

#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace autoware::map_projection_loader
{
autoware_map_msgs::msg::MapProjectorInfo load_info_from_yaml(const std::string & filename)
{
  YAML::Node data = YAML::LoadFile(filename);

  autoware_map_msgs::msg::MapProjectorInfo msg;
  msg.projector_type = data["projector_type"].as<std::string>();
  if (msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::MGRS) {
    msg.vertical_datum = data["vertical_datum"].as<std::string>();
    msg.mgrs_grid = data["mgrs_grid"].as<std::string>();

  } else if (
    msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL_CARTESIAN_UTM ||
    msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::TRANSVERSE_MERCATOR) {
    msg.vertical_datum = data["vertical_datum"].as<std::string>();
    msg.map_origin.latitude = data["map_origin"]["latitude"].as<double>();
    msg.map_origin.longitude = data["map_origin"]["longitude"].as<double>();
    msg.map_origin.altitude = data["map_origin"]["altitude"].as<double>();

  } else if (msg.projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL) {
    ;  // do nothing

  } else if (msg.projector_type == "local") {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("MapProjectionLoader"),
      "Load " << filename << "\n"
              << "DEPRECATED WARNING: projector type \"local\" is deprecated."
                 "Please use \"Local\" instead. For more info, visit "
                 "https://github.com/autowarefoundation/autoware.universe/blob/main/map/"
                 "map_projection_loader README.md");
    msg.projector_type = autoware_map_msgs::msg::MapProjectorInfo::LOCAL;
  } else {
    throw std::runtime_error(
      "Invalid map projector type. Currently supported types: MGRS, LocalCartesianUTM, "
      "TransverseMercator, and Local");
  }
  return msg;
}

autoware_map_msgs::msg::MapProjectorInfo load_map_projector_info(
  const std::string & yaml_filename, const std::string & lanelet2_map_filename)
{
  autoware_map_msgs::msg::MapProjectorInfo msg;

  if (std::filesystem::exists(yaml_filename)) {
    std::cout << "Load " << yaml_filename << std::endl;
    msg = load_info_from_yaml(yaml_filename);
  } else if (std::filesystem::exists(lanelet2_map_filename)) {
    std::cout << "Load " << lanelet2_map_filename << std::endl;
    std::cout
      << "DEPRECATED WARNING: Loading map projection info from lanelet2 map may soon be deleted. "
         "Please use map_projector_info.yaml instead. For more info, visit "
         "https://github.com/autowarefoundation/autoware.universe/blob/main/map/"
         "map_projection_loader/"
         "README.md"
      << std::endl;
    msg = load_info_from_lanelet2_map(lanelet2_map_filename);
  } else {
    throw std::runtime_error(
      "No map projector info files found. Please provide either "
      "map_projector_info.yaml or lanelet2_map.osm");
  }
  return msg;
}

MapProjectionLoader::MapProjectionLoader(const rclcpp::NodeOptions & options)
: rclcpp::Node("map_projection_loader", options)
{
  const std::string yaml_filename = this->declare_parameter<std::string>("map_projector_info_path");
  const std::string lanelet2_map_filename =
    this->declare_parameter<std::string>("lanelet2_map_path");

  const autoware_map_msgs::msg::MapProjectorInfo msg =
    load_map_projector_info(yaml_filename, lanelet2_map_filename);

  // Publish the message
  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  adaptor.init_pub(publisher_);
  publisher_->publish(msg);
}
}  // namespace autoware::map_projection_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_projection_loader::MapProjectionLoader)
