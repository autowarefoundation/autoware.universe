// Copyright 2021 TierIV
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

/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#include "map_loader/lanelet2_map_loader_node.hpp"

#include "lanelet2_local_projector.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <geography_utils/lanelet2_projector.hpp>
#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/projection/transverse_mercator_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <filesystem>
#include <string>

namespace
{
bool isOsmFile(const std::string & f)
{
  if (std::filesystem::is_directory(f)) {
    return false;
  }

  const std::string ext = std::filesystem::path(f).extension();

  if (ext != ".osm" && ext != ".OSM") {
    return false;
  }

  return true;
}
}  // namespace

Lanelet2MapLoaderNode::Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_loader", options)
{
  declare_parameter<std::string>("lanelet2_map_path");
  declare_parameter<double>("center_line_resolution");

  if (isOsmFile(get_parameter("lanelet2_map_path").as_string())) {
    RCLCPP_INFO(get_logger(), "Single osm file provided. Dynamic loading is disabled.");

    const auto adaptor = component_interface_utils::NodeAdaptor(this);
    adaptor.init_sub(
      sub_map_projector_info_,
      [this](const MapProjectorInfo::Message::ConstSharedPtr msg) { on_map_projector_info(msg); });
  } else {
    RCLCPP_INFO(get_logger(), "Multiple osm file provided. Dynamic loading is enabled.");

    std::vector<std::string> lanelet2_paths_or_directory = {
      get_parameter("lanelet2_map_path").as_string()};
    std::string lanelet2_map_metadata_path =
      declare_parameter<std::string>("lanelet2_map_metadata_path");
    double x_resolution, y_resolution;

    std::map<std::string, Lanelet2FileMetaData> lanelet2_metadata_dict;
    try {
      lanelet2_metadata_dict = getLanelet2Metadata(
        lanelet2_map_metadata_path, getLanelet2Paths(lanelet2_paths_or_directory), x_resolution,
        y_resolution);
    } catch (std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to load lanelet2 metadata");
    }
    differential_loader_module_ = std::make_unique<Lanelet2DifferentialLoaderModule>(
      this, lanelet2_metadata_dict, x_resolution, y_resolution,
      get_parameter("center_line_resolution").as_double());
  }
}

void Lanelet2MapLoaderNode::on_map_projector_info(
  const MapProjectorInfo::Message::ConstSharedPtr msg)
{
  const auto lanelet2_filename = get_parameter("lanelet2_map_path").as_string();
  const auto center_line_resolution = get_parameter("center_line_resolution").as_double();

  // load map from file
  const auto map = load_map(lanelet2_filename, *msg);
  if (!map) {
    RCLCPP_ERROR(get_logger(), "Failed to load lanelet2_map. Not published.");
    return;
  }

  // overwrite centerline
  lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);

  // create map bin msg
  const auto map_bin_msg = create_map_bin_msg(map, lanelet2_filename, now());

  // create publisher and publish
  pub_map_bin_ =
    create_publisher<HADMapBin>("output/lanelet2_map", rclcpp::QoS{1}.transient_local());
  pub_map_bin_->publish(map_bin_msg);
  RCLCPP_INFO(get_logger(), "Succeeded to load lanelet2_map. Map is published.");
}

lanelet::LaneletMapPtr Lanelet2MapLoaderNode::load_map(
  const std::string & lanelet2_filename,
  const tier4_map_msgs::msg::MapProjectorInfo & projector_info)
{
  lanelet::ErrorMessages errors{};
  if (projector_info.projector_type != tier4_map_msgs::msg::MapProjectorInfo::LOCAL) {
    std::unique_ptr<lanelet::Projector> projector =
      geography_utils::get_lanelet2_projector(projector_info);
    const lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, *projector, &errors);
    if (errors.empty()) {
      return map;
    }
  } else {
    const lanelet::projection::LocalProjector projector;
    const lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);

    if (!errors.empty()) {
      for (const auto & error : errors) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
      }
    }

    // overwrite local_x, local_y
    for (lanelet::Point3d point : map->pointLayer) {
      if (point.hasAttribute("local_x")) {
        point.x() = point.attribute("local_x").asDouble().value();
      }
      if (point.hasAttribute("local_y")) {
        point.y() = point.attribute("local_y").asDouble().value();
      }
    }

    // realign lanelet borders using updated points
    for (lanelet::Lanelet lanelet : map->laneletLayer) {
      auto left = lanelet.leftBound();
      auto right = lanelet.rightBound();
      std::tie(left, right) = lanelet::geometry::align(left, right);
      lanelet.setLeftBound(left);
      lanelet.setRightBound(right);
    }

    return map;
  }

  for (const auto & error : errors) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
  }
  return nullptr;
}

HADMapBin Lanelet2MapLoaderNode::create_map_bin_msg(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename, const rclcpp::Time & now)
{
  std::string format_version{}, map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  HADMapBin map_bin_msg;
  map_bin_msg.header.stamp = now;
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.format_version = format_version;
  map_bin_msg.map_version = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  return map_bin_msg;
}

LaneletMapBin Lanelet2MapLoaderNode::create_lanelet_map_bin_msg(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename, const rclcpp::Time & now)
{
  std::string format_version{}, map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  LaneletMapBin map_bin_msg;
  map_bin_msg.header.stamp = now;
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.version_map_format = format_version;
  map_bin_msg.version_map = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  return map_bin_msg;
}

std::vector<std::string> Lanelet2MapLoaderNode::getLanelet2Paths(
  const std::vector<std::string> & lanelet2_paths_or_directory) const
{
  std::vector<std::string> lanelet2_paths;
  for (const auto & path : lanelet2_paths_or_directory) {
    if (!std::filesystem::exists(path)) {
      RCLCPP_ERROR_STREAM(get_logger(), "No such file or directory: " << path);
      continue;
    }

    if (isOsmFile(path)) {
      lanelet2_paths.push_back(path);
    }

    if (std::filesystem::is_directory(path)) {
      for (const auto & file : std::filesystem::directory_iterator(path)) {
        const auto filename = file.path().string();
        if (isOsmFile(filename)) {
          lanelet2_paths.push_back(filename);
        }
      }
    }
  }
  return lanelet2_paths;
}

std::map<std::string, Lanelet2FileMetaData> Lanelet2MapLoaderNode::getLanelet2Metadata(
  const std::string & lanelet2_metadata_path, const std::vector<std::string> & lanelet2_paths,
  double & x_resolution, double & y_resolution) const
{
  std::map<std::string, Lanelet2FileMetaData> lanelet2_metadata_dict;
  if (!std::filesystem::exists(lanelet2_metadata_path)) {
    throw std::runtime_error("Lanelet2 metadata file not found: " + lanelet2_metadata_path);
  }

  lanelet2_metadata_dict = loadLanelet2Metadata(lanelet2_metadata_path, x_resolution, y_resolution);
  lanelet2_metadata_dict = replaceWithAbsolutePath(lanelet2_metadata_dict, lanelet2_paths);
  RCLCPP_INFO_STREAM(get_logger(), "Loaded Lanelet2 metadata: " << lanelet2_metadata_path);

  return lanelet2_metadata_dict;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Lanelet2MapLoaderNode)
