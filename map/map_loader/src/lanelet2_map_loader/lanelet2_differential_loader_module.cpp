// Copyright 2024 The Autoware Contributors
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

#include "map_loader/lanelet2_differential_loader_module.hpp"

#include "lanelet2_local_projector.hpp"
#include "map_loader/lanelet2_map_loader_node.hpp"

Lanelet2DifferentialLoaderModule::Lanelet2DifferentialLoaderModule(
  rclcpp::Node * node,
  const std::map<std::string, Lanelet2FileMetaData> & lanelet2_file_metadata_dict, double & x_res,
  double & y_res, const double & center_line_resolution)
: logger_(node->get_logger()),
  clock_(node->get_clock()),
  lanelet2_file_metadata_dict_(lanelet2_file_metadata_dict),
  center_line_resolution_(center_line_resolution)
{
  pub_whole_map_bin_ =
    node->create_publisher<HADMapBin>("output/lanelet2_map", rclcpp::QoS{1}.transient_local());

  const auto metadata_adaptor = component_interface_utils::NodeAdaptor(node);
  metadata_adaptor.init_pub(pub_lanelet_map_meta_data_);

  const auto adaptor = component_interface_utils::NodeAdaptor(node);
  adaptor.init_sub(
    sub_map_projector_info_,
    [this, &x_res, &y_res](const map_interface::MapProjectorInfo::Message::ConstSharedPtr msg) {
      projector_info_ = *msg;

      // load whole map once when projector info is available
      {
        const auto map = loadWholeMap();
        pub_whole_map_bin_->publish(map);
      }

      // publish lanelet2 map metadata when projector info is available
      {
        const auto msg = getLaneletMapMetaDataMsg(x_res, y_res);
        pub_lanelet_map_meta_data_->publish(msg);
      }
    });

  get_differential_lanelet2_maps_service_ = node->create_service<GetDifferentialLanelet2Map>(
    "service/get_differential_lanelet_map",
    std::bind(
      &Lanelet2DifferentialLoaderModule::onServiceGetDifferentialLanelet2Map, this,
      std::placeholders::_1, std::placeholders::_2));
}

bool Lanelet2DifferentialLoaderModule::onServiceGetDifferentialLanelet2Map(
  GetDifferentialLanelet2Map::Request::SharedPtr req,
  GetDifferentialLanelet2Map::Response::SharedPtr res)
{
  if (!differentialLanelet2Load(req, res)) {
    RCLCPP_ERROR(logger_, "Failed to load differential lanelet2 map");
    return false;
  }
  res->header.frame_id = "map";

  return true;
}

autoware_auto_mapping_msgs::msg::HADMapBin Lanelet2DifferentialLoaderModule::loadWholeMap()
{
  std::vector<std::string> lanelet2_paths;
  for (const auto & file : lanelet2_file_metadata_dict_) {
    if (!std::filesystem::exists(file.first)) {
      continue;
    }
    lanelet2_paths.push_back(file.first);
  }

  if (projector_info_.value().projector_type != tier4_map_msgs::msg::MapProjectorInfo::LOCAL) {
    std::unique_ptr<lanelet::Projector> projector =
      geography_utils::get_lanelet2_projector(projector_info_.value());

    lanelet::ErrorMessages errors{};
    lanelet::io_handlers::MultiOsmParser parser(*projector);
    lanelet::LaneletMapPtr map = parser.parse(lanelet2_paths, errors);

    if (!errors.empty()) {
      for (const auto & error : errors) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
      }
    }

    lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution_, false);

    const auto map_bin_msg =
      Lanelet2MapLoaderNode::create_map_bin_msg(map, lanelet2_paths[0], rclcpp::Clock().now());

    return map_bin_msg;
  } else {
    const lanelet::projection::LocalProjector projector;
    lanelet::ErrorMessages errors{};
    lanelet::io_handlers::MultiOsmParser parser(projector);
    lanelet::LaneletMapPtr map = parser.parse(lanelet2_paths, errors);

    if (!errors.empty()) {
      for (const auto & error : errors) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
      }
    }

    lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution_, false);

    const auto map_bin_msg =
      Lanelet2MapLoaderNode::create_map_bin_msg(map, lanelet2_paths[0], rclcpp::Clock().now());

    return map_bin_msg;
  }
}

bool Lanelet2DifferentialLoaderModule::differentialLanelet2Load(
  GetDifferentialLanelet2Map::Request::SharedPtr & request,
  GetDifferentialLanelet2Map::Response::SharedPtr & response)
{
  // check osm ids
  std::vector<std::string> lanelet2_paths;
  for (const auto & id : request->vector_map_file_ids) {
    auto it = std::find_if(
      lanelet2_file_metadata_dict_.begin(), lanelet2_file_metadata_dict_.end(),
      [&id](const auto & file) { return file.second.id == id; });
    if (it == lanelet2_file_metadata_dict_.end()) {
      continue;
    }
    if (!std::filesystem::exists(it->first)) {
      continue;
    }
    lanelet2_paths.push_back(it->first);
  }
  if (lanelet2_paths.empty()) {
    return false;
  }

  if (projector_info_.value().projector_type != tier4_map_msgs::msg::MapProjectorInfo::LOCAL) {
    std::unique_ptr<lanelet::Projector> projector =
      geography_utils::get_lanelet2_projector(projector_info_.value());

    lanelet::ErrorMessages errors{};
    lanelet::io_handlers::MultiOsmParser parser(*projector);
    lanelet::LaneletMapPtr map = parser.parse(lanelet2_paths, errors);

    if (!errors.empty()) {
      for (const auto & error : errors) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
      }
    }

    lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution_, false);

    const auto map_bin_msg = Lanelet2MapLoaderNode::create_lanelet_map_bin_msg(
      map, lanelet2_paths[0], rclcpp::Clock().now());

    response->differential_map = map_bin_msg;
  } else {
    const lanelet::projection::LocalProjector projector;
    lanelet::ErrorMessages errors{};
    lanelet::io_handlers::MultiOsmParser parser(projector);
    lanelet::LaneletMapPtr map = parser.parse(lanelet2_paths, errors);

    if (!errors.empty()) {
      for (const auto & error : errors) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
      }
    }

    lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution_, false);

    const auto map_bin_msg = Lanelet2MapLoaderNode::create_lanelet_map_bin_msg(
      map, lanelet2_paths[0], rclcpp::Clock().now());

    response->differential_map = map_bin_msg;
  }

  return true;
}

autoware_map_msgs::msg::LaneletMapMetaData
Lanelet2DifferentialLoaderModule::getLaneletMapMetaDataMsg(
  const double & x_res, const double & y_res) const
{
  std::unique_ptr<lanelet::Projector> projector;
  if (projector_info_.value().projector_type != tier4_map_msgs::msg::MapProjectorInfo::LOCAL) {
    projector = geography_utils::get_lanelet2_projector(projector_info_.value());
  } else {
    projector = std::make_unique<lanelet::projection::LocalProjector>();
  }

  autoware_map_msgs::msg::LaneletMapMetaData metadata;
  for (const auto & file : lanelet2_file_metadata_dict_) {
    lanelet::GPSPoint gps_point;
    gps_point.lat = file.second.origin_lat;
    gps_point.lon = file.second.origin_lon;
    gps_point.ele = 0;
    lanelet::BasicPoint3d point = projector->forward(gps_point);

    autoware_map_msgs::msg::LaneletMapTileMetaData tile_msg;
    tile_msg.tile_id = file.second.id;
    tile_msg.min_x = point.x();
    tile_msg.min_y = point.y();
    tile_msg.max_x = point.x() + x_res;
    tile_msg.max_y = point.y() + y_res;

    metadata.metadata_list.push_back(tile_msg);
  }
  metadata.header.frame_id = "map";
  metadata.header.stamp = clock_->now();

  return metadata;
}
