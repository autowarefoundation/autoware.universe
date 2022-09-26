// Copyright 2022 The Autoware Contributors
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

#include "differential_map_loader_module.hpp"

bool sphereAndBoxOverlapExists(
  const geometry_msgs::msg::Point position,
  const double radius,
  const pcl::PointXYZ position_min,
  const pcl::PointXYZ position_max)
{
  if (
    (position_min.x - radius <= position.x && position.x <= position_max.x + radius &&
     position_min.y <= position.y && position.y <= position_max.y && position_min.z <= position.z &&
     position.z <= position_max.z) ||
    (position_min.x <= position.x && position.x <= position_max.x &&
     position_min.y - radius <= position.y && position.y <= position_max.y + radius &&
     position_min.z <= position.z && position.z <= position_max.z) ||
    (position_min.x <= position.x && position.x <= position_max.x && position_min.y <= position.y &&
     position.y <= position_max.y && position_min.z - radius <= position.z &&
     position.z <= position_max.z + radius)) {
    return true;
  }
  double r2 = std::pow(radius, 2.0);
  double minx2 = std::pow(position.x - position_min.x, 2.0);
  double maxx2 = std::pow(position.x - position_max.x, 2.0);
  double miny2 = std::pow(position.y - position_min.y, 2.0);
  double maxy2 = std::pow(position.y - position_max.y, 2.0);
  double minz2 = std::pow(position.z - position_min.z, 2.0);
  double maxz2 = std::pow(position.z - position_max.z, 2.0);
  if (
    minx2 + miny2 + minz2 <= r2 || minx2 + miny2 + maxz2 <= r2 || minx2 + maxy2 + minz2 <= r2 ||
    minx2 + maxy2 + maxz2 <= r2 || maxx2 + miny2 + minz2 <= r2 || maxx2 + miny2 + maxz2 <= r2 ||
    maxx2 + maxy2 + minz2 <= r2 || maxx2 + maxy2 + maxz2 <= r2) {
    return true;
  }
  return false;
}

bool isGridWithinQueriedArea(
  const autoware_map_msgs::msg::AreaInfo area,
  const PCDFileMetadata metadata)
{
  // Currently, the area load only supports spherical area
  geometry_msgs::msg::Point position = area.center;
  double radius = area.radius;
  bool res = sphereAndBoxOverlapExists(position, radius, metadata.min, metadata.max);
  return res;
}

DifferentialMapLoaderModule::DifferentialMapLoaderModule(
  rclcpp::Node * node, const std::vector<std::string> pcd_paths)
: logger_(node->get_logger())
{
  all_pcd_file_metadata_dict_ = generatePCDMetadata(pcd_paths);
  load_pcd_maps_general_service_ = node->create_service<autoware_map_msgs::srv::LoadPCDMapsGeneral>(
    "load_pcd_maps_general", std::bind(
                            &DifferentialMapLoaderModule::loadPCDMapsGeneralCallback, this,
                            std::placeholders::_1, std::placeholders::_2));
}

void DifferentialMapLoaderModule::differentialAreaLoad(
  const autoware_map_msgs::msg::AreaInfo area,
  const std::vector<std::string> already_loaded_ids,
  autoware_map_msgs::srv::LoadPCDMapsGeneral::Response::SharedPtr & response) const
{
  // iterate over all the available pcd map grids
  for (const auto & ele : all_pcd_file_metadata_dict_) {
    std::string path = ele.first;
    PCDFileMetadata metadata = ele.second;

    // assume that the map ID = map path (for now)
    std::string map_id = path;

    // skip if the pcd file is not within the queried area
    if (!isGridWithinQueriedArea(area, metadata)) continue;

    bool is_already_loaded = std::find(already_loaded_ids.begin(), already_loaded_ids.end(), map_id) != already_loaded_ids.end();
    if (is_already_loaded) {
      response->already_loaded_ids.push_back(map_id);
    } else {
      autoware_map_msgs::msg::PCDMapWithID pcd_map_with_id;
      loadPCDMapWithID(path, map_id, pcd_map_with_id);
      response->loaded_pcds.push_back(pcd_map_with_id);
    }
  }
  RCLCPP_INFO_STREAM(logger_, "Finished diff area loading");
}

void DifferentialMapLoaderModule::partialAreaLoad(
  const autoware_map_msgs::msg::AreaInfo area,
  autoware_map_msgs::srv::LoadPCDMapsGeneral::Response::SharedPtr & response) const
{
  // iterate over all the available pcd map grids

  for (const auto & ele : all_pcd_file_metadata_dict_) {
    std::string path = ele.first;
    PCDFileMetadata metadata = ele.second;

    // assume that the map ID = map path (for now)
    std::string map_id = path;

    // skip if the pcd file is not within the queried area
    if (!isGridWithinQueriedArea(area, metadata)) continue;

    autoware_map_msgs::msg::PCDMapWithID pcd_map_with_id;
    loadPCDMapWithID(path, map_id, pcd_map_with_id);
    response->loaded_pcds.push_back(pcd_map_with_id);
  }
}

bool DifferentialMapLoaderModule::loadPCDMapsGeneralCallback(
  autoware_map_msgs::srv::LoadPCDMapsGeneral::Request::SharedPtr req,
  autoware_map_msgs::srv::LoadPCDMapsGeneral::Response::SharedPtr res)
{
  int mode = req->mode;
  if (mode == 0) {
    auto area = req->area;
    partialAreaLoad(area, res);
  } else if (mode == 1) {
    auto area = req->area;
    std::vector<std::string> already_loaded_ids = req->already_loaded_ids;
    differentialAreaLoad(area, already_loaded_ids, res);
  }
  return true;
}

void DifferentialMapLoaderModule::loadPCDMapWithID(
  const std::string path, const std::string map_id,
  autoware_map_msgs::msg::PCDMapWithID & pcd_map_with_id) const
{
  sensor_msgs::msg::PointCloud2 pcd;
  if (pcl::io::loadPCDFile(path, pcd) == -1) {
    RCLCPP_ERROR_STREAM(logger_, "PCD load failed: " << path);
  }
  pcd_map_with_id.pcd = pcd;
  pcd_map_with_id.id = map_id;
}

std::map<std::string, PCDFileMetadata> DifferentialMapLoaderModule::generatePCDMetadata(
  const std::vector<std::string> & pcd_paths) const
{
  pcl::PointCloud<pcl::PointXYZ> partial_pcd;
  std::map<std::string, PCDFileMetadata> all_pcd_file_metadata_dict;
  for (const auto & path : pcd_paths) {
    if (pcl::io::loadPCDFile(path, partial_pcd) == -1) {
      RCLCPP_ERROR_STREAM(logger_, "PCD load failed: " << path);
    }
    PCDFileMetadata metadata = {};
    pcl::getMinMax3D(partial_pcd, metadata.min, metadata.max);
    all_pcd_file_metadata_dict[path] = metadata;
  }
  return all_pcd_file_metadata_dict;
}
