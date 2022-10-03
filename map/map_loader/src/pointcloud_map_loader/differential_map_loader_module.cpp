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

DifferentialMapLoaderModule::DifferentialMapLoaderModule(
  rclcpp::Node * node, const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict)
: logger_(node->get_logger()), all_pcd_file_metadata_dict_(pcd_file_metadata_dict)
{
  load_differential_pcd_maps_service_ = node->create_service<LoadDifferentialPointCloudMap>(
    "service/load_differential_pcd_map",
    std::bind(
      &DifferentialMapLoaderModule::onServiceLoadDifferentialPointCloudMap, this,
      std::placeholders::_1, std::placeholders::_2));
}

void DifferentialMapLoaderModule::differentialAreaLoad(
  const autoware_map_msgs::msg::AreaInfo area, const std::vector<std::string> already_loaded_ids,
  LoadDifferentialPointCloudMap::Response::SharedPtr & response) const
{
  // iterate over all the available pcd map grids
  std::vector<bool> should_remove(int(already_loaded_ids.size()), true);
  for (const auto & ele : all_pcd_file_metadata_dict_) {
    std::string path = ele.first;
    PCDFileMetadata metadata = ele.second;

    // assume that the map ID = map path (for now)
    std::string map_id = path;

    // skip if the pcd file is not within the queried area
    if (!isGridWithinQueriedArea(area, metadata)) continue;

    auto id_in_already_loaded_list =
      std::find(already_loaded_ids.begin(), already_loaded_ids.end(), map_id);
    if (id_in_already_loaded_list != already_loaded_ids.end()) {
      int index = id_in_already_loaded_list - already_loaded_ids.begin();
      should_remove[index] = false;
    } else {
      autoware_map_msgs::msg::PointCloudMapWithID pcd_map_with_id;
      loadPointCloudMapWithID(path, map_id, pcd_map_with_id);
      response->loaded_pcds.push_back(pcd_map_with_id);
    }
  }

  for (int i = 0; i < int(already_loaded_ids.size()); ++i) {
    if (should_remove[i]) {
      response->ids_to_remove.push_back(already_loaded_ids[i]);
    }
  }

  RCLCPP_INFO_STREAM(logger_, "Finished diff area loading");
}

bool DifferentialMapLoaderModule::onServiceLoadDifferentialPointCloudMap(
  LoadDifferentialPointCloudMap::Request::SharedPtr req,
  LoadDifferentialPointCloudMap::Response::SharedPtr res)
{
  auto area = req->area;
  std::vector<std::string> already_loaded_ids = req->already_loaded_ids;
  differentialAreaLoad(area, already_loaded_ids, res);
  return true;
}

void DifferentialMapLoaderModule::loadPointCloudMapWithID(
  const std::string path, const std::string map_id,
  autoware_map_msgs::msg::PointCloudMapWithID & pcd_map_with_id) const
{
  sensor_msgs::msg::PointCloud2 pcd;
  if (pcl::io::loadPCDFile(path, pcd) == -1) {
    RCLCPP_ERROR_STREAM(logger_, "PCD load failed: " << path);
  }
  pcd_map_with_id.pointcloud = pcd;
  pcd_map_with_id.id = map_id;
}
