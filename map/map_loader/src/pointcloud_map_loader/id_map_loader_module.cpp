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

#include "id_map_loader_module.hpp"

IdMapLoaderModule::IdMapLoaderModule(
  rclcpp::Node * node, const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict)
: logger_(node->get_logger()), all_pcd_file_metadata_dict_(pcd_file_metadata_dict)
{
  get_id_pcd_maps_service_ = node->create_service<GetIdPointCloudMap>(
    "service/get_id_pcd_map", std::bind(
                                &IdMapLoaderModule::onServiceGetIdPointCloudMap, this,
                                std::placeholders::_1, std::placeholders::_2));
  // TODO: initialize publisher and publish the map metadata
}

bool IdMapLoaderModule::onServiceGetIdPointCloudMap(
  GetIdPointCloudMap::Request::SharedPtr req, GetIdPointCloudMap::Response::SharedPtr res)
{
  const auto request_ids = req->cell_ids;
  for (const auto & request_id : request_ids) {
    const auto requested_id_map_iterator = all_pcd_file_metadata_dict_.find(request_id);

    // skip if the requested ID is not found
    if (requested_id_map_iterator == all_pcd_file_metadata_dict_.end()) {
      RCLCPP_WARN(logger_, "ID %s not found", request_id.c_str());
      continue;
    }

    const std::string path = requested_id_map_iterator->first;
    // assume that the map ID = map path (for now)
    const std::string map_id = path;
    PCDFileMetadata metadata = requested_id_map_iterator->second;

    autoware_map_msgs::msg::PointCloudMapCellWithID pointcloud_map_cell_with_id =
      loadPointCloudMapCellWithID(path, map_id);
    pointcloud_map_cell_with_id.min_x = metadata.min.x;
    pointcloud_map_cell_with_id.min_y = metadata.min.y;
    pointcloud_map_cell_with_id.max_x = metadata.max.x;
    pointcloud_map_cell_with_id.max_y = metadata.max.y;

    res->new_pointcloud_with_ids.push_back(pointcloud_map_cell_with_id);
  }
  res->header.frame_id = "map";
  return true;
}

autoware_map_msgs::msg::PointCloudMapCellWithID IdMapLoaderModule::loadPointCloudMapCellWithID(
  const std::string & path, const std::string & map_id) const
{
  sensor_msgs::msg::PointCloud2 pcd;
  if (pcl::io::loadPCDFile(path, pcd) == -1) {
    RCLCPP_ERROR_STREAM(logger_, "PCD load failed: " << path);
  }
  autoware_map_msgs::msg::PointCloudMapCellWithID pointcloud_map_cell_with_id;
  pointcloud_map_cell_with_id.pointcloud = pcd;
  pointcloud_map_cell_with_id.cell_id = map_id;
  return pointcloud_map_cell_with_id;
}
