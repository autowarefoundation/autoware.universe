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

#ifndef DIFFERENTIAL_MAP_LOADING_MODULE_HPP_
#define DIFFERENTIAL_MAP_LOADING_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "autoware_map_msgs/srv/load_pcd_maps_general.hpp"

#include <string>
#include <vector>
#include <map>

struct PCDFileMetadata
{
  pcl::PointXYZ min;
  pcl::PointXYZ max;
};

class DifferentialMapLoadingModule
{

public:
  explicit DifferentialMapLoadingModule(
    rclcpp::Node * node, const std::vector<std::string> pcd_paths);
  
private:
  rclcpp::Logger logger_;

  std::map<std::string, PCDFileMetadata> all_pcd_file_metadata_dict_;
  rclcpp::Service<autoware_map_msgs::srv::LoadPCDMapsGeneral>::SharedPtr
    load_pcd_maps_general_service_;

  bool loadPCDMapsGeneralCallback(
    autoware_map_msgs::srv::LoadPCDMapsGeneral::Request::SharedPtr req,
    autoware_map_msgs::srv::LoadPCDMapsGeneral::Response::SharedPtr res);
  void partialAreaLoad(const autoware_map_msgs::msg::AreaInfo area,
    autoware_map_msgs::srv::LoadPCDMapsGeneral::Response::SharedPtr & response) const;
  void differentialAreaLoad(const autoware_map_msgs::msg::AreaInfo area_info,
    const std::vector<std::string> already_loaded_ids,
    autoware_map_msgs::srv::LoadPCDMapsGeneral::Response::SharedPtr & response) const;
  void loadPCDMapWithID(
    const std::string path, const std::string map_id,
    autoware_map_msgs::msg::PCDMapWithID & pcd_map_with_id) const;
  std::map<std::string, PCDFileMetadata> generatePCDMetadata(
    const std::vector<std::string> & pcd_paths) const;
};

#endif  // DIFFERENTIAL_MAP_LOADING_MODULE_HPP_