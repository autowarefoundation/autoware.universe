// Copyright 2020 Tier IV, Inc.
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
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 */

#ifndef MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_
#define MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_

#include "autoware_map_srvs/srv/load_pcd_partially.hpp"
#include "autoware_map_srvs/srv/load_pcd_partially_for_publish.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>


using PointType = pcl::PointXYZI;

class PointCloudMapLoaderNode : public rclcpp::Node
{
public:
  explicit PointCloudMapLoaderNode(const rclcpp::NodeOptions & options);

private:
  // ros param
  bool use_downsample_;
  bool enable_whole_load_;
  bool enable_partial_load_;
  float leaf_size_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_whole_pointcloud_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_partial_pointcloud_map_;
  rclcpp::Service<autoware_map_srvs::srv::LoadPCDPartially>::SharedPtr load_pcd_partially_service_;
  rclcpp::Service<autoware_map_srvs::srv::LoadPCDPartiallyForPublish>::SharedPtr
    load_pcd_partially_for_publish_service_;

  pcl::PointCloud<PointType> loadPCDFiles(const std::vector<std::string> & pcd_paths);
  struct PCDFileMetadata
  {
    pcl::PointXYZ min;
    pcl::PointXYZ max;
    std::string path;
  };
  std::vector<PCDFileMetadata> pcd_file_metadata_array_;
  std::vector<PCDFileMetadata> generatePCDMetadata(
    const std::vector<std::string> & pcd_paths) const;

  sensor_msgs::msg::PointCloud2 loadPCDPartially(
    const geometry_msgs::msg::Point position, const float radius,
    std::vector<PCDFileMetadata> pcd_file_metadata_array) const;

  bool loadPCDPartiallyForPublishServiceCallback(
    autoware_map_srvs::srv::LoadPCDPartiallyForPublish::Request::SharedPtr req,
    autoware_map_srvs::srv::LoadPCDPartiallyForPublish::Response::SharedPtr res);
  bool loadPCDPartiallyServiceCallback(
    autoware_map_srvs::srv::LoadPCDPartially::Request::SharedPtr req,
    autoware_map_srvs::srv::LoadPCDPartially::Response::SharedPtr res);
};

#endif  // MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_
