// Copyright 2024 Autoware Foundation
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

#ifndef STUB_PCD_LOADER_HPP_
#define STUB_PCD_LOADER_HPP_

#include "test_util.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_map_msgs/srv/get_differential_point_cloud_map.hpp"

class StubPcdLoader : public rclcpp::Node
{
  using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

public:
  StubPcdLoader() : Node("stub_pcd_loader")
  {
    get_differential_pcd_maps_service_ = create_service<GetDifferentialPointCloudMap>(
      "pcd_loader_service", std::bind(
                              &StubPcdLoader::on_service_get_differential_point_cloud_map, this,
                              std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::Service<GetDifferentialPointCloudMap>::SharedPtr get_differential_pcd_maps_service_;

  bool on_service_get_differential_point_cloud_map(
    GetDifferentialPointCloudMap::Request::SharedPtr req,
    GetDifferentialPointCloudMap::Response::SharedPtr res)
  {
    (void)req;
    autoware_map_msgs::msg::PointCloudMapCellWithID pcd_map_cell_with_id;
    constexpr float max_position = 20.0f;
    constexpr float interval = 0.2f;
    pcd_map_cell_with_id.metadata.min_x = -max_position;
    pcd_map_cell_with_id.metadata.min_y = -max_position;
    pcd_map_cell_with_id.metadata.max_x = max_position;
    pcd_map_cell_with_id.metadata.max_y = max_position;
    pcd_map_cell_with_id.cell_id = "0";
    pcl::PointCloud<pcl::PointXYZ> cloud = make_sample_pcd(-max_position, max_position, interval);
    pcl::toROSMsg(cloud, pcd_map_cell_with_id.pointcloud);
    res->new_pointcloud_with_ids.push_back(pcd_map_cell_with_id);
    res->header.frame_id = "map";
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return true;
  }
};

#endif  // STUB_PCD_LOADER_HPP_
