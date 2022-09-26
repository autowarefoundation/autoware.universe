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

#include "pointcloud_map_loader_module.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

sensor_msgs::msg::PointCloud2 downsample(
  const sensor_msgs::msg::PointCloud2 & msg_input, const float leaf_size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(msg_input, *pcl_input);
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(pcl_input);
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.filter(*pcl_output);

  sensor_msgs::msg::PointCloud2 msg_output;
  pcl::toROSMsg(*pcl_output, msg_output);
  msg_output.header = msg_input.header;
  return msg_output;
}

PointcloudMapLoaderModule::PointcloudMapLoaderModule(
  rclcpp::Node * node, const std::vector<std::string> & pcd_paths, const std::string publisher_name,
  const boost::optional<float> leaf_size)
: logger_(node->get_logger())
{
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_pointcloud_map_ =
    node->create_publisher<sensor_msgs::msg::PointCloud2>(publisher_name, durable_qos);
  sensor_msgs::msg::PointCloud2 pcd = loadPCDFiles(pcd_paths, leaf_size);

  if (pcd.width == 0) {
    RCLCPP_ERROR(logger_, "No PCD was loaded: pcd_paths.size() = %zu", pcd_paths.size());
    return;
  }

  pcd.header.frame_id = "map";
  pub_pointcloud_map_->publish(pcd);
}

sensor_msgs::msg::PointCloud2 PointcloudMapLoaderModule::loadPCDFiles(
  const std::vector<std::string> & pcd_paths, const boost::optional<float> leaf_size) const
{
  sensor_msgs::msg::PointCloud2 whole_pcd;
  sensor_msgs::msg::PointCloud2 partial_pcd;

  for (int i = 0; i < int(pcd_paths.size()); ++i) {
    auto & path = pcd_paths[i];
    if (i % 50 == 0) {
      RCLCPP_INFO_STREAM(
        logger_, "Load " << path << " (" << i + 1 << " out of " << int(pcd_paths.size()) << ")");
    }

    if (pcl::io::loadPCDFile(path, partial_pcd) == -1) {
      RCLCPP_ERROR_STREAM(logger_, "PCD load failed: " << path);
    }

    if (leaf_size) {
      partial_pcd = downsample(partial_pcd, leaf_size.get());
    }

    if (whole_pcd.width == 0) {
      whole_pcd = partial_pcd;
    } else {
      whole_pcd.width += partial_pcd.width;
      whole_pcd.row_step += partial_pcd.row_step;
      whole_pcd.data.insert(whole_pcd.data.end(), partial_pcd.data.begin(), partial_pcd.data.end());
    }
  }

  whole_pcd.header.frame_id = "map";

  return whole_pcd;
}
