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

#include "map_loader/pointcloud_map_loader_node.hpp"

#include <glob.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
bool isPcdFile(const std::string & p)
{
  if (fs::is_directory(p)) {
    return false;
  }

  const std::string ext = fs::path(p).extension();

  if (ext != ".pcd" && ext != ".PCD") {
    return false;
  }

  return true;
}
}  // namespace

bool sphere_and_box_overlap_exists(
  geometry_msgs::msg::Point position, double radius, pcl::PointXYZ position_min,
  pcl::PointXYZ position_max)
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

pcl::PointCloud<PointType> downsample(pcl::PointCloud<PointType> points, float leaf_size)
{
  pcl::PointCloud<PointType>::Ptr input_points(new pcl::PointCloud<PointType>(points));
  pcl::PointCloud<PointType>::Ptr downsampled_points(new pcl::PointCloud<PointType>);
  pcl::VoxelGrid<PointType> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid_filter.setInputCloud(input_points);
  voxel_grid_filter.filter(*downsampled_points);
  return *downsampled_points.get();
}

PointCloudMapLoaderNode::PointCloudMapLoaderNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_map_loader", options)
{
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  // pub_whole_pointcloud_map_ =
  //   this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud_map/whole", durable_qos);

  const auto pcd_paths_or_directory =
    declare_parameter("pcd_paths_or_directory", std::vector<std::string>({}));

  enable_whole_load_ = declare_parameter("enable_whole_load", true);
  enable_partial_load_ = declare_parameter("enable_partial_load", true);
  use_downsample_ = declare_parameter("use_downsample", false);
  leaf_size_ = declare_parameter("leaf_size", 1.0);

  std::vector<std::string> pcd_paths{};

  if (enable_partial_load_) {
    pub_partial_pointcloud_map_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud_map/partial", durable_qos);
    pcd_file_metadata_array_ = generatePCDMetadata(pcd_paths_or_directory);
    load_pcd_partially_service_ = this->create_service<autoware_map_srvs::srv::LoadPCDPartially>(
      "load_pcd_partially", std::bind(
                              &PointCloudMapLoaderNode::loadPCDPartiallyServiceCallback, this,
                              std::placeholders::_1, std::placeholders::_2));
    load_pcd_partially_for_publish_service_ =
      this->create_service<autoware_map_srvs::srv::LoadPCDPartiallyForPublish>(
        "load_pcd_partially/publish",
        std::bind(
          &PointCloudMapLoaderNode::loadPCDPartiallyForPublishServiceCallback, this,
          std::placeholders::_1, std::placeholders::_2));
  }

  if (enable_whole_load_) {
    pub_whole_pointcloud_map_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud_map/whole", durable_qos);

    for (const auto & p : pcd_paths_or_directory) {
      if (!fs::exists(p)) {
        RCLCPP_ERROR_STREAM(get_logger(), "invalid path: " << p);
      }

      if (isPcdFile(p)) {
        pcd_paths.push_back(p);
      }

      if (fs::is_directory(p)) {
        for (const auto & file : fs::directory_iterator(p)) {
          const auto filename = file.path().string();
          if (isPcdFile(filename)) {
            pcd_paths.push_back(filename);
          }
        }
      }
    }

    const auto pcd = loadPCDFiles(pcd_paths);

    if (pcd.width == 0) {
      RCLCPP_ERROR(get_logger(), "No PCD was loaded: pcd_paths.size() = %zu", pcd_paths.size());
      return;
    }

    sensor_msgs::msg::PointCloud2 whole_pcd;
    if (use_downsample_) {
      const auto downsampled_pcd = downsample(pcd, leaf_size_);
      pcl::toROSMsg(downsampled_pcd, whole_pcd);
    } else {
      pcl::toROSMsg(pcd, whole_pcd);
    }
    whole_pcd.header.frame_id = "map";
    pub_whole_pointcloud_map_->publish(whole_pcd);
  }
}

// TODO: pcl::PointCloud<PointType> -> sensor_msgs::msg::PointCloud2
pcl::PointCloud<PointType> PointCloudMapLoaderNode::loadPCDFiles(
  const std::vector<std::string> & pcd_paths)
{
  pcl::PointCloud<PointType> whole_pcd{};

  pcl::PointCloud<PointType> partial_pcd;
  for (const auto & path : pcd_paths) {
    if (pcl::io::loadPCDFile(path, partial_pcd) == -1) {
      RCLCPP_ERROR_STREAM(get_logger(), "PCD load failed: " << path);
    }

    if (whole_pcd.width == 0) {
      whole_pcd = partial_pcd;
    } else {
      // whole_pcd.width += partial_pcd.width;
      // whole_pcd.row_step += partial_pcd.row_step;
      // whole_pcd.data.reserve(whole_pcd.data.size() + partial_pcd.data.size());
      // whole_pcd.data.insert(whole_pcd.data.end(), partial_pcd.data.begin(), partial_pcd.data.end());
      whole_pcd += partial_pcd;
    }
  }

  whole_pcd.header.frame_id = "map";

  return whole_pcd;
}

std::vector<PointCloudMapLoaderNode::PCDFileMetadata> PointCloudMapLoaderNode::generatePCDMetadata(
  const std::vector<std::string> & pcd_paths) const
{
  std::vector<PCDFileMetadata> metadata_array;
  pcl::PointCloud<pcl::PointXYZ> partial_pcd;
  for (const auto & path : pcd_paths) {
    if (pcl::io::loadPCDFile(path, partial_pcd) == -1) {
      RCLCPP_ERROR_STREAM(get_logger(), "PCD load failed: " << path);
    }
    PointCloudMapLoaderNode::PCDFileMetadata metadata = {};
    pcl::getMinMax3D(partial_pcd, metadata.min, metadata.max);
    metadata.path = path;
    metadata_array.push_back(metadata);
  }
  return metadata_array;
}

sensor_msgs::msg::PointCloud2 PointCloudMapLoaderNode::loadPCDPartially(
  const geometry_msgs::msg::Point position, const float radius,
  std::vector<PointCloudMapLoaderNode::PCDFileMetadata> pcd_file_metadata_array) const
{
  sensor_msgs::msg::PointCloud2 filtered_pcd{};
  sensor_msgs::msg::PointCloud2 pcd;
  for (const auto & metadata : pcd_file_metadata_array) {
    if (sphere_and_box_overlap_exists(position, radius, metadata.min, metadata.max)) {
      if (pcl::io::loadPCDFile(metadata.path, pcd) == -1) {
        RCLCPP_ERROR_STREAM(get_logger(), "PCD load failed: " << metadata.path);
      }
      if (filtered_pcd.width == 0) {
        filtered_pcd = pcd;
      } else {
        filtered_pcd.width += pcd.width;
        filtered_pcd.row_step += pcd.row_step;
        filtered_pcd.data.reserve(filtered_pcd.data.size() + pcd.data.size());
        filtered_pcd.data.insert(filtered_pcd.data.end(), pcd.data.begin(), pcd.data.end());
      }
    }
  }
  filtered_pcd.header.frame_id = "map";
  return filtered_pcd;
}

bool PointCloudMapLoaderNode::loadPCDPartiallyForPublishServiceCallback(
  autoware_map_srvs::srv::LoadPCDPartiallyForPublish::Request::SharedPtr req,
  autoware_map_srvs::srv::LoadPCDPartiallyForPublish::Response::SharedPtr res)
{
  res->position = req->position;
  res->radius = req->radius;
  pub_partial_pointcloud_map_->publish(
    loadPCDPartially(req->position, req->radius, pcd_file_metadata_array_));
  return true;
}

bool PointCloudMapLoaderNode::loadPCDPartiallyServiceCallback(
  autoware_map_srvs::srv::LoadPCDPartially::Request::SharedPtr req,
  autoware_map_srvs::srv::LoadPCDPartially::Response::SharedPtr res)
{
  res->position = req->position;
  res->radius = req->radius;
  res->map = loadPCDPartially(req->position, req->radius, pcd_file_metadata_array_);
  return true;
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudMapLoaderNode)
