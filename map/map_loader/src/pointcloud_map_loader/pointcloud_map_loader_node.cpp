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

#include "pointcloud_map_loader_node.hpp"

#include <glob.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <memory>
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

PointCloudMapLoaderNode::PointCloudMapLoaderNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_map_loader", options)
{
  const auto pcd_paths =
    getPcdPaths(declare_parameter<std::vector<std::string>>("pcd_paths_or_directory"));
  std::string pcd_metadata_path = declare_parameter<std::string>("pcd_metadata_path");
  bool enable_whole_load = declare_parameter<bool>("enable_whole_load");
  bool enable_downsample_whole_load = declare_parameter<bool>("enable_downsampled_whole_load");
  bool enable_partial_load = declare_parameter<bool>("enable_partial_load");
  bool enable_differential_load = declare_parameter<bool>("enable_differential_load");

  if (enable_whole_load) {
    std::string publisher_name = "output/pointcloud_map";
    pcd_map_loader_ =
      std::make_unique<PointcloudMapLoaderModule>(this, pcd_paths, publisher_name, false);
  }

  if (enable_downsample_whole_load) {
    std::string publisher_name = "output/debug/downsampled_pointcloud_map";
    downsampled_pcd_map_loader_ =
      std::make_unique<PointcloudMapLoaderModule>(this, pcd_paths, publisher_name, true);
  }

  if (enable_partial_load | enable_differential_load) {
    if (!fs::exists(pcd_metadata_path)) {
      RCLCPP_ERROR_STREAM(get_logger(), "PCD metadata file not found: " << pcd_metadata_path);
      return;
    }

    pcd_metadata_dict_ = loadPCDMetadata(pcd_metadata_path);
    pcd_metadata_dict_ = replaceWithAbsolutePath(pcd_metadata_dict_, pcd_paths);
    RCLCPP_INFO_STREAM(get_logger(), "Loaded PCD metadata: " << pcd_metadata_path);

  }

  if (enable_partial_load) {
    partial_map_loader_ = std::make_unique<PartialMapLoaderModule>(this, pcd_metadata_dict_);
  }

  if (enable_differential_load) {
    differential_map_loader_ =
      std::make_unique<DifferentialMapLoaderModule>(this, pcd_metadata_dict_);
  }
}

std::vector<std::string> PointCloudMapLoaderNode::getPcdPaths(
  const std::vector<std::string> & pcd_paths_or_directory) const
{
  std::vector<std::string> pcd_paths;
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
  return pcd_paths;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudMapLoaderNode)
