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
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

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

sensor_msgs::msg::PointCloud2 downsample(sensor_msgs::msg::PointCloud2 msg_input, float leaf_size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(msg_input, *pcl_input);
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(pcl_input);
  // filter.setSaveLeafLayout(true);
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.filter(*pcl_output);

  sensor_msgs::msg::PointCloud2 msg_output;
  pcl::toROSMsg(*pcl_output, msg_output);
  msg_output.header = msg_input.header;
  return msg_output;
}

PointCloudMapLoaderNode::PointCloudMapLoaderNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_map_loader", options)
{
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  const auto pcd_paths_or_directory =
    declare_parameter("pcd_paths_or_directory", std::vector<std::string>({}));

  enable_whole_load_ = declare_parameter("enable_whole_load", true);
  enable_partial_load_ = declare_parameter("enable_partial_load", true);
  use_downsample_ = declare_parameter("use_downsample", true);
  leaf_size_ = declare_parameter("leaf_size", 3.0);
  map_frame_ = declare_parameter("map_frame", "map");
  viewer_frame_ = declare_parameter("viewer_frame", "viewer");

  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  std::vector<std::string> pcd_paths{};

  if (enable_whole_load_) {
    pub_whole_pointcloud_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "output/pointcloud_map/whole", durable_qos);
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
    sensor_msgs::msg::PointCloud2 pcd = loadPCDFiles(pcd_paths);

    if (pcd.width == 0) {
      RCLCPP_ERROR(get_logger(), "No PCD was loaded: pcd_paths.size() = %zu", pcd_paths.size());
      return;
    }

    pcd.header.frame_id = "map";
    generateTF(pcd);
    pub_whole_pointcloud_map_->publish(pcd);
  }

  if (enable_partial_load_) {
    pub_partial_pointcloud_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "output/pointcloud_map/partial", durable_qos);
    pcd_file_metadata_array_ = generatePCDMetadata(pcd_paths);
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
    pub_debug_loading_time_ms_ = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("/map/pointcloud_map_loader/loading_time_ms", 10);
  }
}

sensor_msgs::msg::PointCloud2 PointCloudMapLoaderNode::loadPCDFiles(
  const std::vector<std::string> & pcd_paths)
{
  sensor_msgs::msg::PointCloud2 whole_pcd{};

  sensor_msgs::msg::PointCloud2 partial_pcd;

  // for (const auto & path : pcd_paths) {
  for (int i = 0; i < int(pcd_paths.size()); ++i) {
    auto & path = pcd_paths[i];
    if (i % 50 == 0) {
      RCLCPP_INFO_STREAM(get_logger(),
        "Load " << path << " (" << i + 1 << " out of " << int(pcd_paths.size()) << ")");
    }

    if (pcl::io::loadPCDFile(path, partial_pcd) == -1) {
      RCLCPP_ERROR_STREAM(get_logger(), "PCD load failed: " << path);
    }

    if (use_downsample_) {
      partial_pcd = downsample(partial_pcd, leaf_size_);
    }

    if (whole_pcd.width == 0) {
      whole_pcd = partial_pcd;
    } else {
      whole_pcd.width += partial_pcd.width;
      whole_pcd.row_step += partial_pcd.row_step;
      // whole_pcd.data.reserve(whole_pcd.data.size() + partial_pcd.data.size());
      whole_pcd.data.insert(whole_pcd.data.end(), partial_pcd.data.begin(), partial_pcd.data.end());
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
  const auto loading_start_time = std::chrono::system_clock::now();

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
        // filtered_pcd.data.reserve(filtered_pcd.data.size() + pcd.data.size()); // ToDo (koji
        // minoda): erasing this line significantly improved the computation speed. WHY???
        filtered_pcd.data.insert(filtered_pcd.data.end(), pcd.data.begin(), pcd.data.end());
      }
    }
  }

  const auto loading_end_time = std::chrono::system_clock::now();
  const double loading_time =
    std::chrono::duration_cast<std::chrono::microseconds>(loading_end_time - loading_start_time)
      .count() /
    1000.0;
  tier4_debug_msgs::msg::Float32Stamped msg;
  msg.stamp = this->now();
  msg.data = loading_time;
  pub_debug_loading_time_ms_->publish(msg);
  RCLCPP_INFO_STREAM(
    get_logger(), "KOJI loadPCDPartially @map_loader: " << loading_time << " [ms], num points: "
                                                        << int(filtered_pcd.data.size()));

  filtered_pcd.header.frame_id = "map";
  return filtered_pcd;
}

bool PointCloudMapLoaderNode::continueToLoadMaps(
  const geometry_msgs::msg::Point position, const float radius,
  std::vector<PointCloudMapLoaderNode::PCDFileMetadata> pcd_file_metadata_array)
{
  std::set<std::string> pcd_maps_to_load;
  for (const auto & metadata : pcd_file_metadata_array) {
    if (sphere_and_box_overlap_exists(position, radius, metadata.min, metadata.max)) {
      pcd_maps_to_load.insert(metadata.path);
    }
  }

  if (int(pcd_maps_to_load.size()) == 0) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Empty pcd map! (Please check if you are using the right pcd maps and giving an appropriate initial pose.)");
    return false;
  }

  if (pcd_maps_to_load == previously_loaded_pcd_paths_) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "The same request received as before! Skip loading");
    return false;
  }
  return true;
}

bool PointCloudMapLoaderNode::loadPCDPartiallyForPublishServiceCallback(
  autoware_map_srvs::srv::LoadPCDPartiallyForPublish::Request::SharedPtr req,
  autoware_map_srvs::srv::LoadPCDPartiallyForPublish::Response::SharedPtr res)
{
  res->position = req->position;
  res->radius = req->radius;
  if (!continueToLoadMaps(req->position, req->radius, pcd_file_metadata_array_)) {
    return true;
  }
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
  if (!continueToLoadMaps(req->position, req->radius, pcd_file_metadata_array_)) {
    return true;
  }
  res->map = loadPCDPartially(req->position, req->radius, pcd_file_metadata_array_);
  return true;
}

void PointCloudMapLoaderNode::generateTF(sensor_msgs::msg::PointCloud2 & pcd_msg)
{
  // fix random seed to produce the same viewer position every time
  // 3939 is just the author's favorite number
  srand(3939);

  pcl::PointCloud<pcl::PointXYZ> pcd_pcl;
  pcl::fromROSMsg<pcl::PointXYZ>(pcd_msg, pcd_pcl);

  const std::vector<size_t> indices = UniformRandom(pcd_pcl.size(), N_SAMPLES);
  double coordinate[3] = {0, 0, 0};
  for (const auto i : indices) {
    coordinate[0] += pcd_pcl.points[i].x;
    coordinate[1] += pcd_pcl.points[i].y;
    coordinate[2] += pcd_pcl.points[i].z;
  }
  coordinate[0] = coordinate[0] / indices.size();
  coordinate[1] = coordinate[1] / indices.size();
  coordinate[2] = coordinate[2] / indices.size();

  geometry_msgs::msg::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = this->now();
  static_transformStamped.header.frame_id = map_frame_;
  static_transformStamped.child_frame_id = viewer_frame_;
  static_transformStamped.transform.translation.x = coordinate[0];
  static_transformStamped.transform.translation.y = coordinate[1];
  static_transformStamped.transform.translation.z = coordinate[2];
  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();

  static_broadcaster_->sendTransform(static_transformStamped);

  RCLCPP_INFO_STREAM(
    get_logger(), "broadcast static tf. map_frame:"
                    << map_frame_ << ", viewer_frame:" << viewer_frame_ << ", x:" << coordinate[0]
                    << ", y:" << coordinate[1] << ", z:" << coordinate[2]);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudMapLoaderNode)
