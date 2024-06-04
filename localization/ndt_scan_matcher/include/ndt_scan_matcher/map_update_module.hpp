// Copyright 2022 Autoware Foundation
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

#ifndef NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
#define NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_

#include "localization_util/util_func.hpp"
#include "ndt_scan_matcher/diagnostics_module.hpp"
#include "ndt_scan_matcher/hyper_parameters.hpp"
#include "ndt_scan_matcher/particle.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <tier4_autoware_utils/transform/transforms.hpp>

#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <fmt/format.h>
#include <multigrid_pclomp/multigrid_ndt_omp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <ndt_scan_matcher/map_loader.hpp>

class MapUpdateModule
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using TargetCloudType = pcl::PointCloud<PointTarget>;
  using SourceCloudType = pcl::PointCloud<PointSource>;
  using TargetCloudPtr = typename TargetCloudType::Ptr;
  using SourceCloudPtr = typename SourceCloudType::Ptr;
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;
  using NdtPtrType = std::shared_ptr<NdtType>;

public:
  MapUpdateModule(
    rclcpp::Node * node, std::mutex * ndt_ptr_mutex, NdtPtrType & ndt_ptr,
    HyperParameters::DynamicMapLoading param);

private:
  friend class NDTScanMatcher;

  void callback_timer(
    const bool is_activated, const std::optional<geometry_msgs::msg::Point> & position,
    std::unique_ptr<DiagnosticsModule> & diagnostics_ptr);

  void meta_callback(const std_msgs::msg::String::SharedPtr msg);

  [[nodiscard]] bool should_update_map(
    const geometry_msgs::msg::Point & position,
    std::unique_ptr<DiagnosticsModule> & diagnostics_ptr);
  void update_map(
    const geometry_msgs::msg::Point & position,
    std::unique_ptr<DiagnosticsModule> & diagnostics_ptr);
  // Update the specified NDT
  bool update_ndt(
    const geometry_msgs::msg::Point & position, NdtType & ndt,
    std::unique_ptr<DiagnosticsModule> & diagnostics_ptr);

  bool update_ndt_old(
    const geometry_msgs::msg::Point & position, NdtType & ndt,
    std::unique_ptr<DiagnosticsModule> & diagnostics_ptr);
  void publish_partial_pcd_map();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loaded_pcd_pub_;

  rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedPtr
    pcd_loader_client_;

  NdtPtrType & ndt_ptr_;
  std::mutex * ndt_ptr_mutex_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::optional<geometry_msgs::msg::Point> last_update_position_ = std::nullopt;

  HyperParameters::DynamicMapLoading param_;

  // Indicate if there is a prefetch thread waiting for being collected
  NdtPtrType secondary_ndt_ptr_;
  bool need_rebuild_;

  // Direct map loader
  loc::MapLoader<PointTarget> map_loader_;
  std::string metadata_path_, pcd_dir_path_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr meta_subscription_;
};

#endif  // NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
