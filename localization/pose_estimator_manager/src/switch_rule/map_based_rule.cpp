// Copyright 2023 Autoware Foundation
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

#include "pose_estimator_manager/switch_rule/map_based_rule.hpp"

#include "pose_estimator_manager/rule_helper/grid_info.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace multi_pose_estimator
{
std::vector<PoseEstimatorName> MapBasedRule::supporting_pose_estimators()
{
  return {
    PoseEstimatorName::ndt, PoseEstimatorName::yabloc, PoseEstimatorName::eagleye,
    PoseEstimatorName::artag};
}

MapBasedRule::MapBasedRule(
  rclcpp::Node & node, const std::unordered_set<PoseEstimatorName> & running_estimator_list)
: BaseSwitchRule(node),
  pcd_density_upper_threshold_(
    node.declare_parameter<int>("pcd_occupancy_rule/pcd_density_upper_threshold")),
  pcd_density_lower_threshold_(
    node.declare_parameter<int>("pcd_occupancy_rule/pcd_density_lower_threshold")),
  ar_marker_available_distance_(
    node.declare_parameter<int>("ar_marker_rule/ar_marker_available_distance")),
  running_estimator_list_(running_estimator_list)
{
  RCLCPP_INFO_STREAM(get_logger(), "MapBasedRule is initialized");

  using std::placeholders::_1;
  const auto latch_qos = rclcpp::QoS(1).transient_local().reliable();

  auto on_vector_map = std::bind(&MapBasedRule::on_vector_map, this, _1);
  auto on_point_cloud_map = std::bind(&MapBasedRule::on_point_cloud_map, this, _1);
  auto on_pose_cov = std::bind(&MapBasedRule::on_pose_cov, this, _1);
  auto on_initialization_state = [this](InitializationState::ConstSharedPtr msg) -> void {
    initialization_state_ = *msg;
  };
  auto on_eagleye_fix = [this](NavSatFix::ConstSharedPtr) -> void {
    eagleye_is_initialized = true;
  };

  sub_vector_map_ =
    node.create_subscription<HADMapBin>("~/input/vector_map", latch_qos, on_vector_map);
  sub_point_cloud_map_ =
    node.create_subscription<PointCloud2>("~/input/pointcloud_map", latch_qos, on_point_cloud_map);
  sub_pose_cov_ =
    node.create_subscription<PoseCovStamped>("~/input/pose_with_covariance", 10, on_pose_cov);
  sub_initialization_state_ = node.create_subscription<InitializationState>(
    "~/input/initialization_state", latch_qos, on_initialization_state);
  sub_eagleye_fix_ = node.create_subscription<NavSatFix>("~/input/eagleye/fix", 10, on_eagleye_fix);

  if (running_estimator_list.count(PoseEstimatorName::artag)) {
    ar_tag_position_ = std::make_unique<ArTagPosition>(&node);
  }

  //
  initialization_state_.state = InitializationState::UNINITIALIZED;
}

bool MapBasedRule::eagleye_is_available() const
{
  if (running_estimator_list_.count(PoseEstimatorName::eagleye) == 0) {
    return false;
  }

  if (!eagleye_is_initialized) {
    return false;
  }

  if (!latest_pose_.has_value()) {
    return false;
  }

  return eagleye_area_.within(latest_pose_->pose.pose.position);
}
bool MapBasedRule::yabloc_is_available() const
{
  return running_estimator_list_.count(PoseEstimatorName::yabloc) != 0;
}

bool MapBasedRule::ndt_is_available() const
{
  return running_estimator_list_.count(PoseEstimatorName::ndt) != 0;
}

bool MapBasedRule::ndt_is_more_suitable_than_yabloc(std::string * optional_message) const
{
  if (!kdtree_) {
    if (optional_message) {
      *optional_message = "enable YabLoc\npcd is not subscribed yet";
    }
    return false;
  }

  const auto position = latest_pose_->pose.pose.position;
  const pcl::PointXYZ query{
    static_cast<float>(position.x), static_cast<float>(position.y), static_cast<float>(position.z)};
  std::vector<int> indices;
  std::vector<float> distances;
  const int count = kdtree_->radiusSearch(query, 50, indices, distances, 0);

  static bool last_is_ndt_mode = true;
  const bool is_ndt_mode = (last_is_ndt_mode) ? (count > pcd_density_lower_threshold_)
                                              : (count > pcd_density_upper_threshold_);
  last_is_ndt_mode = is_ndt_mode;

  std::stringstream ss;
  if (is_ndt_mode) {
    ss << "enable NDT";
  } else {
    ss << "enable YabLoc";
  }
  ss << "\npcd occupancy: " << count << " > "
     << (last_is_ndt_mode ? pcd_density_lower_threshold_ : pcd_density_upper_threshold_);
  *optional_message = ss.str();

  return true;
}

std::string MapBasedRule::debug_string()
{
  return debug_string_msg_;
}

MapBasedRule::MarkerArray MapBasedRule::debug_marker_array()
{
  MarkerArray array_msg;
  visualization_msgs::msg::Marker msg;

  msg.ns = "pcd_occupancy";
  msg.id = 0;
  msg.header.frame_id = "map";
  msg.scale.set__x(3.0f).set__y(3.0f).set__z(3.f);
  msg.color.set__r(1.0f).set__g(1.0f).set__b(0.2f).set__a(1.0f);

  if (occupied_areas_) {
    for (auto p : occupied_areas_->points) {
      geometry_msgs::msg::Point geometry_point{};
      geometry_point.set__x(p.x).set__y(p.y).set__z(p.z);
      msg.points.push_back(geometry_point);
    }
  }
  array_msg.markers.push_back(msg);

  //
  for (const auto & marker : eagleye_area_.debug_marker_array().markers) {
    array_msg.markers.push_back(marker);
  }

  return array_msg;
}

void MapBasedRule::on_vector_map(HADMapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "subscribed binary vector map");
  eagleye_area_.init(msg);
}

void MapBasedRule::on_point_cloud_map(PointCloud2::ConstSharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  std::unordered_map<GridInfo, size_t> grid_point_count;
  for (pcl::PointXYZ xyz : cloud) {
    grid_point_count[GridInfo(xyz.x, xyz.y)] += 1;
  }

  occupied_areas_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto [grid, count] : grid_point_count) {
    if (count > 50) {
      occupied_areas_->push_back(grid.get_center_point());
    }
  }

  kdtree_ = pcl::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  kdtree_->setInputCloud(occupied_areas_);

  RCLCPP_INFO_STREAM(
    get_logger(), "pose_estiamtor_manager has loaded PCD map " << occupied_areas_->size());
}

void MapBasedRule::on_pose_cov(PoseCovStamped::ConstSharedPtr msg)
{
  latest_pose_ = *msg;
}

bool MapBasedRule::artag_is_available() const
{
  if (running_estimator_list_.count(PoseEstimatorName::artag) == 0) {
    return false;
  }

  if (!latest_pose_.has_value()) {
    return false;
  }

  const double distance_to_marker =
    ar_tag_position_->distance_to_nearest_ar_tag_around_ego(latest_pose_->pose.pose.position);
  return distance_to_marker < ar_marker_available_distance_;
}

}  // namespace multi_pose_estimator
