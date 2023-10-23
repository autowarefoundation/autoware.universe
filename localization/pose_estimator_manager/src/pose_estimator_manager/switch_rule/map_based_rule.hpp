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

#ifndef POSE_ESTIMATOR_MANAGER__SWITCH_RULE__MAP_BASED_RULE_HPP_
#define POSE_ESTIMATOR_MANAGER__SWITCH_RULE__MAP_BASED_RULE_HPP_

#include "pose_estimator_manager/pose_estimator_name.hpp"
#include "pose_estimator_manager/rule_helper/ar_tag_position.hpp"
#include "pose_estimator_manager/rule_helper/eagleye_area.hpp"
#include "pose_estimator_manager/rule_helper/pcd_occupancy.hpp"
#include "pose_estimator_manager/switch_rule/base_switch_rule.hpp"

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pose_estimator_manager::switch_rule
{
class MapBasedRule : public BaseSwitchRule
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using InitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using NavSatFix = sensor_msgs::msg::NavSatFix;

  MapBasedRule(
    rclcpp::Node & node, const std::unordered_set<PoseEstimatorName> & running_estimator_list);

  std::unordered_map<PoseEstimatorName, bool> update() override;

  std::string debug_string() override;

  MarkerArray debug_marker_array() override;

protected:
  const double ar_marker_available_distance_;
  const std::unordered_set<PoseEstimatorName> running_estimator_list_;

  std::unique_ptr<rule_helper::ArTagPosition> ar_tag_position_{nullptr};
  std::unique_ptr<rule_helper::PcdOccupancy> pcd_occupancy_{nullptr};
  std::unique_ptr<rule_helper::EagleyeArea> eagleye_area_{nullptr};

  std::string debug_string_msg_;
  InitializationState initialization_state_;
  std::optional<PoseCovStamped> latest_pose_{std::nullopt};
  bool eagleye_is_initialized{false};

  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_pose_cov_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_point_cloud_map_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Subscription<InitializationState>::SharedPtr sub_initialization_state_;
  rclcpp::Subscription<NavSatFix>::SharedPtr sub_eagleye_fix_;

  bool eagleye_is_available() const;
  bool yabloc_is_available() const;
  bool ndt_is_available() const;
  bool artag_is_available() const;
  bool ndt_is_more_suitable_than_yabloc(std::string * optional_message = nullptr) const;

  void on_point_cloud_map(PointCloud2::ConstSharedPtr msg);
  void on_vector_map(HADMapBin::ConstSharedPtr msg);
  void on_pose_cov(PoseCovStamped::ConstSharedPtr msg);
};
}  // namespace pose_estimator_manager::switch_rule

#endif  // POSE_ESTIMATOR_MANAGER__SWITCH_RULE__MAP_BASED_RULE_HPP_
