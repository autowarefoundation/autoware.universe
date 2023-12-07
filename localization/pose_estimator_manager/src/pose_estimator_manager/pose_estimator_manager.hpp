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

#ifndef POSE_ESTIMATOR_MANAGER__POSE_ESTIMATOR_MANAGER_HPP_
#define POSE_ESTIMATOR_MANAGER__POSE_ESTIMATOR_MANAGER_HPP_

#include "pose_estimator_manager/base_pose_estimator_sub_manager.hpp"
#include "pose_estimator_manager/shared_data.hpp"
#include "pose_estimator_manager/switch_rule/base_switch_rule.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/logger_level_configure.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace pose_estimator_manager
{
class PoseEstimatorManager : public rclcpp::Node
{
public:
  using SetBool = std_srvs::srv::SetBool;
  using String = std_msgs::msg::String;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using InitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
  PoseEstimatorManager();

private:
  const std::unordered_set<PoseEstimatorName> running_estimator_list_;
  const std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;

  std::shared_ptr<SharedData> shared_data_{nullptr};

  // Timer callback
  rclcpp::TimerBase::SharedPtr timer_;
  // Publishers
  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_array_;
  rclcpp::Publisher<String>::SharedPtr pub_debug_string_;
  // For sub manager subscriber
  rclcpp::Subscription<Image>::SharedPtr sub_yabloc_input_;
  rclcpp::Subscription<Image>::SharedPtr sub_artag_input_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ndt_input_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_eagleye_output_;
  // For switch rule subscriber
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_localization_pose_cov_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_point_cloud_map_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Subscription<InitializationState>::SharedPtr sub_initialization_state_;

  std::unordered_map<PoseEstimatorName, BasePoseEstimatorSubManager::SharedPtr> sub_managers_;

  std::shared_ptr<switch_rule::BaseSwitchRule> switch_rule_{nullptr};

  void toggle_all(bool enabled);
  void toggle_each(const std::unordered_map<PoseEstimatorName, bool> & toggle_list);

  void load_switch_rule();

  void publish_diagnostics() const;

  // Timer callback
  void on_timer();
  // For sub manager
  void on_yabloc_input(Image::ConstSharedPtr msg);
  void on_artag_input(Image::ConstSharedPtr msg);
  void on_ndt_input(PointCloud2::ConstSharedPtr msg);
  void on_eagleye_output(PoseCovStamped::ConstSharedPtr msg);
};
}  // namespace pose_estimator_manager

#endif  // POSE_ESTIMATOR_MANAGER__POSE_ESTIMATOR_MANAGER_HPP_
