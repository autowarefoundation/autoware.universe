#pragma once

#include "pose_estimator_manager/pose_estimator_name.hpp"
#include "pose_estimator_manager/switch_rule/base_switch_rule.hpp"
#include "pose_estimator_manager/switch_rule/eagleye_area.hpp"

#include <rclcpp/logger.hpp>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace multi_pose_estimator
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
  std::vector<PoseEstimatorName> supporting_pose_estimators() override;

  std::string debug_string() override;
  MarkerArray debug_marker_array() override;

protected:
  const int pcd_density_threshold_;
  const std::unordered_set<PoseEstimatorName> running_estimator_list_;

  std::string debug_string_msg_;
  InitializationState initialization_state_;
  std::optional<PoseCovStamped> latest_pose_{std::nullopt};
  bool eagleye_is_initialized{false};

  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_pose_cov_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_point_cloud_map_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Subscription<InitializationState>::SharedPtr sub_initialization_state_;
  rclcpp::Subscription<NavSatFix>::SharedPtr sub_eagleye_fix_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr occupied_areas_{nullptr};
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_{nullptr};

  EagleyeArea eagleye_area_;

  bool eagleye_is_available() const;
  bool yabloc_is_available() const;
  bool ndt_is_available() const;

  void on_point_cloud_map(PointCloud2::ConstSharedPtr msg);
  void on_vector_map(HADMapBin::ConstSharedPtr msg);
  void on_pose_cov(PoseCovStamped::ConstSharedPtr msg);
};
}  // namespace multi_pose_estimator