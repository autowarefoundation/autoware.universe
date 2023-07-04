#pragma once

#include <pose_estimator_manager/plugin_interface.hpp>
#include <pose_estimator_manager/pose_estimator_name.hpp>
#include <rclcpp/logger.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace multi_pose_estimator
{

class PcdOccupancyRule : public PluginInterface
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  void init(rclcpp::Node & node) override;
  std::unordered_map<PoseEstimatorName, bool> update() override;
  std::vector<PoseEstimatorName> supporting_pose_estimators() override;

  std::string debug_string() override;
  MarkerArray debug_marker_array() override;

protected:
  int pcd_density_threshold_;
  std::string debug_string_msg_;
  std::optional<PoseCovStamped> latest_pose_{std::nullopt};

  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_pose_cov_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_map_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr occupied_areas_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_{nullptr};

  void on_map(PointCloud2::ConstSharedPtr msg);
  void on_pose_cov(PoseCovStamped::ConstSharedPtr msg);
};
}  // namespace multi_pose_estimator