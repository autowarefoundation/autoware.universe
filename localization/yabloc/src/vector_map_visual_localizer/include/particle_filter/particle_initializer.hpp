#pragma once
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace particle_filter
{
class ParticleInitializer : public rclcpp::Node
{
public:
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using Marker = visualization_msgs::msg::Marker;

  ParticleInitializer();

private:
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_initialpose_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Publisher<ParticleArray>::SharedPtr pub_particle_;
  rclcpp::Publisher<Marker>::SharedPtr pub_marker_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{nullptr};
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_{nullptr};

  void onInitialpose(const PoseCovStamped & initialpose);
  void onMap(const HADMapBin & bin_map);

  void searchLandingLanelet(const Eigen::Vector3f & pos);
  void publishRangeMarker(const Eigen::Vector3f & pos);
};
}  // namespace particle_filter