#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__LIDAR_POSE_CORRECTOR_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__LIDAR_POSE_CORRECTOR_HPP_

#include <optional>
#include <vector>
#include <unordered_map>

#include <boost/circular_buffer.hpp>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>

#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"


class LidarPoseCorrector : public rclcpp::Node
{
public:
  LidarPoseCorrector();

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<modularized_particle_filter_msgs::msg::ParticleArray>::SharedPtr
    particle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;

  rclcpp::Publisher<modularized_particle_filter_msgs::msg::ParticleArray>::SharedPtr
    weighted_particle_pub_;

  rclcpp::Time prev_time_;

  float map_resolution_;
  int particles_buffer_size_;
  boost::circular_buffer<modularized_particle_filter_msgs::msg::ParticleArray>
  particles_circular_buffer_;

  nav_msgs::msg::MapMetaData likelihood_map_meta_data_;
  std::optional<std::vector<std::pair<int, std::vector<std::pair<int, int>>>>>
  likelihood_map_as_jagged_array_opt_;
  nav_msgs::msg::OccupancyGrid occupancy_grid_;

  void mapPointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg);
  void particleCallback(
    const modularized_particle_filter_msgs::msg::ParticleArray::ConstSharedPtr particles);
  void scanPointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_pointcloud_msg);
};

#endif // MODULARIZED_PARTICLE_FILTER__CORRECTION__LIDAR_POSE_CORRECTOR_HPP_
