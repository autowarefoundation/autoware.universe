#ifndef LIDAR_PARTICLE_CORRECTOR__LIDAR_PARTICLE_CORRECTOR_HPP_
#define LIDAR_PARTICLE_CORRECTOR__LIDAR_PARTICLE_CORRECTOR_HPP_

#include <modularized_particle_filter/correction/abst_corrector.hpp>
#include <pcl_ros/impl/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <optional>
#include <unordered_map>
#include <vector>

namespace modularized_particle_filter
{
class LidarPoseCorrector : public AbstCorrector
{
public:
  LidarPoseCorrector();

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;

  rclcpp::Time prev_time_;

  float map_resolution_;

  nav_msgs::msg::MapMetaData likelihood_map_meta_data_;
  std::optional<std::vector<std::pair<int, std::vector<std::pair<int, int>>>>>
    likelihood_map_as_jagged_array_opt_;
  nav_msgs::msg::OccupancyGrid occupancy_grid_;

  void mapPointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg);
  void scanPointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_pointcloud_msg);
};
}  // namespace modularized_particle_filter

#endif  // LIDAR_PARTICLE_CORRECTOR__LIDAR_PARTICLE_CORRECTOR_HPP_
