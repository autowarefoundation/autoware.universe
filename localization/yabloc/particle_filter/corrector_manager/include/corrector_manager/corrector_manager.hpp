#pragma once
#include "corrector_manager/init_area.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pcdless
{
class CorrectorManager : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  CorrectorManager();

private:
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_init_area_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_gnss_pose_;

  std::optional<InitArea> init_area_{std::nullopt};

  void on_init_area(const PointCloud2 & msg);
  void on_gnss_pose(const PoseStamped & msg);
};
}  // namespace pcdless