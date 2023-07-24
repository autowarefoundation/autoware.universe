#pragma once
#include "pose_estimator_manager/base_pose_estimator_sub_manager.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace multi_pose_estimator
{
class SubManagerNdt : public BasePoseEstimatorSubManager
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  SubManagerNdt(rclcpp::Node * node) : BasePoseEstimatorSubManager(node)
  {
    using std::placeholders::_1;
    auto on_pointcloud = std::bind(&SubManagerNdt::on_pointcloud, this, _1);

    sub_pointcloud_ = node->create_subscription<PointCloud2>(
      "~/input/pointcloud", rclcpp::SensorDataQoS(), on_pointcloud);
    pub_pointcloud_ = node->create_publisher<PointCloud2>(
      "~/output/pointcloud", rclcpp::SensorDataQoS().keep_last(10));

    ndt_is_enabled_ = true;
  }

  void set_enable(bool enabled) override { ndt_is_enabled_ = enabled; }

private:
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_;
  bool ndt_is_enabled_;

  void on_pointcloud(PointCloud2::ConstSharedPtr msg)
  {
    if (ndt_is_enabled_) pub_pointcloud_->publish(*msg);
  }
};
}  // namespace multi_pose_estimator