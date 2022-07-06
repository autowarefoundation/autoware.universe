#pragma once
#include "common/synchro_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace imgproc
{
class SegmentFilter : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  SegmentFilter();

private:
  SyncroSubscriber<PointCloud2, PointCloud2> subscriber_;
  // SyncroSubscriber subscriber_;

  void execute(const PointCloud2 & msg1, const PointCloud2 & msg2);
};
}  // namespace imgproc