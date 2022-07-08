#pragma once
#include "common/synchro_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace imgproc
{
class SegmentFilter : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  SegmentFilter();

private:
  SyncroSubscriber<PointCloud2, PointCloud2> subscriber_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  std::optional<CameraInfo> info_{std::nullopt};

  void execute(const PointCloud2 & msg1, const PointCloud2 & msg2);
};
}  // namespace imgproc