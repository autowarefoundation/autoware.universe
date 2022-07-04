#pragma once
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace map
{
class Ll2Decomposer : public rclcpp::Node
{
public:
  using Cloud2 = sensor_msgs::msg::PointCloud2;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  Ll2Decomposer();

private:
  pcl::PointCloud<pcl::PointNormal>::Ptr linestrings_ = nullptr;

  rclcpp::Publisher<Cloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<Cloud2>::SharedPtr pub_sign_board_;

  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;

  void publishSignBoard(const lanelet::LineStringLayer & line_strings, const rclcpp::Time & stamp);

  void mapCallback(const HADMapBin & msg);
};
}  // namespace map
