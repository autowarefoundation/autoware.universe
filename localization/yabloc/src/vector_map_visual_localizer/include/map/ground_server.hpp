#pragma once
#include "common/ground_plane.hpp"

#include <rclcpp/rclcpp.hpp>

#include "vmvl_msgs/srv/ground.hpp"
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace map
{
class GroundServer : public rclcpp::Node
{
public:
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using Ground = vmvl_msgs::srv::Ground;
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Float32 = std_msgs::msg::Float32;
  using Float32Array = std_msgs::msg::Float32MultiArray;
  using Marker = visualization_msgs::msg::Marker;

  GroundServer();

private:
  rclcpp::Service<Ground>::SharedPtr service_;

  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_stamped_;
  rclcpp::Publisher<Float32>::SharedPtr pub_ground_height_;
  rclcpp::Publisher<Float32Array>::SharedPtr pub_ground_plane_;
  rclcpp::Publisher<Marker>::SharedPtr pub_marker_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{nullptr};
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_{nullptr};

  void callbackMap(const HADMapBin & msg);
  void callbackPoseStamped(const PoseStamped & msg);

  common::GroundPlane computeGround(const geometry_msgs::msg::Point & point);
  void publishMarker(const common::GroundPlane & plane);

  void callbackService(
    const std::shared_ptr<Ground::Request> request, std::shared_ptr<Ground::Response> response);
};

}  // namespace map