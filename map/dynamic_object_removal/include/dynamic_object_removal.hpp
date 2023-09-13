#ifndef DYNAMIC_OBJECT_REMOVAL_HPP
#define DYNAMIC_OBJECT_REMOVAL_HPP

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "geometry_msgs/msg/pose.hpp"

// Message filters for synchronizing topics
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// PCL headers for point cloud manipulation
#include "pcl/filters/crop_box.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

// Eigen for linear algebra
#include <Eigen/Geometry>

using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;

class DynamicObjectRemoval : public rclcpp::Node
{
public:
  DynamicObjectRemoval();

  // Publisher for the processed point cloud
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;

  // Callback for synchronized messages
  void callback(const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& pcl_msg,
                const std::shared_ptr<const autoware_auto_perception_msgs::msg::DetectedObjects>& obj_msg);

  // Function to remove objects using CropBox
  void objectRemoveCropBox(PointCloudXYZI::Ptr crop_cloud, const Eigen::Vector4f min_point,
                           const Eigen::Vector4f max_point, const Eigen::Vector3f translation, double orientation_yaw);

private:
  // Subscribers for point cloud and detected objects
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcl_sub_;
  message_filters::Subscriber<autoware_auto_perception_msgs::msg::DetectedObjects> obj_sub_;

  // Synchronizer for the above subscribers
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, autoware_auto_perception_msgs::msg::DetectedObjects>>>
      sync_;
};

#endif  // DYNAMIC_OBJECT_REMOVAL_HPP