#pragma once
#include <eigen3/Eigen/Geometry>
#include <rclcpp/publisher.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace vml_common
{
rclcpp::Time ubloxTime2Stamp(const ublox_msgs::msg::NavPVT & msg);
ublox_msgs::msg::NavPVT stamp2UbloxTime(const builtin_interfaces::msg::Time & stamp);

Eigen::Affine3f pose2Affine(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Pose affine2Pose(const Eigen::Affine3f & affine);
}  // namespace vml_common