/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include<memory>

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/dynamic_object_array.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "obstacle_stop_planner/adaptive_cruise_control.hpp"
#include "obstacle_stop_planner/debug_marker.hpp"

namespace motion_planning
{
class ObstacleStopPlannerNode : public rclcpp::Node
{
public:
  ObstacleStopPlannerNode();

private:
  /*
   * ROS
   */
  // publisher and subscriber
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr current_velocity_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr dynamic_object_sub_;

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr stop_reason_diag_pub_;

  std::shared_ptr<ObstacleStopPlannerDebugNode> debug_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /*
   * Parameter
   */
  std::unique_ptr<motion_planning::AdaptiveCruiseController> acc_controller_;
  sensor_msgs::msg::PointCloud2::Ptr obstacle_ros_pointcloud_ptr_;
  geometry_msgs::msg::TwistStamped::ConstPtr current_velocity_ptr_;
  autoware_perception_msgs::msg::DynamicObjectArray::ConstPtr object_ptr_;
  double wheel_base_, front_overhang_, rear_overhang_, left_overhang_, right_overhang_,
    vehicle_width_, vehicle_length_;
  double stop_margin_;
  double slow_down_margin_;
  double min_behavior_stop_margin_;
  rclcpp::Time prev_col_point_time_;
  pcl::PointXYZ prev_col_point_;
  double expand_slow_down_range_;
  double max_slow_down_vel_;
  double min_slow_down_vel_;
  double max_deceleration_;
  bool enable_slow_down_;
  double step_length_;
  double stop_search_radius_;
  double slow_down_search_radius_;
  void obstaclePointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
  void pathCallback(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr input_msg);
  void dynamicObjectCallback(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg);
  void currentVelocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr input_msg);

private:
  bool convexHull(
    const std::vector<cv::Point2d> pointcloud, std::vector<cv::Point2d> & polygon_points);
  bool decimateTrajectory(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double step_length,
    autoware_planning_msgs::msg::Trajectory & output_trajectory);
  bool decimateTrajectory(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double step_length,
    autoware_planning_msgs::msg::Trajectory & output_trajectory,
    std::map<size_t /* decimate */, size_t /* origin */> & index_map);
  bool trimTrajectoryFromSelfPose(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory,
    const geometry_msgs::msg::Pose self_pose, autoware_planning_msgs::msg::Trajectory & output_trajectory);
  bool trimTrajectoryWithIndexFromSelfPose(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory,
    const geometry_msgs::msg::Pose self_pose, autoware_planning_msgs::msg::Trajectory & output_trajectory,
    size_t & index);
  bool searchPointcloudNearTrajectory(
    const autoware_planning_msgs::msg::Trajectory & trajectory, const double radius,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_pointcloud_ptr);
  void createOneStepPolygon(
    const geometry_msgs::msg::Pose base_stap_pose, const geometry_msgs::msg::Pose next_step_pose,
    std::vector<cv::Point2d> & polygon, const double expand_width = 0.0);
  bool getSelfPose(
    const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer,
    geometry_msgs::msg::Pose & self_pose);
  bool getBackwordPointFromBasePoint(
    const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
    const Eigen::Vector2d & base_point, const double backward_length,
    Eigen::Vector2d & output_point);
  void getNearestPoint(
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
    pcl::PointXYZ * nearest_collision_point, rclcpp::Time * nearest_collision_point_time);
  void getLateralNearestPoint(
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
    pcl::PointXYZ * lateral_nearest_point, double * deviation);
  geometry_msgs::msg::Pose getVehicleCenterFromBase(const geometry_msgs::msg::Pose & base_pose);
};
}  // namespace motion_planning
