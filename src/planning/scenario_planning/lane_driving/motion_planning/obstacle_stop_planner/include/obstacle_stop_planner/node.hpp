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

#include <autoware_planning_msgs/Trajectory.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "obstacle_stop_planner/debug_marker.hpp"

namespace motion_planning
{
class ObstacleStopPlannerNode
{
public:
  ObstacleStopPlannerNode();

private:
  /*
   * ROS
   */
  // publisher and subscriber
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber path_sub_;
  ros::Subscriber obstacle_pointcloud_sub_;
  ros::Publisher path_pub_;
  std::shared_ptr<ObstacleStopPlannerDebugNode> debug_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /*
   * Parameter
   */
  sensor_msgs::PointCloud2::Ptr obstacle_ros_pointcloud_ptr_;
  double wheel_base_, front_overhang_, rear_overhang_, left_overhang_, right_overhang_,
    vehicle_width_;
  double stop_margin_;
  double min_behavior_stop_margin_;
  void obstaclePointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr & input_msg);
  void pathCallback(const autoware_planning_msgs::Trajectory::ConstPtr & input_msg);

private:
  bool convexHull(
    const std::vector<cv::Point2d> pointcloud, std::vector<cv::Point2d> & polygon_points);
  bool decimateTrajectory(
    const autoware_planning_msgs::Trajectory & input_trajectory, const double step_length,
    autoware_planning_msgs::Trajectory & output_trajectory);
  bool decimateTrajectory(
    const autoware_planning_msgs::Trajectory & input_trajectory, const double step_length,
    autoware_planning_msgs::Trajectory & output_trajectory,
    std::map<size_t /* decimate */, size_t /* origin */> & index_map);
  bool trimTrajectoryFromSelfPose(
    const autoware_planning_msgs::Trajectory & input_trajectory,
    const geometry_msgs::Pose self_pose, autoware_planning_msgs::Trajectory & output_trajectory);
  bool trimTrajectoryWithIndexFromSelfPose(
    const autoware_planning_msgs::Trajectory & input_trajectory,
    const geometry_msgs::Pose self_pose, autoware_planning_msgs::Trajectory & output_trajectory,
    size_t & index);
  bool searchPointcloudNearTrajectory(
    const autoware_planning_msgs::Trajectory & trajectory, const double radius,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_pointcloud_ptr);
  void createOneStepPolygon(
    const geometry_msgs::Pose base_stap_pose, const geometry_msgs::Pose next_step_pose,
    std::vector<cv::Point2d> & polygon);
  bool getSelfPose(
    const std_msgs::Header & header, const tf2_ros::Buffer & tf_buffer,
    geometry_msgs::Pose & self_pose);
  bool getBackwordPointFromBasePoint(
    const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
    const Eigen::Vector2d & base_point, const double backward_length,
    Eigen::Vector2d & output_point);
  void getNearestPoint(
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::Pose & base_pose,
    pcl::PointXYZ & nearest_collision_point);
};
}  // namespace motion_planning
