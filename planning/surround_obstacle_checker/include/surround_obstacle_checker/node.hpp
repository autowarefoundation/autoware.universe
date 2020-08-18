/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include <memory>
#include <string>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TwistStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/Trajectory.h>

#include "surround_obstacle_checker/debug_marker.hpp"

using Point2d = boost::geometry::model::d2::point_xy<double>;
using Polygon2d =
  boost::geometry::model::polygon<Point2d, false, false>;  // counter-clockwise, open

enum class State { PASS, STOP };

class SurroundObstacleCheckerNode
{
public:
  SurroundObstacleCheckerNode();

private:
  void pathCallback(const autoware_planning_msgs::Trajectory::ConstPtr & input_msg);
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & input_msg);
  void dynamicObjectCallback(
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr & input_msg);
  void currentVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr & input_msg);
  void insertStopVelocity(const size_t closest_idx, autoware_planning_msgs::Trajectory * traj);
  bool convertPose(
    const geometry_msgs::Pose & pose, const std::string & source, const std::string & target,
    const ros::Time & time, geometry_msgs::Pose & conv_pose);
  bool getPose(
    const std::string & source, const std::string & target, const ros::Time & time,
    geometry_msgs::Pose & pose);
  void getNearestObstacle(double * min_dist_to_obj, geometry_msgs::Point * nearest_obj_point);
  void getNearestObstacleByPointCloud(
    double * min_dist_to_obj, geometry_msgs::Point * nearest_obj_point);
  void getNearestObstacleByDynamicObject(
    double * min_dist_to_obj, geometry_msgs::Point * nearest_obj_point);
  bool isObstacleFound(const double min_dist_to_obj);
  bool isStopRequired(const bool is_obstacle_found, const bool is_stopped);
  size_t getClosestIdx(
    const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::Pose current_pose);
  bool checkStop(const autoware_planning_msgs::TrajectoryPoint & closest_point);
  Polygon2d createSelfPolygon();
  Polygon2d createObjPolygon(const geometry_msgs::Pose & pose, const geometry_msgs::Vector3 & size);
  Polygon2d createObjPolygon(
    const geometry_msgs::Pose & pose, const geometry_msgs::Polygon & footprint);
  diagnostic_msgs::DiagnosticStatus makeStopReasonDiag(
    const std::string no_start_reason, const geometry_msgs::Pose & stop_pose);
  std::string jsonDumpsPose(const geometry_msgs::Pose & pose);

  /*
   * ROS
   */
  // publisher and subscriber
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber path_sub_;
  ros::Subscriber current_velocity_sub_;
  ros::Subscriber dynamic_object_sub_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher path_pub_;
  ros::Publisher stop_reason_diag_pub_;
  std::shared_ptr<SurroundObstacleCheckerDebugNode> debug_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // parameter
  geometry_msgs::TwistStamped::ConstPtr current_velocity_ptr_;
  sensor_msgs::PointCloud2::ConstPtr pointcloud_ptr_;
  autoware_perception_msgs::DynamicObjectArray::ConstPtr object_ptr_;
  double wheel_base_, front_overhang_, rear_overhang_, left_overhang_, right_overhang_,
    wheel_tread_, vehicle_width_, vehicle_length_;
  Polygon2d self_poly_;
  bool use_pointcloud_;
  bool use_dynamic_object_;
  double surround_check_distance_;
  // For preventing chattering,
  // surround_check_recover_distance_ must be  bigger than surround_check_distance_
  double surround_check_recover_distance_;
  double state_clear_time_;
  double stop_state_ego_speed_;
  bool is_surround_obstacle_;

  // State Machine
  State state_ = State::PASS;
  std::shared_ptr<const ros::Time> last_obstacle_found_time_;
};
