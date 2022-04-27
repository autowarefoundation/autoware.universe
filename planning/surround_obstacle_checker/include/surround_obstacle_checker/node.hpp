// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURROUND_OBSTACLE_CHECKER__NODE_HPP_
#define SURROUND_OBSTACLE_CHECKER__NODE_HPP_

#include "surround_obstacle_checker/debug_marker.hpp"
#include "tier4_autoware_utils/trajectory/tmp_conversion.hpp"

#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace surround_obstacle_checker
{

using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using tier4_planning_msgs::msg::VelocityLimit;
using tier4_planning_msgs::msg::VelocityLimitClearCommand;
using vehicle_info_util::VehicleInfo;

enum class State { PASS, STOP };

class SurroundObstacleCheckerNode : public rclcpp::Node
{
public:
  explicit SurroundObstacleCheckerNode(const rclcpp::NodeOptions & node_options);

private:
  void onTimer();

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
  void dynamicObjectCallback(const PredictedObjects::ConstSharedPtr input_msg);
  void currentVelocityCallback(const nav_msgs::msg::Odometry::ConstSharedPtr input_msg);
  bool convertPose(
    const geometry_msgs::msg::Pose & pose, const std::string & source, const std::string & target,
    const rclcpp::Time & time, geometry_msgs::msg::Pose & conv_pose);
  void getNearestObstacle(double * min_dist_to_obj, geometry_msgs::msg::Point * nearest_obj_point);
  void getNearestObstacleByPointCloud(
    double * min_dist_to_obj, geometry_msgs::msg::Point * nearest_obj_point);
  void getNearestObstacleByDynamicObject(
    double * min_dist_to_obj, geometry_msgs::msg::Point * nearest_obj_point);
  bool isObstacleFound(const double min_dist_to_obj);
  bool isStopRequired(const bool is_obstacle_found, const bool is_stopped);
  bool checkStop(const TrajectoryPoint & closest_point);

  bool isVehicleStopped();

  /*
   * ROS
   */
  // publisher and subscriber
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr dynamic_object_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_velocity_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr stop_reason_diag_pub_;
  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr pub_clear_velocity_limit_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;
  std::shared_ptr<SurroundObstacleCheckerDebugNode> debug_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // parameter
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_ptr_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_ptr_;
  PredictedObjects::ConstSharedPtr object_ptr_;
  vehicle_info_util::VehicleInfo vehicle_info_;
  bool use_pointcloud_;
  bool use_dynamic_object_;
  double surround_check_distance_;
  // For preventing chattering,
  // surround_check_recover_distance_ must be  bigger than surround_check_distance_
  double surround_check_recover_distance_;
  double state_clear_time_;
  double stop_state_ego_speed_;
  double stopped_state_entry_duration_time_;
  bool is_surround_obstacle_;

  // State Machine
  State state_ = State::PASS;
  std::shared_ptr<const rclcpp::Time> last_obstacle_found_time_;
  std::shared_ptr<const rclcpp::Time> last_running_time_;
};
}  // namespace surround_obstacle_checker

#endif  // SURROUND_OBSTACLE_CHECKER__NODE_HPP_
