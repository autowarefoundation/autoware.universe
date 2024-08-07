//  Copyright 2022 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef TRAJECTORY_FOLLOWER_F1TENTH__F1TENTH_TRAJECTORY_FOLLOWER_HPP_
#define TRAJECTORY_FOLLOWER_F1TENTH__F1TENTH_TRAJECTORY_FOLLOWER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>

namespace AutowarePlanningMsgs = autoware_planning_msgs::msg;
namespace AutowareAutoPlanningMsgs = autoware_auto_planning_msgs::msg;
namespace f1tenth_trajectory_follower
{
using ackermann_msgs::msg::AckermannDriveStamped;
using autoware_control_msgs::msg::Control;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Header;
using visualization_msgs::msg::Marker;

class F1tenthTrajectoryFollower : public rclcpp::Node
{
public:
  explicit F1tenthTrajectoryFollower(const rclcpp::NodeOptions & options);
  ~F1tenthTrajectoryFollower() = default;

private:
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<AutowareAutoPlanningMsgs::Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr drive_cmd_;
  rclcpp::Publisher<Marker>::SharedPtr traj_marker_pub_;
  rclcpp::Publisher<Marker>::SharedPtr goal_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  AutowareAutoPlanningMsgs::Trajectory::SharedPtr autoware_auto_trajectory_;
  Odometry::SharedPtr odometry_;

  AutowarePlanningMsgs::Trajectory trajectory_;
  AutowarePlanningMsgs::TrajectoryPoint closest_traj_point_;

  Marker marker;
  Marker goal_marker;
  Pose pose;
  Point point;
  Vector3 scale;
  Header header;

  bool use_external_target_vel_;
  double external_target_vel_;
  double lateral_deviation_;

  void convertTrajectoryMsg();
  void onTimer();
  bool checkData();
  void updateClosest();
  double calcSteerCmd();
  double calcAccCmd();
  void createTrajectoryMarker();
};

}  // namespace f1tenth_trajectory_follower

#endif  // TRAJECTORY_FOLLOWER_F1TENTH__F1TENTH_TRAJECTORY_FOLLOWER_HPP_
