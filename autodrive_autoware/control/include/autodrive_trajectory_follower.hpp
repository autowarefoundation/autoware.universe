//  Copyright 2023 Tinker Twins, Tier IV, Inc. All rights reserved.
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

#ifndef AUTODRIVE_TRAJECTORY_FOLLOWER__AUTODRIVE_TRAJECTORY_FOLLOWER_HPP_
#define AUTODRIVE_TRAJECTORY_FOLLOWER__AUTODRIVE_TRAJECTORY_FOLLOWER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <memory>

namespace autodrive_trajectory_follower
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using ackermann_msgs::msg::AckermannDriveStamped;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::Header;
using geometry_msgs::msg::Point;
using std_msgs::msg::Float32;

class AutoDRIVETrajectoryFollower : public rclcpp::Node
{
public:
  explicit AutoDRIVETrajectoryFollower(const rclcpp::NodeOptions & options);
  ~AutoDRIVETrajectoryFollower() = default;

private:
  // Nigel
  rclcpp::Publisher<Float32>::SharedPtr nigel_digitaltwin_throttle_pub_;
  rclcpp::Publisher<Float32>::SharedPtr nigel_digitaltwin_steering_pub_;

  // F1TENTH
  rclcpp::Publisher<Float32>::SharedPtr f1tenth_simulator_throttle_pub_;
  rclcpp::Publisher<Float32>::SharedPtr f1tenth_simulator_steering_pub_;
  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr f1tenth_testbed_gym_rviz_pub_;

  // Hunter SE
  rclcpp::Publisher<Float32>::SharedPtr hunter_simulator_throttle_pub_;
  rclcpp::Publisher<Float32>::SharedPtr hunter_simulator_steering_pub_;
  rclcpp::Publisher<Twist>::SharedPtr hunter_testbed_pub_;

  // OpenCAV
  rclcpp::Publisher<Float32>::SharedPtr opencav_digitaltwin_throttle_pub_;
  rclcpp::Publisher<Float32>::SharedPtr opencav_digitaltwin_steering_pub_;
  rclcpp::Publisher<Float32>::SharedPtr opencav_digitaltwin_brake_pub_;
  rclcpp::Publisher<Float32>::SharedPtr opencav_digitaltwin_handbrake_pub_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Publisher<Marker>::SharedPtr traj_marker_pub_;
  rclcpp::Publisher<Marker>::SharedPtr goal_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Trajectory::SharedPtr trajectory_;
  Odometry::SharedPtr odometry_;

  TrajectoryPoint closest_traj_point_;

  Marker traj_marker;
  Marker goal_marker;
  Pose pose;
  Point point;
  Vector3 scale;
  Header header;

  // General Parameters
  std::string vehicle_name_;
  std::string controller_mode_;
  bool loop_trajectory_;
  bool use_external_target_vel_;
  double external_target_vel_;
  double lateral_deviation_;
  // Longitudinal Controller Parameters
  double kp_lon_;
  double acc_lim_;
  // Lateral Controller Parameters
  double wheel_base_;
  double lookahead_time_;
  double min_lookahead_;
  double kp_lat_;
  double kd_lat_;
  double steer_lim_;

  void onTimer();
  bool checkData();
  void updateClosest();
  double calcSteerCmd();
  double calcAccCmd();
  void createTrajectoryMarker();
};

}  // namespace autodrive_trajectory_follower

#endif  // AUTODRIVE_TRAJECTORY_FOLLOWER__AUTODRIVE_TRAJECTORY_FOLLOWER_HPP_