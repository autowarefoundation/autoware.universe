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

#include "autodrive_trajectory_follower.hpp"
#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>

using namespace std;

namespace autodrive_trajectory_follower
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

AutoDRIVETrajectoryFollower::AutoDRIVETrajectoryFollower(const rclcpp::NodeOptions & options)
: Node("autodrive_trajectory_follower", options)
{
  // General Parameters
  vehicle_name_ = declare_parameter<string>("vehicle_name", "nigel");
  controller_mode_ = declare_parameter<string>("controller_mode", "simulator");
  loop_trajectory_ = declare_parameter<bool>("loop_trajectory", false);
  use_external_target_vel_ = declare_parameter<bool>("use_external_target_vel", false);
  external_target_vel_ = declare_parameter<float>("external_target_vel", 0.0);
  lateral_deviation_ = declare_parameter<float>("lateral_deviation", 0.0);
  // Longitudinal Controller Parameters
  kp_lon_ = declare_parameter<float>("kp_longitudinal", 0.5);
  acc_lim_ = declare_parameter<float>("acceleration_limit", 2.0);
  // Lateral Controller Parameters
  wheel_base_ = declare_parameter<float>("wheel_base", 0.25);
  lookahead_time_ = declare_parameter<float>("lookahead_time", 3.0);
  min_lookahead_ = declare_parameter<float>("min_lookahead", 3.0);
  kp_lat_ = declare_parameter<float>("kp_lateral", 25.0);
  kd_lat_ = declare_parameter<float>("kd_lateral", 5.0);
  steer_lim_ = declare_parameter<float>("steer_limit", 1.0);

  // Nigel
  if(vehicle_name_=="nigel" && (controller_mode_=="simulator" || controller_mode_=="digitaltwin" || controller_mode_=="testbed")){
    nigel_simulator_throttle_pub_ = create_publisher<Float32>("/autodrive/nigel_1/throttle_command", 1);
    nigel_simulator_steering_pub_ = create_publisher<Float32>("/autodrive/nigel_1/steering_command", 1);
  }
  
  // F1TENTH
  if(vehicle_name_=="f1tenth" && (controller_mode_=="simulator" || controller_mode_=="digitaltwin")){
    f1tenth_simulator_throttle_pub_ = create_publisher<Float32>("/autodrive/f1tenth_1/throttle_command", 1);
    f1tenth_simulator_steering_pub_ = create_publisher<Float32>("/autodrive/f1tenth_1/steering_command", 1);
  }
  if(vehicle_name_=="f1tenth" && (controller_mode_=="gym_rviz" || controller_mode_=="testbed")){
    f1tenth_testbed_gym_rviz_pub_ = create_publisher<AckermannDriveStamped>("/drive", 1);
  }

  traj_marker_pub_ = create_publisher<Marker>("/traj_marker", 1);
  goal_marker_pub_ = create_publisher<Marker>("/goal_marker", 1);

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
  
  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(this, get_clock(), 30ms, std::bind(&AutoDRIVETrajectoryFollower::onTimer, this));
}

void AutoDRIVETrajectoryFollower::createTrajectoryMarker(){
  header.frame_id = "map";
  // Trajectory Marker
  scale.x = 0.1;
  scale.y = 0.1;
  scale.z = 0.1;
  traj_marker.type = traj_marker.POINTS;
  traj_marker.pose = pose;
  traj_marker.scale = scale;
  traj_marker.header = header;
  traj_marker.points.clear();
  traj_marker.lifetime = rclcpp::Duration::from_nanoseconds(0.03 * 1e9);
  for(int i = 0; i < (int)trajectory_->points.size(); i++){
    point.x = trajectory_->points[i].pose.position.x;
    point.y = trajectory_->points[i].pose.position.y;
    point.z = 0.0;
    traj_marker.points.push_back(point);
  }
  traj_marker.color.r = 0.0;
  traj_marker.color.g = 1.0;
  traj_marker.color.b = 0.0;
  traj_marker.color.a = 1.0;
  traj_marker_pub_->publish(traj_marker);
  // Goal (Target) Marker
  scale.x = 0.2;
  scale.y = 0.2;
  scale.z = 0.2;
  goal_marker.type = goal_marker.POINTS;
  goal_marker.pose = pose;
  goal_marker.scale = scale;
  goal_marker.header = header;
  goal_marker.lifetime = rclcpp::Duration::from_nanoseconds(0.03 * 1e9);
  point.x = closest_traj_point_.pose.position.x;
  point.y = closest_traj_point_.pose.position.y;
  point.z = 0.0;
  goal_marker.points.clear();
  goal_marker.points.push_back(point);
  goal_marker.color.r = 1.0;
  goal_marker.color.g = 1.0;
  goal_marker.color.b = 0.0;
  goal_marker.color.a = 1.0;
  goal_marker_pub_->publish(goal_marker);
}

void AutoDRIVETrajectoryFollower::onTimer()
{
  if(!checkData()) {
    return;
  }

  updateClosest();
  createTrajectoryMarker();

  double closest_px = closest_traj_point_.pose.position.x;
  double closest_py = closest_traj_point_.pose.position.y;
  double final_px = trajectory_->points.at(trajectory_->points.size()-1).pose.position.x;
  double final_py = trajectory_->points.at(trajectory_->points.size()-1).pose.position.y;

  AckermannControlCommand cmd;
  cmd.stamp = cmd.lateral.stamp = cmd.longitudinal.stamp = get_clock()->now();  
  if(!loop_trajectory_ && closest_px==final_px && closest_py==final_py){
    cmd.longitudinal.speed = 0.0;
    cmd.longitudinal.acceleration = 0.0;
    cmd.lateral.steering_tire_angle = 0.0;
  }
  else{
  cmd.longitudinal.speed = use_external_target_vel_ ? static_cast<float>(external_target_vel_)
                                                    : closest_traj_point_.longitudinal_velocity_mps;
  cmd.longitudinal.acceleration = static_cast<float>(calcAccCmd());
  cmd.lateral.steering_tire_angle = static_cast<float>(calcSteerCmd());
  }
  
  // Nigel
  if(vehicle_name_=="nigel" && (controller_mode_=="simulator" || controller_mode_=="digitaltwin" || controller_mode_=="testbed")){
    std_msgs::msg::Float32 throttle_msg;
    std_msgs::msg::Float32 steering_msg;
    throttle_msg.data = cmd.longitudinal.acceleration;
    steering_msg.data = cmd.lateral.steering_tire_angle;
    nigel_simulator_throttle_pub_->publish(throttle_msg);
    nigel_simulator_steering_pub_->publish(steering_msg);
  }
  
  // F1TEHTH
  if(vehicle_name_=="f1tenth" && (controller_mode_=="simulator" || controller_mode_=="digitaltwin")){
    std_msgs::msg::Float32 throttle_msg;
    std_msgs::msg::Float32 steering_msg;
    throttle_msg.data = cmd.longitudinal.acceleration;
    steering_msg.data = cmd.lateral.steering_tire_angle;
    f1tenth_simulator_throttle_pub_->publish(throttle_msg);
    f1tenth_simulator_steering_pub_->publish(steering_msg);
  }
  if(vehicle_name_=="f1tenth" && (controller_mode_=="gym_rviz" || controller_mode_=="testbed")){
    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
    ackermann_msg.drive.speed = cmd.longitudinal.speed * 0.2;
    ackermann_msg.drive.steering_angle = cmd.lateral.steering_tire_angle * 10;
    f1tenth_testbed_gym_rviz_pub_->publish(ackermann_msg);
  }

}

void AutoDRIVETrajectoryFollower::updateClosest()
{
  const auto closest = findNearestIndex(trajectory_->points, odometry_->pose.pose.position);
  closest_traj_point_ = trajectory_->points.at(closest);
}

double AutoDRIVETrajectoryFollower::calcAccCmd()
{
  // P-Feedback Control
  const auto traj_vel = static_cast<double>(closest_traj_point_.longitudinal_velocity_mps);
  const auto ego_vel = odometry_->twist.twist.linear.x;
  const auto target_vel = use_external_target_vel_ ? external_target_vel_ : traj_vel;
  const auto vel_err = ego_vel - target_vel;
  const auto acc = std::clamp(-kp_lon_ * vel_err, -acc_lim_, acc_lim_);
  return acc;
}

double AutoDRIVETrajectoryFollower::calcSteerCmd()
{
  // Linearized Pure-Pursuit Control
  const auto lat_err = calcLateralDeviation(closest_traj_point_.pose, odometry_->pose.pose.position) - lateral_deviation_;
  const auto yaw_err = calcYawDeviation(closest_traj_point_.pose, odometry_->pose.pose);
  const auto lookahead = min_lookahead_ + lookahead_time_ * std::abs(odometry_->twist.twist.linear.x);
  const auto kp = kp_lat_ * wheel_base_ / (lookahead * lookahead);
  const auto kd = kd_lat_ * wheel_base_ / lookahead;
  const auto steer = std::clamp(-kp * lat_err - kd * yaw_err, -steer_lim_, steer_lim_);
  return steer;
}

bool AutoDRIVETrajectoryFollower::checkData() { return (trajectory_ && odometry_); }

}  // namespace autodrive_trajectory_follower

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autodrive_trajectory_follower::AutoDRIVETrajectoryFollower)