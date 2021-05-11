// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

/**
 * @file simple_planning_simulator_core.hpp
 * @brief vehicle dynamics simulation for autoware
 * @author Takamasa Horibe
 * @date 2019.08.17
 */

#ifndef SIMPLE_PLANNING_SIMULATOR__SIMPLE_PLANNING_SIMULATOR_CORE_HPP_
#define SIMPLE_PLANNING_SIMULATOR__SIMPLE_PLANNING_SIMULATOR_CORE_HPP_

#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_debug_msgs/msg/float32_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_vehicle_msgs/msg/control_mode.hpp"
#include "autoware_vehicle_msgs/msg/engage.hpp"
#include "autoware_vehicle_msgs/msg/shift_stamped.hpp"
#include "autoware_vehicle_msgs/msg/steering.hpp"
#include "autoware_vehicle_msgs/msg/turn_signal.hpp"
#include "autoware_vehicle_msgs/msg/vehicle_command.hpp"

#include "simple_planning_simulator/vehicle_model/sim_model_constant_acceleration.hpp"
#include "simple_planning_simulator/vehicle_model/sim_model_ideal.hpp"
#include "simple_planning_simulator/vehicle_model/sim_model_interface.hpp"
#include "simple_planning_simulator/vehicle_model/sim_model_time_delay.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

class Simulator : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit Simulator(const std::string & node_name, const rclcpp::NodeOptions & options);

  /**
   * @brief default destructor
   */
  // ~Simulator() = default;

private:
  /* ros system */
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    pub_pose_;  //!< @brief topic ros publisher for current pose
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    pub_twist_;  //!< @brief topic ros publisher for current twist
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Steering>::SharedPtr pub_steer_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr pub_velocity_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr pub_turn_signal_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr pub_shift_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlMode>::SharedPtr pub_control_mode_;

  rclcpp::Subscription<autoware_vehicle_msgs::msg::VehicleCommand>::SharedPtr
    sub_vehicle_cmd_;  //!< @brief topic subscriber for vehicle_cmd
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr
    sub_turn_signal_cmd_;  //!< @brief topic subscriber for turn_signal_cmd
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_trajectory_;  //!< @brief topic subscriber for trajectory used for z position
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    sub_initialpose_;  //!< @brief topic subscriber for initialpose topic
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    sub_initialtwist_;  //!< @brief topic subscriber for initialtwist topic
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Engage>::SharedPtr
    sub_engage_;                                   //!< @brief topic subscriber for engage topic
  rclcpp::TimerBase::SharedPtr timer_simulation_;  //!< @brief timer for simulation

  /* tf */
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  /* received & published topics */
  geometry_msgs::msg::PoseStamped::ConstSharedPtr
    initial_pose_ptr_;  //!< @brief initial vehicle pose
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr
    initial_pose_with_cov_ptr_;  //!< @brief initial vehicle pose with cov
  geometry_msgs::msg::TwistStamped::ConstSharedPtr
    initial_twist_ptr_;                      //!< @brief initial vehicle velocity
  geometry_msgs::msg::Pose current_pose_;    //!< @brief current vehicle position and angle
  geometry_msgs::msg::Twist current_twist_;  //!< @brief current vehicle velocity
  autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr
    current_vehicle_cmd_ptr_;  //!< @brief latest received vehicle_cmd
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr
    current_trajectory_ptr_;  //!< @brief latest received trajectory
  double closest_pos_z_;      //!< @brief z position on closest trajectory
  autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr current_turn_signal_cmd_ptr_;
  autoware_vehicle_msgs::msg::ControlMode control_mode_;

  /* frame_id */
  std::string
    simulation_frame_id_;     //!< @brief vehicle frame id simulated by simple_planning_simulator
  std::string map_frame_id_;  //!< @brief map frame_id

  /* simple_planning_simulator parameters */
  double loop_rate_;  //!< @brief frequency to calculate vehicle model & publish result
  double wheelbase_;  //!< @brief wheelbase length to convert angular-velocity & steering
  double sim_steering_gear_ratio_;  //!< @brief for steering wheel angle calculation

  /* flags */
  bool is_initialized_ = false;                //!< @brief flag to check the initial position is set
  bool add_measurement_noise_;                 //!< @brief flag to add measurement noise
  bool simulator_engage_;                      //!< @brief flag to engage simulator
  bool use_trajectory_for_z_position_source_;  //!< @brief flag to get z position from trajectory

  /* saved values */
  std::shared_ptr<rclcpp::Time> prev_update_time_ptr_;  //!< @brief previously updated time

  /* vehicle model */
  enum class VehicleModelType
  {
    IDEAL_TWIST = 0,
    IDEAL_STEER = 1,
    DELAY_TWIST = 2,
    DELAY_STEER = 3,
    CONST_ACCEL_TWIST = 4,
    IDEAL_FORKLIFT_RLS = 5,
    DELAY_FORKLIFT_RLS = 6,
    IDEAL_ACCEL = 7,
    DELAY_STEER_ACC = 8,
  } vehicle_model_type_;  //!< @brief vehicle model type to decide the model dynamics
  std::shared_ptr<SimModelInterface> vehicle_model_ptr_;  //!< @brief vehicle model pointer

  /* to generate measurement noise */
  std::shared_ptr<std::mt19937> rand_engine_ptr_;  //!< @brief random engine for measurement noise
  std::shared_ptr<std::normal_distribution<>>
  pos_norm_dist_ptr_;    //!< @brief Gaussian noise for position
  std::shared_ptr<std::normal_distribution<>>
  vel_norm_dist_ptr_;    //!< @brief Gaussian noise for velocity
  std::shared_ptr<std::normal_distribution<>>
  rpy_norm_dist_ptr_;    //!< @brief Gaussian noise for roll-pitch-yaw
  std::shared_ptr<std::normal_distribution<>>
  angvel_norm_dist_ptr_;    //!< @brief Gaussian noise for angular velocity
  std::shared_ptr<std::normal_distribution<>>
  steer_norm_dist_ptr_;    //!< @brief Gaussian noise for steering angle

  /**
   * @brief set current_vehicle_cmd_ptr_ with received message
   */
  void callbackVehicleCmd(const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr msg);

  /**
   * @brief set current_turn_signal_cmd_ptr with received message
   */
  void callbackTurnSignalCmd(const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg);

  /**
   * @brief set current_trajectory_ptr_ with received message
   */
  void callbackTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  /**
   * @brief set initial pose for simulation with received message
   */
  void callbackInitialPoseWithCov(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  /**
   * @brief set initial pose with received message
   */
  void callbackInitialPoseStamped(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  /**
   * @brief set initial twist with received message
   */
  void callbackInitialTwistStamped(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);

  /**
   * @brief set simulator engage with received message
   */
  void callbackEngage(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg);

  /**
   * @brief get transform from two frame_ids
   * @param [in] parent_frame parent frame id
   * @param [in] child frame id
   * @param [out] transform transform from parent frame to child frame
   */
  void getTransformFromTF(
    const std::string parent_frame, const std::string child_frame,
    geometry_msgs::msg::TransformStamped & transform);

  /**
   * @brief timer callback for simulation with loop_rate
   */
  void timerCallbackSimulation();

  /**
   * @brief set initial state of simulated vehicle
   * @param [in] pose initial position and orientation
   * @param [in] twist initial velocity and angular velocity
   */
  void setInitialState(
    const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & twist);

  /**
   * @brief set initial state of simulated vehicle with pose transformation based on frame_id
   * @param [in] pose initial position and orientation with header
   * @param [in] twist initial velocity and angular velocity
   */
  void setInitialStateWithPoseTransform(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & twist);

  /**
   * @brief set initial state of simulated vehicle with pose transformation based on frame_id
   * @param [in] pose initial position and orientation with header
   * @param [in] twist initial velocity and angular velocity
   */
  void setInitialStateWithPoseTransform(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const geometry_msgs::msg::Twist & twist);

  /**
   * @brief publish pose and twist
   * @param [in] pose pose to be published
   * @param [in] twist twist to be published
   */
  void publishPoseTwist(
    const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & twist);

  /**
   * @brief publish tf
   * @param [in] pose pose used for tf
   */
  void publishTF(const geometry_msgs::msg::Pose & pose);

  /**
   * @brief update closest pose to calculate pos_z
   */
  double getPosZFromTrajectory(const double x, const double y);

  /**
   * @brief convert roll-pitch-yaw Euler angle to ros Quaternion
   * @param [in] roll roll angle [rad]
   * @param [in] pitch pitch angle [rad]
   * @param [in] yaw yaw angle [rad]
   */
  geometry_msgs::msg::Quaternion getQuaternionFromRPY(
    const double & roll, const double & pitch, const double & yaw);
};

#endif  // SIMPLE_PLANNING_SIMULATOR__SIMPLE_PLANNING_SIMULATOR_CORE_HPP_
