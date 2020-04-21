/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

/**
 * @file simple_planning_simulator_core.hpp
 * @brief vehicle dynamics simulation for autoware
 * @author Takamasa Horibe
 * @date 2019.08.17
 */

#ifndef SIMPLE_PLANNING_SIMULATOR_CORE_H_
#define SIMPLE_PLANNING_SIMULATOR_CORE_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <random>

#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_vehicle_msgs/ControlMode.h>
#include <autoware_vehicle_msgs/ShiftStamped.h>
#include <autoware_vehicle_msgs/Steering.h>
#include <autoware_vehicle_msgs/TurnSignal.h>
#include <autoware_vehicle_msgs/VehicleCommand.h>

#include "simple_planning_simulator/vehicle_model/sim_model_constant_acceleration.hpp"
#include "simple_planning_simulator/vehicle_model/sim_model_ideal.hpp"
#include "simple_planning_simulator/vehicle_model/sim_model_interface.hpp"
#include "simple_planning_simulator/vehicle_model/sim_model_time_delay.hpp"

class Simulator
{
public:
  /**
   * @brief constructor
   */
  Simulator();

  /**
   * @brief default destructor
   */
  // ~Simulator() = default;

private:
  /* ros system */
  ros::NodeHandle nh_;        //!< @brief ros node handle
  ros::NodeHandle pnh_;       //!< @brief private ros node handle
  ros::Publisher pub_pose_;   //!< @brief topic ros publisher for current pose
  ros::Publisher pub_twist_;  //!< @brief topic ros publisher for current twist
  ros::Publisher pub_steer_;
  ros::Publisher pub_velocity_;
  ros::Publisher pub_turn_signal_;
  ros::Publisher pub_shift_;

  ros::Subscriber sub_vehicle_cmd_;  //!< @brief topic subscriber for vehicle_cmd
  ros::Subscriber sub_trajectory_;   //!< @brief topic subscriber for trajectory used for z ppsition
  ros::Subscriber sub_initialpose_;  //!< @brief topic subscriber for initialpose topic
  ros::Timer timer_simulation_;      //!< @brief timer for simulation

  /* tf */
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;        //!< @brief tf listener
  tf2_ros::TransformBroadcaster tf_broadcaster_;  //!< @brief tf broadcaster

  /* received & published topics */
  geometry_msgs::Pose
    current_pose_;  //!< @brief current vehicle position ang angle with pose message class
  geometry_msgs::Twist
    current_twist_;  //!< @brief current vehicle velocity with twist message class
  std::shared_ptr<autoware_vehicle_msgs::VehicleCommand>
    current_vehicle_cmd_ptr_;  //!< @brief latest received vehicle_cmd
  std::shared_ptr<autoware_planning_msgs::Trajectory>
    current_trajectory_ptr_;  //!< @brief latest received trajectory
  double closest_pos_z_;      //!< @brief z position on closest trajectory

  /* frame_id */
  std::string
    simulation_frame_id_;     //!< @brief vehicle frame id simulated by simple_planning_simulator
  std::string map_frame_id_;  //!< @brief map frame_id

  /* simple_planning_simulator parameters */
  double loop_rate_;  //!< @brief frequency to calculate vehicle model & pubish result
  double wheelbase_;  //!< @brief wheelbase length to convert angular-velocity & steering
  double sim_steering_gear_ratio_;  //!< @brief for steering wheel angle calcultion

  /* flags */
  bool is_initialized_;         //!< @brief flag to check the initial position is set
  bool add_measurement_noise_;  //!< @brief flag to add measurement noise

  /* saved values */
  std::shared_ptr<ros::Time> prev_update_time_ptr_;  //!< @brief previously updated time

  /* vehicle model */
  enum class VehicleModelType {
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
    pos_norm_dist_ptr_;  //!< @brief Gaussian noise for position
  std::shared_ptr<std::normal_distribution<>>
    vel_norm_dist_ptr_;  //!< @brief Gaussian noise for velocity
  std::shared_ptr<std::normal_distribution<>>
    rpy_norm_dist_ptr_;  //!< @brief Gaussian noise for roll-pitch-yaw
  std::shared_ptr<std::normal_distribution<>>
    angvel_norm_dist_ptr_;  //!< @brief Gaussian noise for angular velocity
  std::shared_ptr<std::normal_distribution<>>
    steer_norm_dist_ptr_;  //!< @brief Gaussian noise for steering angle

  /**
   * @brief set current_vehicle_cmd_ptr_ with received message
   */
  void callbackVehicleCmd(const autoware_vehicle_msgs::VehicleCommandConstPtr & msg);

  /**
   * @brief set current_trajectory_ptr_ with received message
   */
  void callbackTrajectory(const autoware_planning_msgs::TrajectoryConstPtr & msg);

  /**
   * @brief set initial pose for simulation with received message
   */
  void callbackInitialPoseWithCov(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);

  /**
   * @brief set initial pose with received message
   */
  void callbackInitialPoseStamped(const geometry_msgs::PoseStampedConstPtr & msg);

  /**
   * @brief get transform from two frame_ids
   * @param [in] parent_frame parent frame id
   * @param [in] child frame id
   * @param [out] transform transform from parent frame to child frame
   */
  void getTransformFromTF(
    const std::string parent_frame, const std::string child_frame,
    geometry_msgs::TransformStamped & transform);

  /**
   * @brief timer callback for simulation with loop_rate
   */
  void timerCallbackSimulation(const ros::TimerEvent & e);

  /**
   * @brief set initial state of simulated vehicle
   * @param [in] pose initial position and orientation
   * @param [in] twist initial velocity and angular velocity
   */
  void setInitialState(const geometry_msgs::Pose & pose, const geometry_msgs::Twist & twist);

  /**
   * @brief set initial state of simulated vehicle with pose transformation based on frame_id
   * @param [in] pose initial position and orientation with header
   * @param [in] twist initial velocity and angular velocity
   */
  void setInitialStateWithPoseTransform(
    const geometry_msgs::PoseStamped & pose, const geometry_msgs::Twist & twist);

  /**
   * @brief set initial state of simulated vehicle with pose transformation based on frame_id
   * @param [in] pose initial position and orientation with header
   * @param [in] twist initial velocity and angular velocity
   */
  void setInitialStateWithPoseTransform(
    const geometry_msgs::PoseWithCovarianceStamped & pose, const geometry_msgs::Twist & twist);

  /**
   * @brief publish pose and twist
   * @param [in] pose pose to be published
   * @param [in] twist twist to be published
   */
  void publishPoseTwist(const geometry_msgs::Pose & pose, const geometry_msgs::Twist & twist);

  /**
   * @brief publish tf
   * @param [in] pose pose used for tf
   */
  void publishTF(const geometry_msgs::Pose & pose);

  /**
   * @brief update closest pose to calculate pos_z
   */
  double getPosZFromTrajectory(const double x, const double y);

  /**
   * @brief convert roll-pitch-yaw Eular angle to ros Quaternion
   * @param [in] roll roll angle [rad]
   * @param [in] pitch pitch angle [rad]
   * @param [in] yaw yaw angle [rad]
   */
  geometry_msgs::Quaternion getQuaternionFromRPY(
    const double & roll, const double & pitch, const double & yaw);
};

#endif