/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 * @file moc_follower.h
 * @brief mpc follower class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

#pragma once
#include <unistd.h>
#include <chrono>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <mpc_follower/MPCFollowerConfig.h>

#include <osqp_interface/osqp_interface.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_vehicle_msgs/Steering.h>

#include "mpc_follower/interpolate.h"
#include "mpc_follower/lowpass_filter.h"
#include "mpc_follower/mpc_trajectory.h"
#include "mpc_follower/mpc_utils.h"
#include "mpc_follower/qp_solver/qp_solver_osqp.h"
#include "mpc_follower/qp_solver/qp_solver_unconstr_fast.h"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_dynamics.h"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics.h"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.h"

/**
 * @class MPC-based waypoints follower class
 * @brief calculate control command to follow reference waypoints
 */
class MPCFollower
{
public:
  /**
   * @brief constructor
   */
  MPCFollower();

  /**
   * @brief destructor
   */
  ~MPCFollower();

private:
  ros::NodeHandle nh_;                  //!< @brief ros node handle
  ros::NodeHandle pnh_;                 //!< @brief private ros node handle
  ros::Publisher pub_ctrl_cmd_;         //!< @brief topic publisher for control command
  ros::Publisher pub_debug_steer_cmd_;  //!< @brief topic publisher for control command
  ros::Subscriber sub_ref_path_;        //!< @brief topic subscriber for reference waypoints
  ros::Subscriber sub_steering_;        //!< @brief subscriber for currrent steering
  ros::Subscriber sub_current_vel_;     //!< @brief subscriber for currrent velocity
  ros::Timer timer_control_;            //!< @brief timer for control command computation

  MPCTrajectory ref_traj_;                //!< @brief reference trajectory to be followed
  Butterworth2dFilter lpf_steering_cmd_;  //!< @brief lowpass filter for steering command
  Butterworth2dFilter
    lpf_lateral_error_;  //!< @brief lowpass filter for lateral error to calculate derivatie
  Butterworth2dFilter
    lpf_yaw_error_;  //!< @brief lowpass filter for heading error to calculate derivatie
  std::string vehicle_model_type_;                            //!< @brief vehicle model type for MPC
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr_;  //!< @brief vehicle model for MPC
  std::shared_ptr<QPSolverInterface> qpsolver_ptr_;           //!< @brief qp solver for MPC
  std::deque<double>
    input_buffer_;  //!< @brief control input (mpc_output) buffer for delay time conpemsation

  /* parameters for control*/
  double ctrl_period_;  //!< @brief control frequency [s]
  double
    steering_lpf_cutoff_hz_;  //!< @brief cutoff frequency of lowpass filter for steering command [Hz]
  double
    admisible_position_error_;  //!< @brief stop MPC calculation when lateral error is large than this value [m]
  double
    admisible_yaw_error_;  //!< @brief stop MPC calculation when heading error is large than this value [rad]
  double steer_lim_;       //!< @brief steering command limit [rad]
  double steer_rate_lim_;  //!< @brief steering rate limit [rad/s]
  double
    wheelbase_;  //!< @brief vehicle wheelbase length [m] to convert steering angle to angular velocity

  /* parameters for path smoothing */
  bool enable_path_smoothing_;      //< @brief flag for path smoothing
  bool enable_yaw_recalculation_;   //< @brief flag for recalculation of yaw angle after resampling
  int path_filter_moving_ave_num_;  //< @brief param of moving average filter for path smoothing
  int
    curvature_smoothing_num_;  //< @brief point-to-point index distance used in curvature calculation
  double traj_resample_dist_;  //< @brief path resampling interval [m]

  struct MPCParam
  {
    int prediction_horizon;                   //< @brief prediction horizon step
    double prediction_dt;                     //< @brief prediction horizon sampleing time
    double weight_lat_error;                  //< @brief lateral error weight in matrix Q
    double weight_heading_error;              //< @brief heading error weight in matrix Q
    double weight_heading_error_squared_vel;  //< @brief heading error * velocity weight in matrix Q
    double weight_steering_input;             //< @brief steering error weight in matrix R
    double
      weight_steering_input_squared_vel;   //< @brief steering error * velocity weight in matrix R
    double weight_lat_jerk;                //< @brief lateral jerk weight in matrix R
    double weight_steer_rate;              //< @brief steering rate weight in matrix R
    double weight_steer_acc;               //< @brief steering angle acceleration weight in matrix R
    double weight_terminal_lat_error;      //< @brief terminal lateral error weight in matrix Q
    double weight_terminal_heading_error;  //< @brief terminal heading error weight in matrix Q
    double zero_ff_steer_deg;              //< @brief threshold that feed-forward angle becomes zero
    double input_delay;  //< @brief delay time for steering input to be compensated
  };
  MPCParam mpc_param_;  // for mpc design parameter

  struct MPCMatrix
  {
    Eigen::MatrixXd Aex;
    Eigen::MatrixXd Bex;
    Eigen::MatrixXd Wex;
    Eigen::MatrixXd Cex;
    Eigen::MatrixXd Qex;
    Eigen::MatrixXd R1ex;
    Eigen::MatrixXd R2ex;
    Eigen::MatrixXd Urefex;
    Eigen::MatrixXd Yrefex;
  };

  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr_;  //!< @brief current measured pose
  std::shared_ptr<geometry_msgs::TwistStamped>
    current_velocity_ptr_;                     //!< @brief current measured velocity
  std::shared_ptr<double> current_steer_ptr_;  //!< @brief current measured steering
  std::shared_ptr<autoware_planning_msgs::Trajectory>
    current_trajectory_ptr_;  //!< @brief referece trajectory

  double raw_steer_cmd_prev_;  //< @brief steering command calculated by mpc in previous period
  double
    raw_steer_cmd_pprev_;  //< @brief steering command calculated by mpc in two times previous period
  double
    steer_cmd_prev_;  //< @brief steering command calculated by mpc and some filters in previous period
  double lateral_error_prev_;  //< @brief previous lateral error for derivative
  double yaw_error_prev_;      //< @brief previous lateral error for derivative

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;  //!< @brief tf listener

  /**
   * @brief compute and publish control command for path follow with a constant control period
   */
  void timerCallback(const ros::TimerEvent &);

  /**
   * @brief set current_trajectory_ with receved message
   */
  void callbackTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr &);

  /**
   * @brief update current_pose from tf
   */
  void updateCurrentPose();

  /**
   * @brief check if the received data is valid.
   */
  bool checkData();

  /**
   * @brief get varables for mpc calculation
   */
  bool getVar(
    int * closest_idx, double * closest_time, geometry_msgs::Pose * closest_pose, double * steer,
    double * lat_err, double * yaw_err);

  /**
   * @brief set curent_steer with receved message
   */
  void callbackSteering(const autoware_vehicle_msgs::Steering & msg);

  /**
   * @brief set current_velocity with receved message
   */
  void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg);

  /**
   * @brief publish control command as autoware_msgs/ControlCommand type
   * @param [in] cmd published control command
   */
  void publishCtrlCmd(const autoware_control_msgs::ControlCommand & cmd);

  /**
   * @brief calculate control command by MPC algorithm
   * @param [out] cmd calculated control command with mpc algorithm
   */
  bool calculateMPC(autoware_control_msgs::ControlCommand * cmd);

  /**
   * @brief set initial condition for mpc
   * @param [in] lat_err lateral error
   * @param [in] yaw_err yaw error
   */
  Eigen::VectorXd getInitialState(
    const double & lat_err, const double & yaw_err, const double & steer);

  /**
   * @brief update status for delay compensation
   * @param [in] start_time start time for update
   * @param [out] x updated state at delayed_time
   */
  bool updateStateForDelayCompensation(const double & start_time, Eigen::VectorXd * x);

  /**
   * @brief generate MPC matrix with trajectory and vehicle model
   * @param [in] reference_trajectory used for linearization around reference trajectory
   */
  MPCMatrix generateMPCMatrix(const MPCTrajectory & reference_trajectory);

  /**
   * @brief generate MPC matrix with trajectory and vehicle model
   * @param [out] Uex optimized input vector
   */
  bool executeOptimization(
    const MPCMatrix & mpc_matrix, const Eigen::VectorXd & x0, Eigen::VectorXd * Uex);

  /**
   * @brief get stop command
   */
  autoware_control_msgs::ControlCommand getStopControlCommand() const;

  bool resampleMPCTrajectoryByTime(
    double start_time, const MPCTrajectory & input, MPCTrajectory * output) const;
  MPCTrajectory calcActualVelocity(const int closest, const MPCTrajectory & trajectory);
  double getPredictionTime() const;
  void addSteerWeightR(Eigen::MatrixXd * R) const;
  void addSteerWeightF(Eigen::MatrixXd * f) const;

  /* dynamic reconfigure */
  dynamic_reconfigure::Server<mpc_follower::MPCFollowerConfig> dyncon_server_;
  void dynamicRecofCallback(mpc_follower::MPCFollowerConfig & config, uint32_t level)
  {
    mpc_param_.prediction_horizon = config.mpc_prediction_horizon;
    mpc_param_.prediction_dt = config.mpc_prediction_dt;
    mpc_param_.weight_lat_error = config.mpc_weight_lat_error;
    mpc_param_.weight_heading_error = config.mpc_weight_heading_error;
    mpc_param_.weight_heading_error_squared_vel = config.mpc_weight_heading_error_squared_vel;
    mpc_param_.weight_steering_input = config.mpc_weight_steering_input;
    mpc_param_.weight_steering_input_squared_vel = config.mpc_weight_steering_input_squared_vel;
    mpc_param_.weight_lat_jerk = config.mpc_weight_lat_jerk;
    mpc_param_.weight_steer_rate = config.mpc_weight_steer_rate;
    mpc_param_.weight_steer_acc = config.mpc_weight_steer_acc;
    mpc_param_.weight_terminal_lat_error = config.mpc_weight_terminal_lat_error;
    mpc_param_.weight_terminal_heading_error = config.mpc_weight_terminal_heading_error;
    mpc_param_.zero_ff_steer_deg = config.mpc_zero_ff_steer_deg;
  }

  /* ---------- debug ---------- */
  bool show_debug_info_;  //!< @brief flag to display debug info
  ros::Publisher pub_debug_marker_;
  ros::Publisher pub_debug_values_;             //!< @brief publisher for debug info
  ros::Publisher pub_debug_mpc_calc_time_;      //!< @brief publisher for debug info
  ros::Subscriber sub_estimate_twist_;          //!< @brief subscriber for /estimate_twist for debug
  geometry_msgs::TwistStamped estimate_twist_;  //!< @brief received /estimate_twist for debug
};
