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
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include <dynamic_reconfigure/server.h>
#include <motion_velocity_optimizer/MotionVelocityOptimizerConfig.h>

#include <motion_velocity_optimizer/motion_velocity_optimizer_utils.hpp>

#include <osqp_interface/osqp_interface.h>

class MotionVelocityOptimizer
{
public:
  MotionVelocityOptimizer();
  ~MotionVelocityOptimizer();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_trajectory_;                //!< @brief publisher for output trajectory
  ros::Subscriber sub_current_velocity_;         //!< @brief subscriber for current velocity
  ros::Subscriber sub_current_trajectory_;       //!< @brief subscriber for reference trajectory
  ros::Subscriber sub_external_velocity_limit_;  //!< @brief subscriber for external velocity limit
  tf2_ros::Buffer tf_buffer_;                    //!< @brief tf butter
  tf2_ros::TransformListener tf_listener_;       //!< @brief tf listener
  ros::Timer timer_;

  boost::shared_ptr<geometry_msgs::PoseStamped const> current_pose_ptr_;  // current vehicle pose
  boost::shared_ptr<geometry_msgs::TwistStamped const>
    current_velocity_ptr_;  // current vehicle twist
  boost::shared_ptr<autoware_planning_msgs::Trajectory const>
    base_traj_raw_ptr_;  // current base_waypoints
  boost::shared_ptr<std_msgs::Float32 const>
    external_velocity_limit_ptr_;  // current external_velocity_limit
  boost::shared_ptr<double> external_velocity_limit_acc_limited_ptr_;

  autoware_planning_msgs::Trajectory prev_output_;  // previously published trajectory

  enum class InitializeType {
    INIT = 0,
    LARGE_DEVIATION_REPLAN = 1,
    ENGAGING = 2,
    NORMAL = 3,
  };
  InitializeType initialize_type_;

  osqp::OSQPInterface qp_solver_;

  bool show_debug_info_;      // printF level 1
  bool show_debug_info_all_;  // print level 2
  bool show_figure_;          // for plot visualize
  bool publish_debug_trajs_;  // publish planned trajectories

  struct MotionVelocityOptimizerParam
  {
    double max_velocity;        // max velocity [m/s]
    double max_accel;           // max acceleration in planning [m/s2] > 0
    double min_decel;           // min deceltion in planning [m/s2] < 0
    double max_lateral_accel;   // max lateral acceleartion [m/ss] > 0
    double min_curve_velocity;  // min velocity at curve [m/s]
    double
      decel_distance_before_curve;  // distance before you slow down for lateral acceleration limit at a curve
    double
      decel_distance_after_curve;  // distance after you slow down for lateral acceleration limit at a curve
    double
      replan_vel_deviation;  // replan with current speed if speed deviation exceeds this value [m/s]
    double engage_velocity;      // use this speed when start moving [m/s]
    double engage_acceleration;  // use this acceleration when start moving [m/ss]
    double
      engage_exit_ratio;  // exit engage sequence when the velocity exceeds ratio x engage_velocity.
    double extract_ahead_dist;     // forward waypoints distance from current position [m]
    double extract_behind_dist;    // backward waypoints distance from current position [m]
    double max_trajectory_length;  // max length of the objective trajectory for resample
    double min_trajectory_length;  // min length of the objective trajectory for resample
    double resample_time;          // max time to calculate trajectory length
    double resample_dt;            // dt to calculate trajectory length
    double
      min_trajectory_interval_distance;  // minimum interval distance between each trajectory points
    double
      stop_dist_to_prohibit_engage;  // set zero vel when vehicle stops and stop dist is closer than this
    double
      delta_yaw_threshold;  // delta yaw between ego_pose and traj point when calc closest point
  } planning_param_;

  struct QPParam
  {
    double pseudo_jerk_weight;
    double over_v_weight;
    double over_a_weight;
  } qp_param_;

  /* topic callback */
  void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr msg);
  void callbackCurrentTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr msg);
  void callbackExternalVelocityLimit(const std_msgs::Float32::ConstPtr msg);
  void timerCallback(const ros::TimerEvent & e);

  /* non-const methods */
  void run();
  void updateCurrentPose();
  autoware_planning_msgs::Trajectory calcTrajectoryVelocity(
    const autoware_planning_msgs::Trajectory & base_traj);
  void updateExternalVelocityLimit(const double dt);
  void optimizeVelocity(
    const autoware_planning_msgs::Trajectory & input, const int input_closest,
    const autoware_planning_msgs::Trajectory & prev_output_traj, const int prev_output_closest,
    autoware_planning_msgs::Trajectory & output);
  void calcInitialMotion(
    const double & base_speed, const autoware_planning_msgs::Trajectory & base_waypoints,
    const int base_closest, const autoware_planning_msgs::Trajectory & prev_replanned_traj,
    const int prev_replanned_traj_closest, double & initial_vel, double & initial_acc);
  void solveOptimization(
    const double initial_vel, const double initial_acc,
    const autoware_planning_msgs::Trajectory & input, const int closest,
    autoware_planning_msgs::Trajectory & output);

  /* const methods */
  bool resampleTrajectory(
    const autoware_planning_msgs::Trajectory & input,
    autoware_planning_msgs::Trajectory & output) const;

  bool lateralAccelerationFilter(
    const autoware_planning_msgs::Trajectory & input,
    autoware_planning_msgs::Trajectory & output) const;
  bool extractPathAroundIndex(
    const autoware_planning_msgs::Trajectory & input, const int index,
    autoware_planning_msgs::Trajectory & output) const;
  bool externalVelocityLimitFilter(
    const autoware_planning_msgs::Trajectory & input,
    autoware_planning_msgs::Trajectory & output) const;
  void preventMoveToCloseStopLine(
    const int closest, autoware_planning_msgs::Trajectory & trajectory) const;

  void publishTrajectory(const autoware_planning_msgs::Trajectory & traj) const;
  void publishStopDistance(
    const autoware_planning_msgs::Trajectory & trajectory, const int closest) const;
  void insertBehindVelocity(
    const int prev_out_closest, const autoware_planning_msgs::Trajectory & prev_output,
    const int output_closest, autoware_planning_msgs::Trajectory & output) const;

  /* dynamic reconfigure */
  dynamic_reconfigure::Server<motion_velocity_optimizer::MotionVelocityOptimizerConfig>
    dyncon_server_;
  void dynamicRecofCallback(
    motion_velocity_optimizer::MotionVelocityOptimizerConfig & config, uint32_t level)
  {
    planning_param_.max_velocity = config.max_velocity;
    planning_param_.max_accel = config.max_accel;
    planning_param_.min_decel = config.min_decel;
    planning_param_.max_lateral_accel = config.max_lateral_accel;
    planning_param_.min_curve_velocity = config.min_curve_velocity;
    planning_param_.decel_distance_before_curve = config.decel_distance_before_curve;
    planning_param_.decel_distance_after_curve = config.decel_distance_after_curve;
    planning_param_.replan_vel_deviation = config.replan_vel_deviation;
    planning_param_.engage_velocity = config.engage_velocity;
    planning_param_.engage_acceleration = config.engage_acceleration;
    planning_param_.engage_exit_ratio = config.engage_exit_ratio;
    planning_param_.extract_ahead_dist = config.extract_ahead_dist;
    planning_param_.extract_behind_dist = config.extract_behind_dist;
    planning_param_.stop_dist_to_prohibit_engage = config.stop_dist_to_prohibit_engage;
    planning_param_.delta_yaw_threshold = config.delta_yaw_threshold;

    planning_param_.resample_time = config.resample_time;
    planning_param_.resample_dt = config.resample_dt;
    planning_param_.max_trajectory_length = config.max_trajectory_length;
    planning_param_.min_trajectory_length = config.min_trajectory_length;
    planning_param_.min_trajectory_interval_distance = config.min_trajectory_interval_distance;

    qp_param_.pseudo_jerk_weight = config.pseudo_jerk_weight;
    qp_param_.over_v_weight = config.over_v_weight;
    qp_param_.over_a_weight = config.over_a_weight;

    show_debug_info_ = config.show_debug_info;
  }

  /* debug */
  ros::Publisher pub_dist_to_stopline_;  //!< @brief publisher for stop distance
  ros::Publisher pub_trajectory_raw_;
  ros::Publisher pub_trajectory_vel_lim_;
  ros::Publisher pub_trajectory_latcc_filtered_;
  ros::Publisher pub_trajectory_resampled_;
  ros::Publisher debug_closest_velocity_;
  void publishClosestVelocity(const double & vel) const;
};
