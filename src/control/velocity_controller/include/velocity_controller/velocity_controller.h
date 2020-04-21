/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not enable this file except in compliance with the License.
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

#ifndef VELOCITY_CONTROLLER
#define VELOCITY_CONTROLLER

#include <memory>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_planning_msgs/Trajectory.h>

#include "delay_compensation.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "velocity_controller_mathutils.h"

struct CtrlCmd
{
  double vel;
  double acc;
};

class VelocityController
{
public:
  VelocityController();
  ~VelocityController() = default;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_current_vel_;
  ros::Subscriber sub_trajectory_;
  ros::Publisher pub_control_cmd_;
  ros::Publisher pub_debug_;
  ros::Timer timer_control_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;  //!< @brief tf listener

  // parameters
  // enabled flags
  bool show_debug_info_;  //!< @brief flag to show debug info
  bool enable_smooth_stop_;
  bool enable_overshoot_emergency_;
  bool enable_slope_compensation_;

  // timer callback
  double control_rate_;

  // closest waypoint
  double closest_dist_thr_;
  double closest_angle_thr_;

  // stop state
  double stop_state_vel_;
  double stop_state_acc_;
  double stop_state_entry_ego_speed_;
  double stop_state_entry_target_speed_;

  // delay compensation
  double delay_compensation_time_;

  // emergency stop
  double emergency_stop_acc_;
  double emergency_stop_jerk_;
  double emergency_overshoot_dist_;

  // smooth stop
  struct SmoothStopParam
  {
    double exit_ego_speed;
    double entry_ego_speed;
    double exit_target_speed;
    double entry_target_speed;
    double weak_brake_time;
    double weak_brake_acc;
    double increasing_brake_gradient;
    double increasing_brake_time;
    double stop_brake_time;
    double stop_brake_acc;
    double stop_dist_;
  } smooth_stop_param_;

  // acceleration limit
  double max_acc_;
  double min_acc_;

  // jerk limit
  double max_jerk_;
  double min_jerk_;

  // slope compensation
  double max_pitch_rad_;
  double min_pitch_rad_;

  // velocity feedback
  double current_vel_threshold_pid_integrate_;

  // controller mode (0: init check, 1: PID, 2: Stop, 3: Smooth stop, 4: Emergency stop, 5: Error)
  enum class ControlMode {
    INIT = 0,
    PID_CONTROL = 1,
    STOPPED = 2,
    SMOOTH_STOP = 3,
    EMERGENCY_STOP = 4,
    ERROR = 5,
  };
  ControlMode controller_mode_;

  // variables
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr_;
  std::shared_ptr<geometry_msgs::TwistStamped> current_vel_ptr_;
  std::shared_ptr<autoware_planning_msgs::Trajectory> trajectory_ptr_;

  // calculate dt
  std::shared_ptr<ros::Time> prev_control_time_;

  // shift mode
  enum Shift {
    Forward = 0,
    Reverse,
  } prev_shift_;

  // smooth stop
  bool is_smooth_stop_;
  bool is_emergency_stop_;
  std::shared_ptr<ros::Time> start_time_smooth_stop_;

  // diff limit
  double prev_acc_cmd_;
  double prev_vel_cmd_;

  // slope compensation
  Lpf1d lpf_pitch_;

  // velocity feedback
  PIDController pid_vel_;
  Lpf1d lpf_vel_error_;

  // methods
  void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg);
  void callbackTrajectory(const autoware_planning_msgs::TrajectoryConstPtr & msg);
  void callbackTimerControl(const ros::TimerEvent & event);

  bool updateCurrentPose(const double timeout_sec);
  bool getCurretPoseFromTF(const double timeout_sec, geometry_msgs::PoseStamped & ps);

  double getPitch(const geometry_msgs::Quaternion & quaternion) const;
  double getDt();
  enum Shift getCurrentShift(const double target_velocity) const;

  /* check condition */
  bool checkIsStopped(double current_vel, double target_vel) const;
  bool checkSmoothStop(const int closest, const double target_vel) const;
  bool checkEmergency(int closest, double target_vel) const;

  /* reset flags */
  void resetHandling(ControlMode control_mode);
  void resetEmergencyStop();
  void resetSmoothStop();

  /* calc control command */
  CtrlCmd calcCtrlCmd();
  void publishCtrlCmd(const double vel, const double acc);
  double calcInterpolatedTargetVelocity(
    const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::PoseStamped & curr_pose,
    const double current_vel, const int closest) const;
  double calcSmoothStopAcc();
  double calcFilteredAcc(
    const double raw_acc, const double pitch, const double dt, const Shift shift) const;
  double applyVelocityFeedback(
    const double target_acc, const double target_vel, const double dt, const double current_vel);
  double applyLimitFilter(const double input_val, const double max_val, const double min_val) const;
  double applyRateFilter(
    const double input_val, const double prev_val, const double dt, const double max_val,
    const double min_val) const;
  double applySlopeCompensation(const double acc, const double pitch, const Shift shift) const;
  double calcStopDistance(
    const autoware_planning_msgs::Trajectory & trajectory, const int closest) const;

  /* Debug */
  mutable std_msgs::Float32MultiArray debug_values_;
  enum DBGVAL {
    DT = 0,
    CURR_V = 1,
    TARGET_V = 2,
    TARGET_ACC = 3,
    CLOSEST_V = 4,
    CLOSEST_ACC = 5,
    SHIFT = 6,
    PITCH_LPFED_RAD = 7,
    PITCH_RAW_RAD = 8,
    PITCH_LPFED_DEG = 9,
    PITCH_RAW_DEG = 10,
    ERROR_V = 11,
    ERROR_V_FILTERED = 12,
    CTRL_MODE = 13,
    ACCCMD_PID_APPLIED = 14,
    ACCCMD_ACC_LIMITED = 15,
    ACCCMD_JERK_LIMITED = 16,
    ACCCMD_SLOPE_APPLIED = 17,
    ACCCMD_PUBLISHED = 18,
    ACCCMD_FB_P_CONTRIBUTION = 19,
    ACCCMD_FB_I_CONTRIBUTION = 20,
    ACCCMD_FB_D_CONTRIBUTION = 21,
    FLAG_SMOOTH_STOP = 22,
    FLAG_EMERGENCY_STOP = 23,
  };
  static constexpr unsigned int num_debug_values_ = 24;

  void writeDebugValues(
    const double dt, const double current_velocity, const double target_vel,
    const double target_acc, const Shift shift, const double pitch,
    const int32_t closest_waypoint_index);
};

#endif
