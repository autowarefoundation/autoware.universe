// Copyright 2018-2021 The Autoware Foundation
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

#ifndef MPC_LATERAL_CONTROLLER__MPC_HPP_
#define MPC_LATERAL_CONTROLLER__MPC_HPP_

#include "mpc_lateral_controller/interpolate.hpp"
#include "mpc_lateral_controller/lowpass_filter.hpp"
#include "mpc_lateral_controller/mpc_trajectory.hpp"
#include "mpc_lateral_controller/mpc_utils.hpp"
#include "mpc_lateral_controller/qp_solver/qp_solver_osqp.hpp"
#include "mpc_lateral_controller/qp_solver/qp_solver_unconstr_fast.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_dynamics.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"
#include "osqp_interface/osqp_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{

using autoware_auto_control_msgs::msg::AckermannLateralCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::Pose;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct MPCWeight
{
  double lat_error;                   // lateral error weight
  double heading_error;               // heading error weight
  double heading_error_squared_vel;   // heading error * velocity weight
  double terminal_lat_error;          // terminal lateral error weight
  double terminal_heading_error;      // terminal heading error weight
  double steering_input;              // steering error weight
  double steering_input_squared_vel;  // steering error * velocity weight
  double lat_jerk;                    // lateral jerk weight
  double steer_rate;                  // steering rate weight
  double steer_acc;                   // steering angle acceleration weight
};

struct MPCParam
{
  int prediction_horizon;         // prediction horizon step
  double prediction_dt;           // prediction horizon sampling time
  double zero_ff_steer_deg;       // threshold that feed-forward angle becomes zero
  double input_delay;             // delay time for steering input to be compensated
  double acceleration_limit;      // for trajectory velocity calculation
  double velocity_time_constant;  // for trajectory velocity calculation
  double min_prediction_length;   // minimum prediction dist for low velocity
  double steer_tau;               // time constant for steer model

  MPCWeight nominal_weight;        // parameters for the MPC weight
  MPCWeight low_curvature_weight;  // parameters for the MPC weight used in low curvature path

  double low_curvature_thresh_curvature;  // threshold of curvature to use "low curvature" parameter
};
/**
 * MPC problem data
 */
struct MPCData
{
  size_t nearest_idx{};
  double nearest_time{};
  Pose nearest_pose{};
  double steer{};
  double predicted_steer{};
  double lateral_err{};
  double yaw_err{};

  MPCData() = default;
};
/**
 * Matrices used for MPC optimization
 */
struct MPCMatrix
{
  MatrixXd Aex;
  MatrixXd Bex;
  MatrixXd Wex;
  MatrixXd Cex;
  MatrixXd Qex;
  MatrixXd R1ex;
  MatrixXd R2ex;
  MatrixXd Uref_ex;

  MPCMatrix() = default;
};
/**
 * MPC-based waypoints follower class
 * @brief calculate control command to follow reference waypoints
 */
class MPC
{
private:
  rclcpp::Logger m_logger = rclcpp::get_logger("mpc_logger");  // ROS logger used for debug logging
  rclcpp::Clock::SharedPtr m_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);  // ROS clock

  std::string m_vehicle_model_type;                            // vehicle model type for MPC
  std::shared_ptr<VehicleModelInterface> m_vehicle_model_ptr;  // vehicle model for MPC
  std::shared_ptr<QPSolverInterface> m_qpsolver_ptr;           // qp solver for MPC

  Butterworth2dFilter m_lpf_steering_cmd;   // lowpass filter for steering command
  Butterworth2dFilter m_lpf_lateral_error;  // lowpass filter for lateral error
  Butterworth2dFilter m_lpf_yaw_error;      // lowpass filter for heading error

  double m_raw_steer_cmd_pprev = 0.0;  // raw output computed two iterations ago
  double m_lateral_error_prev = 0.0;   // previous lateral error for derivative
  double m_yaw_error_prev = 0.0;       // previous lateral error for derivative

  std::shared_ptr<double> m_steer_prediction_prev;              // previous predicted steering
  rclcpp::Time m_time_prev = rclcpp::Time(0, 0, RCL_ROS_TIME);  // previous computation time

  bool m_is_forward_shift = true;                       // shift is forward or not.
  std::vector<AckermannLateralCommand> m_ctrl_cmd_vec;  // buffer of sent command

  double m_min_prediction_length = 5.0;  // minimum prediction distance

  /**
   * @brief get variables for mpc calculation
   */
  std::pair<bool, MPCData> getData(
    const MPCTrajectory & traj, const SteeringReport & current_steer, const Pose & current_pose);

  /**
   * @brief calculate predicted steering
   */
  double calcSteerPrediction();

  /**
   * @brief get the sum of all steering commands over the given time range
   */
  double getSteerCmdSum(
    const rclcpp::Time & t_start, const rclcpp::Time & t_end, const double time_constant) const;

  /**
   * @brief set the reference trajectory to follow
   */
  void storeSteerCmd(const double steer);

  /**
   * @brief set initial condition for mpc
   * @param [in] data mpc data
   */
  VectorXd getInitialState(const MPCData & data);

  /**
   * @brief update status for delay compensation
   * @param [in] traj MPCTrajectory to follow
   * @param [in] start_time start time for update
   * @param [out] x updated state at delayed_time
   */
  std::pair<bool, VectorXd> updateStateForDelayCompensation(
    const MPCTrajectory & traj, const double & start_time, const VectorXd x0_orig);

  /**
   * @brief generate MPC matrix with trajectory and vehicle model
   * @param [in] reference_trajectory used for linearization around reference trajectory
   */
  MPCMatrix generateMPCMatrix(
    const MPCTrajectory & reference_trajectory, const double prediction_dt);

  /**
   * @brief generate MPC matrix with trajectory and vehicle model
   * @param [in] mpc_matrix parameters matrix to use for optimization
   * @param [in] x0 initial state vector
   * @param [in] prediction_dt prediction delta time
   * @param [in] trajectory mpc reference trajectory
   * @param [in] current_velocity current ego velocity
   * @param [out] Uex optimized input vector
   */
  std::pair<bool, VectorXd> executeOptimization(
    const MPCMatrix & mpc_matrix, const VectorXd & x0, const double prediction_dt,
    const MPCTrajectory & trajectory, const double current_velocity);

  /**
   * @brief resample trajectory with mpc resampling time
   */
  std::pair<bool, MPCTrajectory> resampleMPCTrajectoryByTime(
    const double start_time, const double prediction_dt, const MPCTrajectory & input) const;

  /**
   * @brief apply velocity dynamics filter with v0 from closest index
   */
  MPCTrajectory applyVelocityDynamicsFilter(
    const MPCTrajectory & trajectory, const Pose & current_pose, const double v0) const;

  /**
   * @brief get prediction delta time of mpc.
   * If trajectory length is shorter than min_prediction length, adjust delta time.
   */
  double getPredictionDeltaTime(
    const double start_time, const MPCTrajectory & input, const Pose & current_pose) const;

  /**
   * @brief add weights related to lateral_jerk, steering_rate, steering_acc into R
   */
  void addSteerWeightR(const double prediction_dt, MatrixXd * R) const;

  /**
   * @brief add weights related to lateral_jerk, steering_rate, steering_acc into f
   */
  void addSteerWeightF(const double prediction_dt, MatrixXd * f) const;

  /**
   * @brief calculate desired steering rate.
   */
  double calcDesiredSteeringRate(
    const MPCMatrix & m, const MatrixXd & x0, const MatrixXd & Uex, const double u_filtered,
    const float current_steer, const double predict_dt) const;

  /**
   * @brief check if the matrix has invalid value
   */
  bool isValid(const MPCMatrix & m) const;

  /**
   * @brief return the weight for the MPC optimization
   */
  inline MPCWeight getWeight(const double curvature)
  {
    return std::fabs(curvature) < m_param.low_curvature_thresh_curvature
             ? m_param.low_curvature_weight
             : m_param.nominal_weight;
  }

  /**
   * @brief calculate predicted trajectory for the ego vehicle based on the mpc result
   */
  Trajectory calcPredictedTrajectory(
    const MPCTrajectory & mpc_resampled_ref_traj, const MPCMatrix & mpc_matrix,
    const VectorXd & x0_delayed, const VectorXd & Uex) const;

  /**
   * @brief calculate diagnostic data for the debugging purpose
   */
  Float32MultiArrayStamped generateDiagData(
    const MPCTrajectory & reference_trajectory, const MPCData & mpc_data,
    const MPCMatrix & mpc_matrix, const AckermannLateralCommand & ctrl_cmd, const VectorXd & Uex,
    const Pose & current_pose, const double current_velocity) const;

  /**
   * @brief logging with warn and return false
   */
  inline bool fail_warn_throttle(const std::string & msg, const int duration_ms = 1000) const
  {
    RCLCPP_WARN_THROTTLE(m_logger, *m_clock, duration_ms, "%s", msg.c_str());
    return false;
  }

  /**
   * @brief logging with warn
   */
  inline void warn_throttle(const std::string & msg, const int duration_ms = 1000) const
  {
    RCLCPP_WARN_THROTTLE(m_logger, *m_clock, duration_ms, "%s", msg.c_str());
  }

public:
  MPCTrajectory m_ref_traj;           // reference trajectory to be followed
  MPCParam m_param;                   // MPC design parameter
  std::deque<double> m_input_buffer;  // mpc_output buffer for delay time compensation
  double m_raw_steer_cmd_prev = 0.0;  // mpc raw output in previous period

  /* parameters for control*/
  double m_admissible_position_error;  // use stop cmd when lateral error exceeds this [m]
  double m_admissible_yaw_error_rad;   // use stop cmd when yaw error exceeds this [rad]
  double m_steer_lim;                  // steering command limit [rad]
  double m_ctrl_period;                // control frequency [s]

  //!< @brief steering rate limit list depending on curvature [/m], [rad/s]
  std::vector<std::pair<double, double>> m_steer_rate_lim_map_by_curvature{};

  //!< @brief steering rate limit list depending on velocity [m/s], [rad/s]
  std::vector<std::pair<double, double>> m_steer_rate_lim_map_by_velocity{};

  /* parameters for path smoothing */
  bool m_use_steer_prediction;        // flag to use predicted steer, not measured steer.
  double ego_nearest_dist_threshold;  // parameters for nearest index search
  double ego_nearest_yaw_threshold;   // parameters for nearest index search

  /**
   * @brief constructor
   */
  MPC() = default;

  /**
   * @brief calculate control command by MPC algorithm
   * @param [in] current_steer current steering of the vehicle
   * @param [in] current_velocity current velocity of the vehicle [m/s]
   * @param [in] current_pose current pose of the vehicle
   * @param [out] ctrl_cmd control command calculated with mpc algorithm
   * @param [out] predicted_traj predicted MPC trajectory
   * @param [out] diagnostic diagnostic msg to be filled-out
   */
  bool calculateMPC(
    const SteeringReport & current_steer, const double current_velocity, const Pose & current_pose,
    AckermannLateralCommand & ctrl_cmd, Trajectory & predicted_traj,
    Float32MultiArrayStamped & diagnostic);

  /**
   * @brief set the reference trajectory to follow
   */
  void setReferenceTrajectory(
    const Trajectory & trajectory_msg, const double traj_resample_dist,
    const bool enable_path_smoothing, const int path_filter_moving_ave_num,
    const int curvature_smoothing_num_traj, const int curvature_smoothing_num_ref_steer,
    const bool extend_trajectory_for_end_yaw_control);

  /**
   * @brief reset previous result of MPC
   */
  void resetPrevResult(const SteeringReport & current_steer);

  /**
   * @brief set the vehicle model of this MPC
   */
  inline void setVehicleModel(
    std::shared_ptr<VehicleModelInterface> vehicle_model_ptr,
    const std::string & vehicle_model_type)
  {
    m_vehicle_model_ptr = vehicle_model_ptr;
    m_vehicle_model_type = vehicle_model_type;
  }

  /**
   * @brief set the QP solver of this MPC
   */
  inline void setQPSolver(std::shared_ptr<QPSolverInterface> qpsolver_ptr)
  {
    m_qpsolver_ptr = qpsolver_ptr;
  }

  /**
   * @brief initialize low pass filters
   */
  inline void initializeLowPassFilters(
    const double steering_lpf_cutoff_hz, const double error_deriv_lpf_cutoff_hz)
  {
    m_lpf_steering_cmd.initialize(m_ctrl_period, steering_lpf_cutoff_hz);
    m_lpf_lateral_error.initialize(m_ctrl_period, error_deriv_lpf_cutoff_hz);
    m_lpf_yaw_error.initialize(m_ctrl_period, error_deriv_lpf_cutoff_hz);
  }

  /**
   * @brief return true if the vehicle model of this MPC is set
   */
  inline bool hasVehicleModel() const { return m_vehicle_model_ptr != nullptr; }

  /**
   * @brief return true if the QP solver of this MPC is set
   */
  inline bool hasQPSolver() const { return m_qpsolver_ptr != nullptr; }

  /**
   * @brief set the RCLCPP logger to use for logging
   */
  inline void setLogger(rclcpp::Logger logger) { m_logger = logger; }

  /**
   * @brief set the RCLCPP clock to use for time keeping
   */
  inline void setClock(rclcpp::Clock::SharedPtr clock) { m_clock = clock; }
};  // class MPC
}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // MPC_LATERAL_CONTROLLER__MPC_HPP_
