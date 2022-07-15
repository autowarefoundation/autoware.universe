/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
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

#ifndef NONLINEAR_MPC_NODE__NONLINEAR_MPC_NODE_HPP_
#define NONLINEAR_MPC_NODE__NONLINEAR_MPC_NODE_HPP_

// Standard and other headers.
#include <eigen3/Eigen/StdVector>
#include <deque>
#include <functional>
#include <memory>
#include <string>

// Autoware Headers
#include "common/types.hpp"
#include "motion_common/motion_common.hpp"
#include "motion_common/trajectory_common.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_vehicle_msgs/msg/nonlinear_mpc_performance_report.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "nonlinear_mpc_node/nonlinear_mpc_node_visualization.hpp"
#include <vehicle_info_util/vehicle_info_util.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "autoware_auto_vehicle_msgs/msg/delay_compensation_refs.hpp"
#include "autoware_auto_vehicle_msgs/msg/controller_error_report.hpp"
#include "utils_act/act_utils.hpp"
#include "utils_act/act_utils_eigen.hpp"

// ROS headers
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// Transform related.
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Autoware headers.
#include "common/types.hpp"
#include "helper_functions/angle_utils.hpp"
#include "motion_common/motion_common.hpp"
#include "motion_common/trajectory_common.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_vehicle_msgs/msg/nonlinear_mpc_performance_report.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

// NMPC package headers.
#include "nonlinear_mpc_core/nmpc_core.hpp"
#include "nonlinear_mpc_core/nmpc_kalman_filter.hpp"
#include "nonlinear_mpc_core/nmpc_speed_tracking_input_observer.hpp"
#include "nonlinear_mpc_node_visualization.hpp"
#include "splines/bspline_interpolator_templated.hpp"
#include "splines/bsplines_smoother.hpp"
#include "utils/nmpc_utils.hpp"
#include "utils/nmpc_utils_eigen.hpp"
#include "nonlinear_mpc_node/nonlinear_mpc_state_machine.h"

namespace ns_mpc_nonlinear
{
using VelocityMsg = nav_msgs::msg::Odometry;
using TrajectoryMsg = autoware_auto_planning_msgs::msg::Trajectory;
using ControlCmdMsg = autoware_auto_control_msgs::msg::AckermannControlCommand;
using SteeringMeasuredMsg = autoware_auto_vehicle_msgs::msg::SteeringReport;
using NonlinearMPCPerformanceMsg = autoware_auto_vehicle_msgs::msg::NonlinearMPCPerformanceReport;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using namespace std::chrono_literals;

using autoware_auto_vehicle_msgs::msg::DelayCompensationRefs;
using ErrorReportMsg = autoware_auto_vehicle_msgs::msg::ControllerErrorReport;
auto constexpr
  EPS = std::numeric_limits<double>::epsilon();

struct DebugData
{
  geometry_msgs::msg::Pose current_closest_pose{};

};

template<typename T>
void update_param(const std::vector<rclcpp::Parameter> &parameters, const std::string &name, T &value)
{
  auto it = std::find_if(parameters.cbegin(), parameters.cend(),
                         [&name](const rclcpp::Parameter &parameter)
                         { return parameter.get_name() == name; });
  if (it != parameters.cend())
  {
    value = static_cast<T>(it->template get_value<T>());
  }
}

class NonlinearMPCNode : public rclcpp::Node
{
 public:

  /**
   * @brief Constructor
   */
  explicit NonlinearMPCNode(const rclcpp::NodeOptions &node_options);

  /**
   * @brief Destructor
   */
  ~NonlinearMPCNode() override;

  // Type definitions.
  // Smoother spline type definition.
  /**
   * @brief Takes a fixed size (in rows) raw trajectory matrix --> smoothes out to fixed size reference trajectory.
   *
   * */
  using bspline_type_t = ns_splines::BSplineInterpolatorTemplated<ns_splines::MPC_MAP_SMOOTHER_IN,
                                                                  ns_splines::MPC_MAP_SMOOTHER_OUT>;

  /**
   *  @brief Public curvature interpolator, updated by the node, used all over the modules.
   *
   * */
  ns_splines::InterpolatingSplinePCG interpolator_curvature_pws;  // pws stands for point-wise

 private:
  // Publishers and subscribers.
  // PUBLISHERS.
  //<-@brief publishes control command.
  rclcpp::Publisher<ControlCmdMsg>::SharedPtr pub_control_cmd_{nullptr};

  // Closest point marker publisher.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_closest_point_debug_marker_{nullptr};

  // Publish the predicted trajectory.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_predicted_traj_{nullptr};

  // <-@brief topic publisher for predicted trajectory
  rclcpp::Publisher<TrajectoryMsg>::SharedPtr pub_predicted_traj_{nullptr};

  // <-@brief publishes the nonlinear mpc performance variables.
  rclcpp::Publisher<NonlinearMPCPerformanceMsg>::SharedPtr pub_nmpc_performance_{nullptr};

  // <-@brief publishes the nonlinear mpc error variables.
  rclcpp::Publisher<ErrorReportMsg>::SharedPtr pub_nmpc_error_report_{nullptr};

  // SUBSCRIBERS.
  // subscribe to the trajectory.
  rclcpp::Subscription<TrajectoryMsg>::SharedPtr sub_trajectory_{nullptr};

  // subscribe to the longitudinal velocity.
  rclcpp::Subscription<VelocityMsg>::SharedPtr sub_velocity_{nullptr};

  // subscribe to the measured vehicle steering.
  rclcpp::Subscription<SteeringMeasuredMsg>::SharedPtr sub_vehicle_steering_;

  //!< @brief subscription for current communication delay compensator messages.
  rclcpp::Subscription<DelayCompensationRefs>::SharedPtr sub_com_delay_;

  // subscribe to the measured vehicle steering.
  tf2::BufferCore tf_buffer_{tf2::BUFFER_CORE_DEFAULT_CACHE_TIME};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  //!<- @brief timer to update after a given interval.
  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  // Node members.
  double wheel_base_{};
  ns_data::ParamsNMPCNode params_node_{};
  std::unique_ptr<ns_nmpc_interface::NonlinearMPCController> nonlinear_mpc_controller_ptr_;

  /**
   *  @brief Bspline interpolators, takes a fixed size trajectory EigenMatrix [x, y] and yields a interpolated smooth
   *  [x, y] matrix. These smoothed coordinates are used to interpolate their first and second derivatives rdot(x, y),
   *  rddot(x,y). We compute a curvature vector using these derivatives.
   * */
  std::unique_ptr<bspline_type_t> bspline_interpolator_ptr_;  // smoothing interpolator.
  // std::unique_ptr<ns_splines::BSplineSmoother> bspline_interpolator_ptr_{nullptr};

  /**
   * @brief Kalman filter to predict initial states.
   * */
  ns_filters::KalmanUnscentedSQRT kalman_filter_{};

  /**
   * @brief Finite State Machine for tracking vehicle motion states.
   * */
  ns_states::VehicleMotionFSM vehicle_motion_fsm_{};

  // Pointers to the received messages.
  size_t current_trajectory_size_{};
  TrajectoryMsg::SharedPtr current_trajectory_ptr_{nullptr};

  // < @brief measured steering.
  VelocityMsg::SharedPtr current_velocity_ptr_{nullptr};

  // !<=@brief measured steering.
  SteeringMeasuredMsg::SharedPtr current_steering_ptr_{nullptr};

  //!< @brief current communication delay message pointer.
  DelayCompensationRefs::SharedPtr current_comm_delay_ptr_{nullptr};

  // !<-@brief measured pose.
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_ptr_{nullptr};
  std::unique_ptr<geometry_msgs::msg::PoseStamped> current_COG_pose_ptr_{nullptr};

  /**
   * @brief exact location of the vehicle projection on the current trajectory.
   * */
  std::unique_ptr<TrajectoryPoint> current_interpolated_traj_point_ptr_{nullptr};

  /**
   * @brief current error msg.
   * */
  ErrorReportMsg current_error_report_{};

  // Computed trajectory variables.
  // !<-@brief current distance on trajectory at vehicle position projection point.
  double current_s0_{};

  // !<-@brief current distance on trajectory at vehicle position projection point.
  double current_predicted_s0_{};
  double current_t0_{};  // !<-@brief current relative time on trajectory at vehicle pos. proj.

  double average_mpc_solve_time_{0.015};  // milliseconds.

  // Current and predicted curvatures
  double current_curvature_k0_{};  // !<-@brief current curvature value.
  double current_predicted_curvature_p0_{};
  double current_feedforward_steering_{};

  /** @brief the prev waypoint index, vehicle just has left behind. */
  std::unique_ptr<size_t> idx_prev_wp_ptr_{nullptr};

  /** @brief the next waypoint index, vehicle heading to*/
  std::unique_ptr<size_t> idx_next_wp_ptr_{nullptr};

  // BUFFERS. Use a single buffer.
  std::deque<std::array<double, 4>> inputs_buffer_common_;  // !< @brief  [ax, vx, steering_rate, steering]

  // Data members.
  bool is_ctrl_cmd_prev_initialized_{false};  // !< @brief flag of ctrl_cmd_prev_ initialization

  // MPC booleans
  bool is_mpc_solved_{false};
  bool is_data_ready_{false};

  ControlCmdMsg ctrl_cmd_prev_{};  // !< @brief previous control command

  /**
  * @brief  nonlinear MPC performance variables messages to be published.
  *         float64 steering_angle_input
  *         float64 steering_angle_velocity_input
  *         float64 steering_angle_measured
  *         float64 acceleration_input
  *         float64 nmpc_lateral_error
  *         float64 nmpc_yaw_error
  *         float64 nmpc_curvature
  *         float64 lateral_error_ukf
  *         float64 yaw_error_ukf
  *         float64 steering_angle_ukf
  *         float64 long_velocity_ukf
  *         float64 long_velocity_target
  *         float64 long_velocity_input
  *         float64 long_velocity_measured
  *         float64 avg_nmpc_computation_time
  *         float64 yaw_angle_measured
  *         float64 yaw_angle_target
  *         float64 yaw_angle_traj
  *         float64 lateral_acceleration
  *         int8 fsm_state
  *         float64 distance_to_stop_point
  */
  NonlinearMPCPerformanceMsg nmpc_performance_vars_{};

  // Vehicle motion finite state
  ns_states::motionStateEnums current_fsm_state_{
    ns_states::motionStateEnums::isAtCompleteStop};  // state machine.

  /**
   * @brief PLACEHOLDERS where the computed variables are updated and hold.
   * */
  Model::state_vector_t x0_initial_states_;  // !<-@brief measured initial states placeholder.
  Model::state_vector_t x0_previous_initial_states_;         // !<-@brief measured previous initial states placeholder.
  Model::state_vector_t x0_predicted_;   // !<-@brief predicted initial state for the input delay.
  Model::state_vector_t x0_kalman_est_;  // !<-@brief Kalman filter estimate of the initial states.
  Model::input_vector_t u0_kalman_;      // !<-@brief The first control vector in the queue.
  Model::param_vector_t current_model_params_;  // !<-@brief [curvature_0, target_speed_0]
  Model::param_vector_t predicted_model_params_;

  // Control signals are stored in the following placeholders.
  // !<-@brief These signals are computed in the model space, vehicle previous
  // controls are stored in the delay queue.
  Model::input_vector_t u_solution_;  // !<-@brief [vx input, steering input] controller solution
  Model::input_vector_t u_previous_solution_;

  // Node callback methods.
  /**
   * @brief computes NMPC controls and publishes.
   * */
  void onTimer();

  // Node methods.
  void loadNodeParameters();  // load the parameters used in the node.

  void loadVehicleParameters(ns_models::ParamsVehicle &params_vehicle);  // load vehicle model params.

  // Load NMPCCore, LPV parameters and OSQP class parameters.
  void loadNMPCoreParameters(ns_data::data_nmpc_core_type_t &data_nmpc_core,
                             ns_data::param_lpv_type_t &params_lpv,
                             ns_data::ParamsOptimization &params_optimization);

  void loadFilterParameters(ns_data::ParamsFilters &params_filters);

  /**
   *   @brief We use state and control scaling for within the optimization algorithms.
   *   states : [Xw, Yw, psi, s, e_y, e_yaw, vx, delta]
   *
   *   Linear scaling equations.
   *   x = a * xhat + b
   *   u = a* uhat + b
   *
   *   Simple algebraic equation solution gives ax, au, bx, bu
   *   a = (xmax-xmin) / (xmaxhat - xminhat) and b = (xmax, xmin) - a(xmaxhat, xminhat).
   *
   *  ax, au are put on the diagonals of the scaling matrices. bx, bu are the centering vector entries.
   * */
  static void computeScalingMatrices(ns_data::ParamsOptimization &params);

  /**
   * @brief set current_trajectory_ with received message
   */
  void onTrajectory(TrajectoryMsg::SharedPtr msg);

  /**
   * @brief set current measured longitudinal speed.
   */
  void onVelocity(VelocityMsg::SharedPtr msg);

  /**
   * @brief received communication time delay compensation message.
   */
  void onCommDelayCompensation(const DelayCompensationRefs::SharedPtr msg);

  /**
  * @brief set current measured steering with received message
  */
  void onSteeringMeasured(SteeringMeasuredMsg::SharedPtr msg);

  /**
   * @brief Dynamic update of the parameters.
   * */
  OnSetParametersCallbackHandle::SharedPtr is_parameters_set_res_;

  rcl_interfaces::msg::SetParametersResult onParameterUpdate(const std::vector<rclcpp::Parameter> &parameters);

  void updateCurrentPose();

  bool isDataReady();

  void initializeEigenPlaceholders();

  /**
   *  @brief finds the interval on the path on which the vehicle's position is projected.
   */
  void findClosestPrevWayPointIdx();

  void computeClosestPointOnTraj();

  void setCurrentCOGPose(geometry_msgs::msg::PoseStamped const &ps);

  /**
   * @brief calculates the distance to the vehicle states in which the speed is less than a threshold.
   * */
  double calcStopDistance(const size_t &prev_waypoint_index) const;

  // Gets the distance to the stopping point, ego speed and the speed at the next traj. point.
  std::array<double, 3> getDistanceEgoTargetSpeeds() const;

  /** @brief Get average MPC computation time in seconds. */
  void getAverageMPCcomputeTime(double &avg_compute_time_in_sec) const;

  /**
   * @brief compute the initial error and set the initial states and controls.
   *  - Locates the vehicle w.r.t the path and finds a reference pose,
   *  - Computes the error states and initializes the starting state x0.
   *      where x0 = [x, y, psi, s, e_y, e_yaw, v, delta] and the controls
   *      [vx input m/s, steering input - rad].
   *  - Starting from the current pose, computes an initial trajectory for the states and controls.
   *
   *  Note: x0 in this method is in the Autoware space.
   *  */
  void updateInitialStatesAndControls_fromMeasurements();

  /**
   * @brief  Computes the error states [e_y, e_psi].
   * */
  std::array<double, 2> computeErrorStates();

  /**
   * @brief predict the initial states using the vx control input buffer.
   * @param [in-out] xd0 the integrated initial state iteratively.
   * */
  void predictDelayedInitialStateBy_MPCPredicted_Inputs(Model::state_vector_t &x0_predicted);

  /**
   * @brief predict the initial states using the vx received from the trajectory planner.
   * @param [in-out] x_predicted_ the integrated initial state iteratively.
   * */
  void predictDelayedInitialStateBy_TrajPlanner_Speeds(Model::state_vector_t &xd0);

  void estimateDisturbanceInput();

  /**
   * @brief Set the timer object.
   **/
  void initTimer(double period_s);  //!< @brief Set the timer object.

  bool isValidTrajectory(const TrajectoryMsg &msg_traj) const;

  /**
   * f resamples the current trajectory with a varying size into a fixed-sized trajectories and store them in an associated raw trajectory data class.
   * @return true
   * @return false
   */
  bool resampleRawTrajectoriesToaFixedSize();

  /**
   * @brief creates a matrix of a fixed number of rows for [s, x, y, z] coordinates given the raw trajectories.
   * @param mpc_traj_raw raw trajectory container created when a planning trajectory is received.
   * @param fixed_map_ref_sxyz Eigen matrix that stores the interpolated coordinates.
   * */
  static bool makeFixedSizeMat_sxyz(ns_data::MPCdataTrajectoryVectors const &mpc_traj_raw,
                                    map_matrix_in_t &fixed_map_ref_sxyz);

  /**
   * @brief given a fixed size [s, x, y, z] trajectory, create the other references. InterpolateImplicitCoordinates [s, x, y] for down-sampling.
   * We do not use these references except the arc_length-curvature table.
   * */
  bool createSmoothTrajectoriesWithCurvature(ns_data::MPCdataTrajectoryVectors const &mpc_traj_raw,
                                             map_matrix_in_t const &fixed_map_ref_sxyz);

  static ControlCmdMsg createControlCommand(double const &ax,
                                            double const &vx,
                                            double const &steering_rate,
                                            double const &steering_val);

  /**
   * @brief publish control command as autoware_msgs/ControlCommand type
   * @param [in] cmd published control command
   */
  void publishControlCommands(ControlCmdMsg &control_cmd) const;

  void publishControlsAndUpdateVars(ControlCmdMsg &ctrl_cmd);

  void publishPerformanceVariables(const ControlCmdMsg &control_cmd);

  void publishPredictedTrajectories(std::string const &ns, std::string const &frame_id) const;
  void publishClosestPointMarker(std::string const &ns) const;

  void setErrorReport(Model::state_vector_t const &x);
  void publishErrorReport(ErrorReportMsg &error_rpt_msg);

  visualization_msgs::msg::MarkerArray createPredictedTrajectoryMarkers(std::string const &ns,
                                                                        std::string const &frame_id,
                                                                        std::array<double, 2> xy0,
                                                                        Model::trajectory_data_t const &td) const;

  /**
   * @brief get initial command
   */
  ControlCmdMsg getInitialControlCommand() const;

  /**
   * @brief get stop command
   */
  static ControlCmdMsg getStopControlCommand();

  // Debug Parameters
  mutable DebugData debug_data_{};

  // Friend class for testing.
  friend class NMPCTestSuiteMembers;

  bool is_testing_{false};  // !<-@brief only used for testing purpose in loading parameters.
};
} // namespace ns_mpc_nonlinear
#endif  // NONLINEAR_MPC_NODE__NONLINEAR_MPC_NODE_HPP_
