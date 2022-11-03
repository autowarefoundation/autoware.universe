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

#include "nonlinear_mpc_node/nonlinear_mpc_node.hpp"

#include "nonlinear_mpc_core/nmpc_core.hpp"

#include <algorithm>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ns_mpc_nonlinear
{
// Constructor.
NonlinearMPCNode::NonlinearMPCNode(const rclcpp::NodeOptions &node_options)
  : Node("mpc_nonlinear", node_options)
{
  using std::placeholders::_1;

  // Initialize the publishers.
  pub_control_cmd_ = create_publisher<ControlCmdMsg>("~/output/control_cmd", 1);

  pub_predicted_traj_ = create_publisher<TrajectoryMsg>("~/output/predicted_trajectory", 1);

  // Debug publishers.
  pub_closest_point_debug_marker_ =
    create_publisher<visualization_msgs::msg::Marker>("debug/nmpc_closest_point_markers", 1);

  // Predicted trajectory and its markers.
  pub_debug_predicted_traj_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("debug/nmpc_predicted_trajectory", 1);

  //  Error reports
  pub_nmpc_error_report_ = create_publisher<ErrorReportMsg>("/nmpc_error_report", 1);

  // NMPC performance variables for visualizing.
  pub_nmpc_performance_ = create_publisher<NonlinearMPCPerformanceMsg>("~/debug/nmpc_vars", 1);

  // Initialize the subscribers.
  sub_trajectory_ = create_subscription<TrajectoryMsg>(
    "~/input/reference_trajectory", rclcpp::QoS{1},
    std::bind(&NonlinearMPCNode::onTrajectory, this, _1));

  sub_velocity_ = create_subscription<VelocityMsg>(
    "~/input/current_velocity", rclcpp::QoS{1}, std::bind(&NonlinearMPCNode::onVelocity, this, _1));

  sub_vehicle_steering_ = create_subscription<SteeringMeasuredMsg>(
    "~/input/current_steering", rclcpp::QoS{1},
    std::bind(&NonlinearMPCNode::onSteeringMeasured, this, _1));

  sub_com_delay_ = create_subscription<DelayCompensationRefs>(
    "~/input/current_com_delay", rclcpp::QoS{1},
    std::bind(&NonlinearMPCNode::onCommDelayCompensation, this, _1));

  // Dynamic Parameter Update.
  is_parameters_set_res_ =
    this->add_on_set_parameters_callback(std::bind(&NonlinearMPCNode::onParameterUpdate, this, _1));

  // Request wheel_base parameter.
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  wheel_base_ = vehicle_info.wheel_base_m;

  // Load the parameters.
  /**
   * @brief loads parameters used for the node operations.
   * @param is_testing is a boolean parameter used for gtests (true if testing).
   * Its aim is to eliminate other node dependency to load the wheel_base.
   */
  loadNodeParameters();  // read the parameters used by the node.

  // Load vehicle model parameters.
  ns_models::ParamsVehicle params_vehicle{};
  loadVehicleParameters(params_vehicle);

  // Create vehicle model and make it shared_ptr.
  auto vehicle_model_ptr = std::make_shared<Model>();
  vehicle_model_ptr->updateParameters(params_vehicle);

  if (!vehicle_model_ptr->IsInitialized())
  {
    vehicle_model_ptr->InitializeModel();
    // vehicle_model_ptr->testModel();
  }

  /**
   * @brief to check if the automatic differential modules are working properly.
   * */

  // vehicle_model_ptr->testModel();
  // Initialize NMPCcore, LPV and Optimization parameters.
  double const &&mpc_time_step_dt = declare_parameter("mpc_prediction_dt", 0.1);
  ns_data::data_nmpc_core_type_t data_nmpc_core(mpc_time_step_dt);
  data_nmpc_core.wheel_base = wheel_base_;

  ns_data::param_lpv_type_t params_lpv;
  ns_data::ParamsOptimization params_optimization;

  loadNMPCoreParameters(data_nmpc_core, params_lpv, params_optimization);

  // Compute state and control scaling variables.
  computeScalingMatrices(params_optimization);

  // Create NMPC core object.
  auto nonlinear_nmpc_core = ns_nmpc_interface::NonlinearMPCController(
    vehicle_model_ptr, data_nmpc_core, params_lpv, params_optimization);

  nonlinear_nmpc_core.setLoggerName(this->get_logger().get_name());

  nonlinear_mpc_controller_ptr_ =
    std::make_unique<ns_nmpc_interface::NonlinearMPCController>(nonlinear_nmpc_core);

  /**
   * @brief sets the BSpline interpolator. Choose the option compute_derivatives:true.
   * The knots ratio is the ratio of the number of knots to the total number of points.
   * bspline_interpolator_ptr_ = std::make_unique<spline_type_bspline>(0.3, true);
   */

  bspline_interpolator_ptr_ = std::make_unique<bspline_type_t>(0.5, true);
  // bspline_interpolator_ptr_ =
  // std::make_unique<ns_splines::BSplineSmoother>(ns_nmpc_interface::MPC_MAP_SMOOTHER_IN, 0.3);

  // Vehicle motion finite state machine.
  vehicle_motion_fsm_ = ns_states::VehicleMotionFSM(
    params_node_.stop_state_entry_ego_speed, params_node_.stop_state_entry_target_speed,
    params_node_.stop_state_keep_stopping_dist, params_node_.will_stop_state_dist);

  // Initialize the timer.
  initTimer(params_node_.control_period);

  /**
   * Initialize input buffer map.
   * */
  for (size_t k = 0; k < params_node_.input_delay_discrete_nsteps; ++k)
  {
    ControlCmdMsg cmd{};
    cmd.longitudinal.acceleration = 0.0;
    cmd.lateral.steering_tire_angle = 0.0;
    cmd.stamp = this->now() + rclcpp::Duration::from_seconds(params_node_.control_period * k);

    inputs_buffer_.emplace_back(cmd);
  }
}

// Destructor.
NonlinearMPCNode::~NonlinearMPCNode()
{
  ControlCmdMsg stop_cmd{};
  getStopControlCommand(stop_cmd);
  publishControlCommands(stop_cmd);
}

void NonlinearMPCNode::initTimer(double period_s)
{
  auto timer_callback = std::bind(&NonlinearMPCNode::onTimer, this);

  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());

  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

// Callbacks.
void NonlinearMPCNode::onTimer()
{
  // Update the current pose.
  updateCurrentPose();

  // Check Data Stream
  is_data_ready_ = isDataReady();
  if (!is_data_ready_)
  {
    // Warnings are generated in the isDataReady() method.
    // Send stop command to the vehicle.

    ControlCmdMsg stop_cmd{};
    getStopControlCommand(stop_cmd);
    publishControlsAndUpdateVars(stop_cmd);
    // publishPerformanceVariables(stop_cmd);
    return;
  }

  // If the previous control command is not assigned, assign.
  if (!is_control_cmd_prev_initialized_)
  {
    control_cmd_prev_ = getInitialControlCommand();
    is_control_cmd_prev_initialized_ = true;

    publishControlsAndUpdateVars(control_cmd_prev_);
    // publishPerformanceVariables(control_cmd_prev_);
    return;
  }

  /**
   * @brief MPC loop : THIS SCOPE is important to update and maintain the MPC_CORE object. The
   * operations are in an order.
   * */
  std::string const timer_mpc_step{"mpc_calc_time"};
  stop_watch_.tic(timer_mpc_step);


  // Find the index of the prev and next waypoint.
  current_error_report_ = ErrorReportMsg{};
  findClosestPrevWayPointIdx();
  computeClosestPointOnTraj();

  // Update the measured initial states.
  updateInitialStatesAndControls_fromMeasurements();

  // Vehicle Motion State Machine keep tracks of if vehicle is moving, stopping.
  auto const &&dist_v0_vnext = getDistanceEgoTargetSpeeds();

  // Set performance distance of performance variable for monitoring.
  nmpc_performance_vars_.distance_to_stop_point = dist_v0_vnext[0];
  vehicle_motion_fsm_.toggle(dist_v0_vnext);

  // vehicle_motion_fsm_.printCurrentStateMsg();
  // ns_utils::print("Distance, V0, Vnext ", dist_v0_vnext[0], dist_v0_vnext[1], dist_v0_vnext[2]);

  current_fsm_state_ = vehicle_motion_fsm_.getCurrentStateType();
  // ns_utils::print("Finite state machine state numbers : ",
  // ns_states::as_integer(current_fsm_state_));

  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    get_logger(), *get_clock(), (1000ms).count(), "\n[mpc_nonlinear] %s",
    std::string(vehicle_motion_fsm_.getFSMTypeReport()).c_str());

  //    if (current_fsm_state_ == ns_states::motionStateEnums::isAtComplete   Stop ||
  //        current_fsm_state_ == ns_states::motionStateEnums::isInEmergency)
  if (current_fsm_state_ == ns_states::motionStateEnums::isAtCompleteStop)
  {
    // kalman_filter_ptr_->reset();
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(), "\n[mpc_nonlinear] %s",
      std::string(vehicle_motion_fsm_.getFSMTypeReport()).c_str());

    ControlCmdMsg control_cmd{};
    getStopControlCommand(control_cmd);

    // Prevent reverse motion when stopped.
    if (current_velocity_ptr_->twist.twist.linear.x < EPS)
    {
      control_cmd.longitudinal.acceleration = 0.0;
    }

    publishControlsAndUpdateVars(control_cmd);

    inputs_buffer_.pop_front();

    control_cmd.stamp = this->now() + rclcpp::Duration::from_seconds(params_node_.input_delay_time);
    inputs_buffer_.emplace_back(control_cmd);

    return;
  }

  // Prepare the x0_predicted placeholder by fetching the initial states into it.
  x0_predicted_.setZero();
  nonlinear_mpc_controller_ptr_->getInitialState(x0_predicted_);

  // DEBUG
  // TODO: debug
  //  ns_utils::print("Initial states");
  //  ns_eigen_utils::printEigenMat(x0_predicted_);
  // end of DEBUG

  // If there is no input delay in the system, predict_initial_states is set to false automatically
  // in the constructor.

  if (params_node_.predict_initial_states)
  {
    /**
     * If the NMPC vx input is computed, we can use it to predict the initial states due to delayed
     * inputs.
     * */

    predictDelayedInitialStateBy_MPCPredicted_Inputs(x0_predicted_);
  }

  setErrorReport(x0_predicted_);

  /**
   * Publish the error before the controller computes the control signal based on this error for the
   * time-delay compensators.
   * */
  publishErrorReport(current_error_report_);

  // ns_utils::print("before using comm delay error refs...");
  if (params_node_.use_cdob && current_comm_delay_ptr_)
  {
    x0_predicted_(4) = current_comm_delay_ptr_->lateral_deviation_error_compensation_ref;
    x0_predicted_(5) = current_comm_delay_ptr_->heading_angle_error_compensation_ref;
    x0_predicted_(7) = current_comm_delay_ptr_->steering_compensation_ref;
    // ns_utils::print("NMPC using error references...");
  }

  // Update the initial state of the NMPCcore object.
  nonlinear_mpc_controller_ptr_->updateInitialStates_x0(x0_predicted_);

  /**
   * Target states are predicted based-on the NMPC input trajectory [vx input, steering input].
   * */

  nonlinear_mpc_controller_ptr_->updateRefTargetStatesByTimeInterpolation(average_mpc_solve_time_);

  if (!nonlinear_mpc_controller_ptr_->isInitialized())
  {
    auto const &use_linear_initialization = params_node_.use_linear_trajectory_initialization;

    // Compute the initial reference trajectories and discretisize the system.
    if (auto const &&is_initialized = nonlinear_mpc_controller_ptr_->initializeTrajectories(
        interpolator_curvature_pws, use_linear_initialization);
      !is_initialized)
    {
      // vehicle_motion_fsm_.setEmergencyFlag(true);

      RCLCPP_WARN_SKIPFIRST_THROTTLE(
        get_logger(), *get_clock(), (1000ms).count(),
        "[mpc_nonlinear] Couldn't initialize the LPV controller ... %s \n",
        std::string(vehicle_motion_fsm_.getFSMTypeReport()).c_str());

      // Publish previous control command.
      publishControlsAndUpdateVars(control_cmd_prev_);
      return;
    }

    // Get trajectories as a matrix and print for debugging purpose.
    // DEBUG
    // TODO {ali}: disable the debugs.
    //    auto const & traj_data = nonlinear_mpc_controller_ptr_->getCurrentTrajectoryData();
    //    auto && Xtemp = ns_eigen_utils::getTrajectory(traj_data.X);
    //    auto && Utemp = ns_eigen_utils::getTrajectory(traj_data.U);
    //
    //    ns_utils::print("\nComputed LPV trajectories : ");
    //    [x, y, psi, s, ey, epsi, vx, delta, vy]
    //    ns_eigen_utils::printEigenMat(Xtemp.transpose());
    //
    //    ns_utils::print("\nComputed LPV trajectories U : ");
    //    ns_eigen_utils::printEigenMat(Utemp.transpose());
    // end of DEBUG
  }

  /**
   * @brief
   * - Simulate the controls starting from the delayed initial state.
   * - Apply the predicted control to the vehicle model starting from predicted initial state and
   * replace the
   * - measured initial state with the predicted initial state.
   */

  /**
   * All inputs of the NMPC [vx, steering] are applied and the system is simulated.
   * */
  nonlinear_mpc_controller_ptr_->simulateControlSequenceByPredictedInputs(
    x0_predicted_, interpolator_curvature_pws);

  // DEBUG
  // Get trajectories as a matrix and print for debugging purpose.
  //  // TODO {ali}: disable the debugs.
  //  auto const & traj_data = nonlinear_mpc_controller_ptr_->getCurrentTrajectoryData();
  //  auto && Xtemp = ns_eigen_utils::getTrajectory(traj_data.X);
  //  auto && Utemp = ns_eigen_utils::getTrajectory(traj_data.U);
  //
  //  ns_utils::print("\nSimulated MPC trajectories : ");
  //  ns_eigen_utils::printEigenMat(Xtemp.transpose());  // [x, y, psi, s, ey, epsi, vx, delta, vy]
  //
  //  ns_utils::print("\nSimulated MPC trajectories U : ");
  //  ns_eigen_utils::printEigenMat(Utemp.transpose());
  // end of DEBUG

  // Model::input_vector_t u_model_solution_; // [velocity input - m/s, steering input - rad]

  // Prepare the solution vector placeholder and set it zero before fetching it.
  u_solution_.setZero();

  // use the NMPC.
  // Solve the problem, update the trajectory_data.

  bool is_mpc_solved_tmp{true};
  is_mpc_solved_ = false;

  for (size_t k = 0; k < params_node_.number_of_sqp_iterations; ++k)
  {
    auto &&is_mpc_solved_k =
      nonlinear_mpc_controller_ptr_->solveNMPC_problem(interpolator_curvature_pws);
    is_mpc_solved_tmp = is_mpc_solved_tmp && is_mpc_solved_k;
  }

  // Store mpc solution result.
  is_mpc_solved_ = is_mpc_solved_tmp;

  if (!is_mpc_solved_)
  {
    // vehicle_motion_fsm_.setEmergencyFlag(true);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(),
      "[nonlinear_mpc]: Could not solve the mpc problem ... %s \n",
      std::string(vehicle_motion_fsm_.getFSMTypeReport()).c_str());

    // Publish stopping command.
    ControlCmdMsg stop_cmd{};
    getStopControlCommand(stop_cmd);
    publishControlsAndUpdateVars(stop_cmd);

    return;
  }

  // ns_utils::print("the NMPC problem is solved ...");

  // Get MPC controls [acc, steering rate]
  nonlinear_mpc_controller_ptr_->getControlSolutions(u_solution_);  // [acc, steering_rate]

  // If MPC is solved.
  nonlinear_mpc_controller_ptr_->shiftControls();

  // Compute MPC model predicted longitudinal speed by Euler integration.
  auto const &mpc_vx = nonlinear_mpc_controller_ptr_->getPredictedVxControl();

  // Compute the steering rate by numerical differentiation.
  if (params_node_.use_dob && current_comm_delay_ptr_)
  {
    auto const &dob_steering_ff = current_comm_delay_ptr_->steering_dob;
    u_solution_(1) += dob_steering_ff;
  }

  auto const steering_rate =
    (u_solution_(1) - static_cast<double>(control_cmd_prev_.lateral.steering_tire_angle)) /
    params_node_.control_period;

  // createControlCommand(ax, vx, steer_rate, steer_val)
  ControlCmdMsg control_cmd{};

  // Set the control command.

  control_cmd = createControlCommand(
    u_solution_(ns_utils::toUType(VehicleControlIds::u_vx)),        /* ax */
    mpc_vx,                                                         /* vx input */
    steering_rate,                                                  /* steering_rate */
    u_solution_(ns_utils::toUType(VehicleControlIds::u_steering))); /* steering input */

  // Publish the control command.
  publishControlsAndUpdateVars(control_cmd);

  // Publish the predicted trajectories.
  publishPredictedTrajectories("predicted_trajectory", current_trajectory_ptr_->header.frame_id);

  /**
   * Maintain the input and steering_buffer for predicting the initial states.
   * Put the control command into the input_buffer_map for prediction.
   * */

  auto const &time_that_input_will_apply =
    this->now() + rclcpp::Duration::from_seconds(params_node_.input_delay_time);
  control_cmd.stamp = time_that_input_will_apply;
  inputs_buffer_.emplace_back(control_cmd);

  /** estimate the average solve time */
  const auto &current_mpc_solve_time_msec = stop_watch_.toc(timer_mpc_step);

  // convert milliseconds to seconds.
  auto &&current_mpc_solve_time_sec = current_mpc_solve_time_msec;
  average_mpc_solve_time_ =
    ns_utils::exponentialMovingAverage(average_mpc_solve_time_, 100, current_mpc_solve_time_sec);

  // Set NMPC avg_mpc_computation_time.
  nonlinear_mpc_controller_ptr_->setCurrentAvgMPCComputationTime(average_mpc_solve_time_);

  //  ns_utils::print("\nOne step MPC step takes time to compute in milliseconds : ",
  //  current_mpc_solve_time_msec); ns_utils::print("Average MPC solve time takes time to compute in
  //  milliseconds : ",
  //                  average_mpc_solve_time_ * 1000, "\n");

  // -------------------------------- END of the MPC loop. -------------------------------
  // Set Debug Marker next waypoint
  debug_data_.current_closest_pose = current_interpolated_traj_point_ptr_->pose;

  // Publish the closest point of the trajectory to the vehicle.
  publishClosestPointMarker("closest_point");
}

// Node methods.
void NonlinearMPCNode::updateCurrentPose()
{
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (5000ms).count(),
      "[mpc_nonlinear] Cannot get map to base_link transform. %s", ex.what());
    return;
  }

  geometry_msgs::msg::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;

  current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(ps);

  setCurrentCOGPose(ps);
}

double NonlinearMPCNode::calcStopDistance(const size_t &prev_waypoint_index) const
{
  const double &zero_velocity = EPS;
  const double &origin_velocity = static_cast<double>(
    current_trajectory_ptr_->points.at(prev_waypoint_index).longitudinal_velocity_mps);

  double stop_dist{};

  // Get distances vector from current trajectory computed in the NMPC core.
  std::vector<double> scomputed{};
  nonlinear_mpc_controller_ptr_->getPlannerTravelledDistanceVector(scomputed);

  // Find the zero velocity waypoint ahead.
  if (std::fabs(origin_velocity) > zero_velocity)
  {
    auto it = std::find_if(
      current_trajectory_ptr_->points.cbegin() + static_cast<int>(prev_waypoint_index),
      current_trajectory_ptr_->points.cend(), [&zero_velocity](auto &point)
      {
        return static_cast<double>(std::fabs(point.longitudinal_velocity_mps)) < zero_velocity;
      });

    if (it == current_trajectory_ptr_->points.cend())
    {
      stop_dist = scomputed.back() - scomputed[prev_waypoint_index];
    } else
    {
      auto const &&index_of_zero_vel_point =
        static_cast<size_t>(std::distance(current_trajectory_ptr_->points.cbegin(), it));

      stop_dist = scomputed[index_of_zero_vel_point] - scomputed[prev_waypoint_index];
    }
  }

  // DEBUG
  // ns_utils::print("\nStopping distance : ", stop_dist);
  // ns_utils::print("prev waypoint ", prev_waypoint_index);
  // end of debug

  return stop_dist;
}

bool NonlinearMPCNode::isDataReady()
{
  if (!current_pose_ptr_)
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(),
      "[mpc_nonlinear] Waiting for the current pose ...");
    return false;
  }

  if (!current_trajectory_ptr_)
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(),
      "[mpc_nonlinear] Waiting for the current trajectory ... ");
    return false;
  }

  if (!current_velocity_ptr_)
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(),
      "[mpc_nonlinear] Waiting for the current speed ...");
    return false;
  }

  if (!current_steering_ptr_)
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(),
      "[mpc_nonlinear] Waiting for the current steering measurement ...");
    return false;
  }

  if (!interpolator_curvature_pws.isInitialized())
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(),
      "[mpc_nonlinear] Waiting for the curvature interpolator to be initialized ...");
    return false;
  }

  return true;
}

void NonlinearMPCNode::publishControlCommands(ControlCmdMsg &control_cmd) const
{
  control_cmd.stamp = this->now();
  pub_control_cmd_->publish(control_cmd);
}

void NonlinearMPCNode::publishControlsAndUpdateVars(ControlCmdMsg &control_cmd)
{
  // publish the control command.
  publishControlCommands(control_cmd);

  // Publish NMPC performance variables.
  publishPerformanceVariables(control_cmd);

  // Update the node variables
  control_cmd_prev_ = control_cmd;
  u_previous_solution_ = u_solution_;  // required for the disturbance observer
}

void NonlinearMPCNode::publishPerformanceVariables(ControlCmdMsg const &control_cmd)
{
  // Set nmpc performance variables.
  nmpc_performance_vars_.stamp = this->now();
  nmpc_performance_vars_.steering_angle_input =
    static_cast<double>(control_cmd.lateral.steering_tire_angle);
  nmpc_performance_vars_.steering_angle_rate_input =
    static_cast<double>(control_cmd.lateral.steering_tire_rotation_rate);

  nmpc_performance_vars_.acceleration_input =
    static_cast<double>(control_cmd.longitudinal.acceleration);
  nmpc_performance_vars_.long_velocity_input = static_cast<double>(control_cmd.longitudinal.speed);

  // Set the measured vehicle steering.
  double &&current_steering =
    is_data_ready_ ? static_cast<double>(current_steering_ptr_->steering_tire_angle) : 0.0;
  nmpc_performance_vars_.steering_angle_measured = current_steering;

  // Set the obj_val.
  double &&value_function_value =
    is_mpc_solved_ ? nonlinear_mpc_controller_ptr_->getObjectiveValue() : 0.0;
  nmpc_performance_vars_.nmpc_value = value_function_value;

  // Finite state machine state.
  nmpc_performance_vars_.fsm_state = static_cast<int8_t>(ns_utils::toUType(current_fsm_state_));

  // Average computation time.
  nmpc_performance_vars_.avg_nmpc_computation_time = average_mpc_solve_time_ * 1000;  // s->ms

  // Predicted next steering angle.
  nmpc_performance_vars_.mpc_steering_input_original = u_solution_(1);
  nmpc_performance_vars_.mpc_steering_reference =
    nonlinear_mpc_controller_ptr_->getPredictedSteeringState();

  pub_nmpc_performance_->publish(nmpc_performance_vars_);

  // DEBUG
  // ns_utils::print("IN performance vars Current FSMstate");
}

ControlCmdMsg NonlinearMPCNode::getInitialControlCommand() const
{
  ControlCmdMsg cmd;

  // Steering values.
  cmd.lateral.steering_tire_angle = current_steering_ptr_->steering_tire_angle;
  cmd.lateral.steering_tire_rotation_rate = 0.0;

  // Acceleration and longitudinal speed.
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  return cmd;
}

void NonlinearMPCNode::getStopControlCommand(ControlCmdMsg &control_cmd)
{
  // Steering values.
  control_cmd.lateral.steering_tire_angle = 0.0;
  control_cmd.lateral.steering_tire_rotation_rate = 0.0;

  // Longitudinal control values.
  control_cmd.longitudinal.speed = 0.0;
  control_cmd.longitudinal.acceleration = -1.5;
}

void NonlinearMPCNode::loadNodeParameters()
{
  // Control parameters.
  params_node_.control_frequency = declare_parameter<double>("control_frequency", 30.);
  params_node_.control_period = 1. / params_node_.control_frequency;

  // Delay related parameters :: steering and speed models.
  params_node_.use_delay_sim_model = declare_parameter<bool>("use_delay_sim_model", true);

  // Input delay.
  params_node_.input_delay_time = declare_parameter("input_delay_time", 0.24);

  // Stop state parameters.
  params_node_.stop_state_entry_ego_speed =
    declare_parameter<double>("stop_state_entry_ego_speed", 0.2);

  params_node_.stop_state_entry_target_speed =
    declare_parameter<double>("stop_state_entry_target_speed", 0.5);
  params_node_.stop_state_keep_stopping_dist =
    declare_parameter<double>("stop_state_keep_stopping_dist", 0.5);
  params_node_.will_stop_state_dist = declare_parameter<double>("will_stop_state_dist", 2.0);

  // Compute the discrete input_delay steps.
  if (ns_utils::isEqual(params_node_.input_delay_time, 0.))
  {
    params_node_.input_delay_discrete_nsteps = 1;
    params_node_.predict_initial_states = false;
  } else
  {
    params_node_.input_delay_discrete_nsteps =
      static_cast<size_t>(std::round(params_node_.input_delay_time / params_node_.control_period));
    params_node_.predict_initial_states = true;
  }

  // Initialization parameters.
  params_node_.use_linear_trajectory_initialization =
    declare_parameter<bool>("use_linear_trajectory_initialization", false);

  // Control options.
  params_node_.use_mpc_controller = declare_parameter<bool>("use_mpc_controller", true);
  params_node_.number_of_sqp_iterations = declare_parameter<uint8_t>("number_of_sqp_iterations", 1);

  // Read the vehicle parameters.
  params_node_.lr = declare_parameter<double>("cog_rear_lr", 1.4);

  // CDOB - DOB options
  params_node_.use_cdob = declare_parameter<bool>("use_cdob");
  params_node_.use_dob = declare_parameter<bool>("use_dob");
}

void NonlinearMPCNode::loadVehicleParameters(ns_models::ParamsVehicle &params_vehicle)
{
  params_vehicle.wheel_base = wheel_base_;
  params_vehicle.lr = params_node_.lr;

  params_vehicle.steering_tau = declare_parameter<double>("steering_time_constant", 0.27);
  params_vehicle.speed_tau = declare_parameter<double>("speed_time_constant", 0.61);
  params_vehicle.use_delay_model = params_node_.use_delay_sim_model;

  params_node_.steering_tau = params_vehicle.steering_tau;
}

void NonlinearMPCNode::loadNMPCoreParameters(
  ns_data::data_nmpc_core_type_t &data_nmpc_core, ns_data::param_lpv_type_t &params_lpv,
  ns_data::ParamsOptimization &params_optimization)
{
  data_nmpc_core.wheel_base = wheel_base_;

  // mpc_timestep_dt is already set in the constructor.
  data_nmpc_core.input_delay_time = params_node_.input_delay_time;

  // Load optimization parameters.
  // State and control weights. Q. Reads only the diagonal terms .
  std::vector<double> temp(Model::state_dim);
  std::vector<double> default_vec{0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0., 0.0};
  temp = declare_parameter<std::vector<double>>("state_weights", default_vec);
  params_optimization.Q.diagonal() = Model::state_vector_t::Map(temp.data());

  // QN
  temp.clear();
  temp.reserve(Model::state_dim);
  default_vec = std::vector<double>{0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0};
  temp = declare_parameter<std::vector<double>>("state_weights_terminal", default_vec);
  params_optimization.QN.diagonal() = Model::state_vector_t::Map(temp.data());

  // R - Control weights.
  temp.clear();
  temp.reserve(Model::input_dim);
  default_vec = std::vector<double>{0.001, 0.0001};
  temp = declare_parameter<std::vector<double>>("control_weights", default_vec);
  params_optimization.R.diagonal() = Model::input_vector_t::Map(temp.data());

  // Rj - Jerk weights.
  temp.clear();
  temp.reserve(Model::input_dim);
  default_vec = std::vector<double>{1., 1.};
  temp = declare_parameter<std::vector<double>>("jerk_weights", default_vec);
  params_optimization.Rj.diagonal() = Model::input_vector_t::Map(temp.data());

  // State and input bounds. xlower bound.
  temp.clear();
  temp.reserve(Model::state_dim);
  default_vec = std::vector<double>{-kInfinity, -kInfinity, -kInfinity, -kInfinity, -2.0,
                                    -1.0, 0.0, -0.69, -kInfinity};

  temp = declare_parameter<std::vector<double>>("xlower", default_vec);
  params_optimization.xlower = Model::state_vector_t::Map(temp.data());

  // State and input bounds. xupper bound.
  temp.clear();
  temp.reserve(Model::state_dim);
  default_vec = std::vector<double>{kInfinity, kInfinity, kInfinity, kInfinity, 2.0,
                                    1.0, 25.0, 0.69, kInfinity};
  temp = declare_parameter<std::vector<double>>("xupper", default_vec);
  params_optimization.xupper = Model::state_vector_t::Map(temp.data());

  // State and input bounds. ulower bound.
  temp.clear();
  temp.reserve(Model::input_dim);
  default_vec = std::vector<double>{-30., -1.};
  temp = declare_parameter<std::vector<double>>("ulower", default_vec);
  params_optimization.ulower = Model::input_vector_t::Map(temp.data());

  temp.clear();
  temp.reserve(Model::input_dim);
  default_vec = std::vector<double>{30., 1.};
  temp = declare_parameter<std::vector<double>>("uupper", default_vec);
  params_optimization.uupper = Model::input_vector_t::Map(temp.data());

  // xmax bound: xmax is used for scaling, whereas xlower is related to whether there is an upper
  // bound.
  temp.clear();
  temp.reserve(Model::state_dim);
  default_vec = std::vector<double>{-50., -50., -3.14, 0.0, -3.0, -1.0, 0.0, -0.69, -5.0};
  temp = declare_parameter<std::vector<double>>("xmin_for_scaling", default_vec);
  params_optimization.xmin_for_scaling = Model::state_vector_t::Map(temp.data());

  temp.clear();
  temp.reserve(Model::state_dim);
  default_vec = std::vector<double>{50., 50., 3.14, 40.0, 3.0, 1.0, 10.0, 0.69, 5.0};
  temp = declare_parameter<std::vector<double>>("xmax_for_scaling", default_vec);
  params_optimization.xmax_for_scaling = Model::state_vector_t::Map(temp.data());

  temp.clear();
  temp.reserve(Model::input_dim);
  default_vec = std::vector<double>{-50., -1.0};
  temp = declare_parameter("umin_for_scaling", default_vec);
  params_optimization.umin_for_scaling = Model::input_vector_t::Map(temp.data());

  temp.clear();
  temp.reserve(Model::input_dim);
  default_vec = std::vector<double>{50., 1.0};
  temp = declare_parameter<std::vector<double>>("umax_for_scaling", default_vec);
  params_optimization.umax_for_scaling = Model::input_vector_t::Map(temp.data());

  // Load the normalization scaling range.
  params_optimization.scaling_range =
    declare_parameter<std::vector<double>>("scaling_range", std::vector<double>{-1., 1.});

  // OSQP parameters
  params_optimization.osqp_warm_start = declare_parameter<bool>("osqp_warm_start", true);
  params_optimization.osqp_polishing = declare_parameter<bool>("osqp_polishing", true);
  params_optimization.osqp_scaling_iter = declare_parameter<int32_t>("osqp_scaling_iter", 10);

  params_optimization.osqp_max_iters = declare_parameter<int64_t>("osqp_max_iters", 200);
  params_optimization.osqp_eps_abs = declare_parameter<double>("osqp_eps_abs", 1e-5);
  params_optimization.osqp_eps_rel = declare_parameter<double>("osqp_eps_rel", 1e-5);
  params_optimization.osqp_verbose = declare_parameter<bool>("osqp_verbose", true);
  params_optimization.osqp_polish_iters = declare_parameter<int64_t>("osqp_polish_iters", 50);
  params_optimization.osqp_time_limit = declare_parameter<double>("osqp_time_limit", 0.2);
  params_optimization.osqp_scaled_termination = declare_parameter("osqp_scaled_termination", false);

  // LOAD LPV parameters
  params_lpv.num_of_nonlinearities = ns_nmpc_interface::NUM_LPV_NONLINEARITIES;
  // declare_parameter<int8_t>("num_of_nonlinearities", 5);

  // There is one more Lyapunov matrices X and Y than the number of parameters.
  size_t const &&nx = Model::estate_dim;
  size_t const &&nu = Model::input_dim;

  // Reading no-delayed or delayed Lyapunov matrices.
  auto labelX_tag = "Xn";  // No delay nodel for steering and longitudinal speed
  auto labelY_tag = "Yn";

  for (size_t k = 0; k < params_lpv.num_of_nonlinearities; k++)
  {
    std::vector<double> tempX;
    tempX.reserve(nx * nx);

    std::vector<double> tempY;
    tempY.reserve(nx * nu);

    auto labelX = labelX_tag + std::to_string(k + 1);
    auto labelY = labelY_tag + std::to_string(k + 1);

    tempX = declare_parameter<std::vector<double>>(labelX);
    tempY = declare_parameter<std::vector<double>>(labelY);

    auto tempXmat = Model::state_matrix_X_t::Map(tempX.data());
    auto tempYmat = Model::input_matrix_Y_t::Map(tempY.data());

    params_lpv.lpvXcontainer.emplace_back(tempXmat);
    params_lpv.lpvYcontainer.emplace_back(tempYmat);
  }
}

/**
 *@brief : The state and control variables are scaled to a predefined interval which is typically
 *[-1, 1]. xmax = a*xmax_s + b xmin = a*xmin_s + b where xmin, xmax are the original magnitude and
 *_s tag is the scaling interval.
 *
 * a = xmax - xmin / (xmax_s - xmin_s) and b = xmax - a*xmax_s or (with max replaced my min)
 * */

void NonlinearMPCNode::computeScalingMatrices(ns_data::ParamsOptimization &params)
{
  // Scaling interval. [-1, 1]
  double const &xuhatmin = params.scaling_range[0];
  double const &xuhatmax = params.scaling_range[1];

  params.Sx.diagonal() =
    (params.xmax_for_scaling - params.xmin_for_scaling) / (xuhatmax - xuhatmin);

  params.Cx = params.xmax_for_scaling - params.Sx.diagonal() * xuhatmax;

  params.Su.diagonal() =
    (params.umax_for_scaling - params.umin_for_scaling) / (xuhatmax - xuhatmin);

  params.Cu = params.umax_for_scaling - params.Su.diagonal() * xuhatmax;

  // Inverse scalers.
  params.Sx_inv.diagonal() = params.Sx.diagonal().unaryExpr([](auto const &x)
                                                            { return 1 / x; });

  params.Su_inv.diagonal() = params.Su.diagonal().unaryExpr([](auto const &x)
                                                            { return 1 / x; });

  // Using the scaling matrices compute the normalized bounds of the states and the inputs.
  // Set the scaled lower and upper bounds.

  for (Eigen::Index k = 0; k < Model::state_dim; ++k)
  {
    double const &&val_low =
      (params.xlower(k) >= kInfinity - EPS || params.xlower(k) <= -kInfinity + EPS)
      ? params.xlower(k)
      : (params.xlower(k) - params.Cx(k)) * params.Sx_inv.diagonal()(k);

    double const &&val_up =
      (params.xupper(k) >= kInfinity - EPS || params.xupper(k) <= -kInfinity + EPS)
      ? params.xupper(k)
      : (params.xupper(k) - params.Cx(k)) * params.Sx_inv.diagonal()(k);

    params.xlower_scaled(k) = val_low;

    params.xupper_scaled(k) = val_up;
  }

  for (Eigen::Index k = 0; k < Model::input_dim; ++k)
  {
    double const &&val_low =
      (params.ulower(k) >= kInfinity - EPS || params.ulower(k) <= -kInfinity + EPS)
      ? params.ulower(k)
      : (params.ulower(k) - params.Cu(k)) * params.Su_inv.diagonal()(k);

    double const &&val_up =
      (params.uupper(k) >= kInfinity - EPS || params.uupper(k) <= -kInfinity + EPS)
      ? params.uupper(k)
      : (params.uupper(k) - params.Cu(k)) * params.Su_inv.diagonal()(k);

    params.ulower_scaled(k) = val_low;
    params.uupper_scaled(k) = val_up;
  }
}

// Callbacks.
void NonlinearMPCNode::onTrajectory(const TrajectoryMsg::SharedPtr msg)
{
  if (msg->points.size() < 3)
  {
    RCLCPP_DEBUG(get_logger(), "received path size is < 3, not enough.");
    return;
  }

  if (!isValidTrajectory(*msg))
  {
    RCLCPP_ERROR(get_logger(), "Trajectory is invalid!! stop computing.");
    return;
  }

  current_trajectory_ptr_ = msg;
  current_trajectory_size_ = msg->points.size();

  // Resample the planning trajectory and store in the MPCdataTrajectoryVectors.
  if (bool const &&is_resampled = resampleRawTrajectoriesToaFixedSize(); !is_resampled)
  {
    RCLCPP_ERROR(get_logger(), "[nonlinear_mpc] Could not resample the trajectory ...");
    return;
  }
}

void NonlinearMPCNode::onVelocity(VelocityMsg::SharedPtr msg)
{ current_velocity_ptr_ = msg; }

void NonlinearMPCNode::onCommDelayCompensation(const DelayCompensationRefs::SharedPtr msg)
{
  current_comm_delay_ptr_ = msg;
}

void NonlinearMPCNode::onSteeringMeasured(SteeringMeasuredMsg::SharedPtr msg)
{
  current_steering_ptr_ = msg;
}

bool NonlinearMPCNode::isValidTrajectory(const TrajectoryMsg &msg_traj)
{
  bool const &&check_condition =
    std::all_of(std::cbegin(msg_traj.points), std::cend(msg_traj.points), [](auto point)
    {
      const auto &p = point.pose.position;
      const auto &o = point.pose.orientation;
      const auto &vx = point.longitudinal_velocity_mps;
      const auto &vy = point.lateral_velocity_mps;
      const auto &wxf = point.front_wheel_angle_rad;
      const auto &wxr = point.rear_wheel_angle_rad;

      return static_cast<bool>(
        isfinite(p.x) && isfinite(p.y) && isfinite(p.z) && isfinite(o.x) && isfinite(o.y) &&
        isfinite(o.z) && isfinite(o.w) && isfinite(vx) && isfinite(vy) && isfinite(wxf) &&
        isfinite(wxr));
    });

  return check_condition;
}

bool NonlinearMPCNode::resampleRawTrajectoriesToaFixedSize()
{
  // Creating MPC trajectory instance.
  ns_data::MPCdataTrajectoryVectors mpc_traj_raw(current_trajectory_size_);
  mpc_traj_raw.emplace_back(*current_trajectory_ptr_);

  if (mpc_traj_raw.size() == 0)
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(),
      "[mpc_nonlinear] MPCdataTrajectoryVectors are not assigned properly ...");

    return false;
  }
  nonlinear_mpc_controller_ptr_->setMPCtrajectoryRawVectorsPtr(mpc_traj_raw);

  // DEBUG
  // end of DEBUG

  // --------------- Fill the Eigen reference_map_sxyz --------------------------
  // !<-@brief smooth map reference of [MPC_MAP_SMOOTHER_OUT x 4]
  // !<-@brief [s, x, y, z ] size [MPC_MAP_SMOOTHER_IN x 4]
  map_matrix_in_t reference_map_sxyz(map_matrix_in_t::Zero());

  if (bool const &&is_resampled = makeFixedSizeMat_sxyz(mpc_traj_raw, reference_map_sxyz);
    !is_resampled)
  {
    RCLCPP_ERROR(
      get_logger(),
      "[mpc_nonlinear - resampleRawTrajectoryToAFixedSize ] Could not interpolate the "
      "coordinates ...");
    return false;
  }

  // ------------------- Smooth Trajectories ---------------------------------------
  // Create MPCtraj smooth_ref_traj.
  bool const &&is_smoothed =
    createSmoothTrajectoriesWithCurvature(mpc_traj_raw, reference_map_sxyz);

  return is_smoothed;
}

bool NonlinearMPCNode::makeFixedSizeMat_sxyz(
  const ns_data::MPCdataTrajectoryVectors &mpc_traj_raw, map_matrix_in_t &fixed_map_ref_sxyz)
{
  /**
   * @brief Raw trajectory enters in variable size, it is resampled into a fixed size, and curvature
   * map is created to mpc interpolator_spline_pws size (ns_nmpc_interface::MPC_MAP_SMOOTHER_OUT).
   */

  size_t const &map_in_fixed_size = ns_splines::MPC_MAP_SMOOTHER_IN;

  // Create a new distance vector.
  double const &initial_distance = mpc_traj_raw.s.front();  // s0
  double const &final_distance = mpc_traj_raw.s.back();     // sf

  /**
   * @brief A new coordinate vector for a fixed size trajectories.
   **/
  std::vector<double> s_fixed_size_coordinate =
    ns_utils::linspace(initial_distance, final_distance, map_in_fixed_size);

  /**
   * @brief Create a piece-wise cubic interpolator for x and y.
   * */
  ns_splines::InterpolatingSplinePCG interpolator_spline_pws(3);  // piecewise

  /**
   * @brief Create a piece-wise linear interpolator for the rest of the coordinates.
   * */
//  ns_splines::InterpolatingSplinePCG interpolator_linear(1);
//
//  // Interpolated vector containers.
//  std::vector<double> xinterp;
//  std::vector<double> yinterp;
//  std::vector<double> zinterp;
//
//  xinterp.reserve(map_in_fixed_size);
//  yinterp.reserve(map_in_fixed_size);
//  zinterp.reserve(map_in_fixed_size);
//
//  // Resample the varying size raw trajectory into a fixed size trajectory points.
//  auto const &&is_interpolated_x = interpolator_spline_pws.Interpolate(
//    mpc_traj_raw.s, mpc_traj_raw.x, s_fixed_size_coordinate, xinterp);
//
//  auto const &&is_interpolated_y = interpolator_spline_pws.Interpolate(
//    mpc_traj_raw.s, mpc_traj_raw.y, s_fixed_size_coordinate, yinterp);
//
//  auto const &&is_interpolated_z = interpolator_linear.Interpolate(
//    mpc_traj_raw.s, mpc_traj_raw.z, s_fixed_size_coordinate, zinterp);

  ns_utils::print("in make fixed map : sbase start end vs sfixed start end: ");
  ns_utils::print("in make fixed map : sbase start end vs sfixed start end: ",
                  mpc_traj_raw.s.front(),
                  mpc_traj_raw.s.back(),
                  s_fixed_size_coordinate.front(),
                  s_fixed_size_coordinate.back());

  // Interpolated vector containers.
  // Resample the varying size raw trajectory into a fixed size trajectory points.
  auto xinterp = interpolation::spline(mpc_traj_raw.s, mpc_traj_raw.x, s_fixed_size_coordinate);
  auto yinterp = interpolation::spline(mpc_traj_raw.s, mpc_traj_raw.y, s_fixed_size_coordinate);
  auto zinterp = interpolation::spline(mpc_traj_raw.s, mpc_traj_raw.z, s_fixed_size_coordinate);

  // ns_utils::print("on trajectory vector sizes, zinterp vs map size", zinterp.size(),
  // map_in_fixed_size);

  /**
   * @brief save into the reference map which accepts the resampled fixed size coordinates.
   * */
  fixed_map_ref_sxyz.col(0) = map_vector_in_t::Map(s_fixed_size_coordinate.data());  // snew
  fixed_map_ref_sxyz.col(1) = map_vector_in_t::Map(xinterp.data());                  // xnew
  fixed_map_ref_sxyz.col(2) = map_vector_in_t::Map(yinterp.data());                  // ynew
  fixed_map_ref_sxyz.col(3) = map_vector_in_t::Map(zinterp.data());                  // znew

  bool are_interpolated = s_fixed_size_coordinate.size() == xinterp.size();
  are_interpolated &= s_fixed_size_coordinate.size() == yinterp.size();
  are_interpolated &= s_fixed_size_coordinate.size() == zinterp.size();

  return are_interpolated;
}

bool NonlinearMPCNode::createSmoothTrajectoriesWithCurvature(
  ns_data::MPCdataTrajectoryVectors const &mpc_traj_raw,
  map_matrix_in_t const &fixed_map_ref_sxyz)
{
  using ::ns_data::trajVectorVariant;

  // Maps the raw trajectory into the MPC trajectory.
  size_t const &map_out_mpc_size = ns_splines::MPC_MAP_SMOOTHER_OUT;  // to the NMPC
  ns_data::MPCdataTrajectoryVectors mpc_traj_smoothed(map_out_mpc_size);

  Eigen::MatrixXd interpolated_map;  // [s, x, y, z]

  // Signature (base map, to interpolated map).
  bspline_interpolator_ptr_->InterpolateImplicitCoordinates(fixed_map_ref_sxyz, interpolated_map);

  /**
   * @brief compute the first derivatives; xdot, ydot for the curvature computations.
   * rdot = [xdot, ydot]
   * */
  Eigen::MatrixXd rdot_interp(static_cast<Eigen::Index>(ns_splines::MPC_MAP_SMOOTHER_OUT), 2);

  /**
   * @brief compute the second derivatives; xddot, yddot for the curvature computations.
   * rddot = [xddot, yddot]
   * */
  Eigen::MatrixXd rddot_interp(static_cast<Eigen::Index>(ns_splines::MPC_MAP_SMOOTHER_OUT), 2);

  // Get the first and second derivatives of [x, y]
  auto const &xy_data = fixed_map_ref_sxyz.middleCols(1, 2);  // start from x and gets xy
  bspline_interpolator_ptr_->getFirstDerivative(xy_data, rdot_interp);
  bspline_interpolator_ptr_->getSecondDerivative(xy_data, rddot_interp);

  /** @brief Compute the curvature column. */
  auto const &curvature = ns_eigen_utils::Curvature(rdot_interp, rddot_interp);

  std::vector<double> s_smooth_vect(map_out_mpc_size);
  std::vector<double> x_smooth_vect(map_out_mpc_size);
  std::vector<double> y_smooth_vect(map_out_mpc_size);
  std::vector<double> z_smooth_vect(map_out_mpc_size);
  std::vector<double> v_smooth_vect(map_out_mpc_size);
  std::vector<double> curvature_smooth_vect(map_out_mpc_size);

  // These are not smoothed, but they belong to the smooth MPCtraj.
  std::vector<double> t_smooth_vect(map_out_mpc_size);
  std::vector<double> acc_smooth_vect(map_out_mpc_size);

  // Smooth yaw.
  std::vector<double> yaw_smooth_vect(map_out_mpc_size);  // We do not use yaw as a reference.

  // Compute time and acceleration reference.
  // Insert the first elements.
  t_smooth_vect.at(0) = 0.;

  for (Eigen::Index k = 0; k < rdot_interp.rows(); ++k)
  {
    s_smooth_vect.at(k) = interpolated_map.col(0)(k);
    x_smooth_vect.at(k) = interpolated_map.col(1)(k);
    y_smooth_vect.at(k) = interpolated_map.col(2)(k);
    z_smooth_vect.at(k) = interpolated_map.col(3)(k);

    // interpolate the longitudinal speed.
    double vx_temp{};
    ns_utils::interp1d_linear(mpc_traj_raw.s, mpc_traj_raw.vx, s_smooth_vect.at(k), vx_temp);

    v_smooth_vect.at(k) = vx_temp;

    curvature_smooth_vect.at(k) = curvature.col(0)(k);
    yaw_smooth_vect.at(k) = std::atan2(rdot_interp(k, 1), rdot_interp(k, 0));

    // Insert  t and acc,
    if (k > 0)
    {
      // until k-1
      double const &ds = s_smooth_vect.at(k) - s_smooth_vect.at(k - 1);

      // used for trapezoidal integration rule.
      double const &mean_v = (v_smooth_vect.at(k) + v_smooth_vect.at(k - 1)) / 2;
      double const &dv = v_smooth_vect.at(k) - v_smooth_vect.at(k - 1);  // dv = v[k]-v[k-1],
      double const &dt = ds / std::max(mean_v, 0.1);  // to prevent zero division.

      // this acceleration is implied by x,y,z and vx in the planner.
      double const &&acc_computed = dv / (EPS + dt);
      t_smooth_vect.at(k) = t_smooth_vect.at(k - 1) + dt;
      acc_smooth_vect.at(k - 1) = acc_computed;
    }
  }

  // Repeat the last acceleration.
  acc_smooth_vect.rbegin()[0] = acc_smooth_vect.rbegin()[1];

  // Convert smooth yaw to a monotonic series
  ns_utils::unWrap(yaw_smooth_vect);

  mpc_traj_smoothed.setTrajectoryVector(s_smooth_vect, trajVectorVariant{ns_data::s_tag()});
  mpc_traj_smoothed.setTrajectoryVector(t_smooth_vect, trajVectorVariant{ns_data::t_tag()});
  mpc_traj_smoothed.setTrajectoryVector(acc_smooth_vect, trajVectorVariant{ns_data::a_tag()});
  mpc_traj_smoothed.setTrajectoryVector(x_smooth_vect, trajVectorVariant{ns_data::x_tag()});
  mpc_traj_smoothed.setTrajectoryVector(y_smooth_vect, trajVectorVariant{ns_data::y_tag()});
  mpc_traj_smoothed.setTrajectoryVector(z_smooth_vect, trajVectorVariant{ns_data::z_tag()});
  mpc_traj_smoothed.setTrajectoryVector(v_smooth_vect, trajVectorVariant{ns_data::vx_tag()});
  mpc_traj_smoothed.setTrajectoryVector(yaw_smooth_vect, trajVectorVariant{ns_data::yaw_tag()});
  mpc_traj_smoothed.setTrajectoryVector(
    curvature_smooth_vect, trajVectorVariant{ns_data::curv_tag()});

  // Compute relative time and acceleration from the given data.
  if (current_fsm_state_ == ns_states::motionStateEnums::willbeStopping)
  {
    mpc_traj_smoothed.addExtraEndPoints(average_mpc_solve_time_);
  }

  nonlinear_mpc_controller_ptr_->setMPCtrajectorySmoothVectorsPtr(mpc_traj_smoothed);

  // DEBUG
  // TODO: remove this debug
  //  ns_utils::print("Smoothed MPC traj ");
  //  mpc_traj_smoothed.print();
  // end of DEBUG

  // Verify size
  if (auto const &size_of_mpc_smooth = mpc_traj_smoothed.size(); size_of_mpc_smooth == 0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "[mpc_nonlinear - resampleRawTrajectoryToAFixedSize ] the smooth "
      "MPCdataTrajectoryVectors  are not assigned properly ... ");
    return false;
  }

  /**
   *  @brief update curvature spline interpolator data. This interpolator is used all across the
   * node modules.
   * */

  if (auto const &is_updated =
      interpolator_curvature_pws.Initialize(mpc_traj_smoothed.s, mpc_traj_smoothed.curvature);
    !is_updated)
  {
    RCLCPP_ERROR(
      get_logger(),
      "[mpc_nonlinear - resampling] Could not update the point-wise curvature "
      "interpolator_spline_pws  data ...");
    return false;
  }

  // DEBUG

  // end of DEBUG
  return true;
}

void NonlinearMPCNode::findClosestPrevWayPointIdx()
{
  /**
   * from the Autoware motion utils.
   * */

  auto const &xvec = current_COG_pose_ptr_->pose.position.x;
  auto const &yvec = current_COG_pose_ptr_->pose.position.y;
  auto const &yawvec = tf2::getYaw(current_COG_pose_ptr_->pose.orientation);
  double min_dist = std::numeric_limits<double>::max();

  size_t nearest_idx{};

  for (size_t k = 0; k < current_trajectory_size_; ++k)
  {
    auto const &point = current_trajectory_ptr_->points[k];
    auto const &pose_yaw = tf2::getYaw(point.pose.orientation);

    if (auto const &yaw_diff = ns_utils::angleDistance(yawvec, pose_yaw);
      std::fabs(yaw_diff) > (M_PI / 3.0))
    {
      continue;
    }

    auto const &posex = point.pose.position.x;
    auto const &posey = point.pose.position.y;

    auto const &dist_to_point = std::hypot(posex - xvec, posey - yvec);

    if (dist_to_point < min_dist)
    {
      // ns_utils::print("yaw condition matched k", k);
      min_dist = dist_to_point;
      nearest_idx = k;
    }
  }

  // Check if the nearest idx point is behind or ahead of the vehicle.
  auto const &nearest_position = current_trajectory_ptr_->points[nearest_idx].pose.position;
  std::array<double, 2> next_idx_point_vec{nearest_position.x - xvec, nearest_position.y - yvec};

  auto const &vehicle_tangent = ns_utils::getTangentVector(yawvec);
  auto const &projection_on_vehicle_tangent =
    vehicle_tangent[0] * next_idx_point_vec[0] + vehicle_tangent[1] * next_idx_point_vec[1];

  size_t nearest_idx_prev{};
  size_t nearest_idx_next{};

  if (projection_on_vehicle_tangent <= 0)
  {
    nearest_idx_prev = nearest_idx;
  } else
  {
    size_t zero{};
    nearest_idx_prev = std::max(zero, nearest_idx - 1);
  }

  nearest_idx_next = std::min(nearest_idx_prev + 1, current_trajectory_size_ - 1);

  idx_prev_wp_ptr_ = std::make_unique<size_t>(nearest_idx_prev);
  idx_next_wp_ptr_ = std::make_unique<size_t>(nearest_idx_next);
}

void NonlinearMPCNode::computeClosestPointOnTraj()
{
  // Create performance variables msg to track the performance of NMPC.
  nmpc_performance_vars_ = NonlinearMPCPerformanceMsg{};

  // Create a new pose to keep the interpolated trajectory point.
  TrajectoryPoint interpolated_traj_point{};

  // Get index of prev waypoint and sanity check.
  if (!idx_prev_wp_ptr_)
  {
    RCLCPP_ERROR(get_logger(), "[mpc_nonlinear_core] Cannot find the next waypoint.");
    return;
  }

  /**
   *  Create two vectors originating from the previous waypoints to the next waypoint and the
   * vehicle position and find projection of vehicle vector on the the trajectory section
   * */

  // Previous waypoint to next waypoint
  double const &&dx_prev_to_next =
    current_trajectory_ptr_->points.at(*idx_next_wp_ptr_).pose.position.x -
    current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.position.x;

  double const &&dy_prev_to_next =
    current_trajectory_ptr_->points.at(*idx_next_wp_ptr_).pose.position.y -
    current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.position.y;

  double const &&dz_prev_to_next =
    current_trajectory_ptr_->points.at(*idx_next_wp_ptr_).pose.position.z -
    current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.position.z;

  // Create a vector from p0 (prev) --> p1 (to next wp).
  // std::vector<double> v_prev_to_next_wp{dx_prev_to_next, dy_prev_to_next};

  // Previous waypoint to the vehicle pose
  /**
   *   p0:previous waypoint ----> p1 next waypoint
   *   vector = p1 - p0. We project vehicle vector on this interval to
   * */

  double const &&dx_prev_to_vehicle =
    current_COG_pose_ptr_->pose.position.x -
    current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.position.x;

  double const &&dy_prev_to_vehicle =
    current_COG_pose_ptr_->pose.position.y -
    current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.position.y;

  // Vector from p0 to p_vehicle.
  std::vector<double> v_prev_to_vehicle{dx_prev_to_vehicle, dy_prev_to_vehicle};

  // Compute the length of v_prev_to_next_wp : vector from p0 --> p1.
  double const &&magnitude_p0_to_p1 =
    std::hypot(dx_prev_to_next, dy_prev_to_next, dz_prev_to_next);

  /**
   * Compute how far the car is away from p0 in direction. p_interp is the location of the
   * interpolated waypoint. This is the dot product normalized by the length of the interval.
   * a.b = |a|.|b|.cos(alpha) -- > |a|.cos(alpha) = a.b / |b| where b is the path interval.
   * */
  double const &&ds_distance_p0_to_p_interp =
    (dx_prev_to_next * dx_prev_to_vehicle + dy_prev_to_next * dy_prev_to_vehicle) /
    magnitude_p0_to_p1;

  // Get the current distance from the origin and keep in te current_s0_ variable.
  nonlinear_mpc_controller_ptr_->getRawDistanceAtIdx(*idx_prev_wp_ptr_, current_s0_);
  current_s0_ += ds_distance_p0_to_p_interp;
  nonlinear_mpc_controller_ptr_->setCurrent_s0(current_s0_);

  /**
   * Using the smooth variables.
   * */
  std::array<double, 3> xyz{};
  nonlinear_mpc_controller_ptr_->getSmoothXYZAtDistance(current_s0_, xyz);

  interpolated_traj_point.pose.position.x = xyz[0];
  interpolated_traj_point.pose.position.y = xyz[1];
  interpolated_traj_point.pose.position.z = xyz[2];

  double interp_yaw_angle{};
  nonlinear_mpc_controller_ptr_->getSmoothYawAtDistance(current_s0_, interp_yaw_angle);
  interp_yaw_angle = ns_utils::wrapToPi(interp_yaw_angle);

  // nmpc_performance_vars_.lateral_velocity = interp_yaw_angles;
  // double const &&interp_yaw_angle = ns_utils::angleDistance(prev_yaw + ratio_t *
  // dyaw_prev_to_next);

  geometry_msgs::msg::Quaternion orient_msg =
    ns_nmpc_utils::createOrientationMsgfromYaw(interp_yaw_angle);
  interpolated_traj_point.pose.orientation = orient_msg;

  // InterpolateInCoordinates the Vx longitudinal speed.
  double const &vx_prev = static_cast<double>(
    current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).longitudinal_velocity_mps);
  // double const &vx_next =
  // current_trajectory_ptr_->points.at(*idx_next_wp_ptr_).longitudinal_velocity_mps;

  // double const &&vx_target = vx_prev + ratio_t * (vx_next - vx_prev);
  double vx_target{};
  nonlinear_mpc_controller_ptr_->getSmoothVxAtDistance(current_s0_, vx_target);

  interpolated_traj_point.longitudinal_velocity_mps =
    static_cast<decltype(interpolated_traj_point.longitudinal_velocity_mps)>(vx_target);

  // Set the current target speed of nmpc_performance_vars_.
  nmpc_performance_vars_.long_velocity_target = vx_target;

  // Compute relative time.
  double const &&average_speed_v0_to_vtarget = (vx_prev + vx_target) / 2;

  // The time takes travelling from p0 to the p_interpolated with the prescribed vx.
  double const &&dt_time = ds_distance_p0_to_p_interp / std::max(average_speed_v0_to_vtarget, 0.1);

  nonlinear_mpc_controller_ptr_->getRawRelativeTimeAtIdx(*idx_prev_wp_ptr_, current_t0_);
  current_t0_ += dt_time;

  nonlinear_mpc_controller_ptr_->setCurrent_t0(current_t0_);

  // Create the pointer to keep.
  current_interpolated_traj_point_ptr_ = std::make_unique<TrajectoryPoint>(interpolated_traj_point);
}

std::array<double, 2> NonlinearMPCNode::computeErrorStates()
{
  /**
   * @brief current closest point on the trajectory
   * */
  std::array<double, 2> const current_pose_xy{
    current_COG_pose_ptr_->pose.position.x, current_COG_pose_ptr_->pose.position.y};

  // Get Yaw angles of the reference waypoint and the vehicle
  std::array<double, 2> interpolated_smooth_traj_point_xy{
    current_interpolated_traj_point_ptr_->pose.position.x,
    current_interpolated_traj_point_ptr_->pose.position.y};

  auto reference_yaw_angle = tf2::getYaw(current_interpolated_traj_point_ptr_->pose.orientation);
  reference_yaw_angle =
    ns_utils::angleDistance(reference_yaw_angle);  // overloaded angle distance also wraps.

  double const &vehicle_yaw_angle = tf2::getYaw(current_COG_pose_ptr_->pose.orientation);

  /**
   *  We project the vector from the reference point to the car on the normal of reference point.
   * */
  auto &&normal_vector = ns_utils::getNormalVector(reference_yaw_angle);

  // Vector to path point originating from the vehicle
  std::array<double, 2> vector_to_path_point{
    current_pose_xy[0] - interpolated_smooth_traj_point_xy[0],
    current_pose_xy[1] - interpolated_smooth_traj_point_xy[1]};

  auto const &error_ey =
    normal_vector[0] * vector_to_path_point[0] + normal_vector[1] * vector_to_path_point[1];

  double const &heading_yaw_error =
    1.0 * ns_utils::angleDistance(vehicle_yaw_angle, reference_yaw_angle);
  // heading_yaw_error = autoware::common::helper_functions::wrap_angle(heading_yaw_error);

  // Set nmpc_performance yaw angles.
  nmpc_performance_vars_.yaw_angle_measured = vehicle_yaw_angle;
  nmpc_performance_vars_.yaw_angle_target = reference_yaw_angle;
  nmpc_performance_vars_.yaw_angle_traj =
    tf2::getYaw(current_interpolated_traj_point_ptr_->pose.orientation);

  std::array<double, 2> const error_states{error_ey, 1.0 * heading_yaw_error};

  return error_states;
}

void NonlinearMPCNode::updateInitialStatesAndControls_fromMeasurements()
{
  using ns_utils::toUType;
  // Compute the errors  [e_y, e_psi].
  std::array<double, 2> const &&error_states = computeErrorStates();

  // Fill the initial states [xw, yw, yaw, s, ey, e_yaw, v, delta].
  x0_previous_initial_states_ = x0_initial_states_;
  x0_initial_states_.setZero();

  // Vehicle x-y are always zero at each iteration.
  // x0_initial_states_(0) = 0.0;  // <-@brief current_pose_ptr_->pose.position.x - xw0;
  // x0_initial_states_(1) = 0.0;  // <-@brief current_pose_ptr_->pose.position.y - yw0;
  x0_initial_states_(toUType(VehicleStateIds::yaw)) =
    tf2::getYaw(current_COG_pose_ptr_->pose.orientation);
  x0_initial_states_(toUType(VehicleStateIds::s)) = current_s0_;

  // Error model states.
  x0_initial_states_(toUType(VehicleStateIds::ey)) =
    error_states[0];  // e_y  : lateral tracking error.
  x0_initial_states_(toUType(VehicleStateIds::eyaw)) = error_states[1];  // e_psi : heading error

  auto const &vx_meas = current_velocity_ptr_->twist.twist.linear.x;
  x0_initial_states_(toUType(VehicleStateIds::vx)) = vx_meas;  // vx longitudinal speed state
  x0_initial_states_(toUType(VehicleStateIds::steering)) =
    static_cast<double>(current_steering_ptr_->steering_tire_angle);

  x0_initial_states_(toUType(VehicleStateIds::vy)) = current_velocity_ptr_->twist.twist.linear.y;

  // TODO: remove this warning.
  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    get_logger(), *get_clock(), (1000ms).count(), "\n[mpc_nonlinear] ey : %2.2f, eyaw : %2.2f",
    error_states[0], error_states[1]);

  // compute the current curvature and store it.
  if (auto const &&could_interpolate =
      interpolator_curvature_pws.Interpolate(current_s0_, current_curvature_k0_);
    !could_interpolate)
  {
    RCLCPP_ERROR(
      get_logger(),
      "[mpc_nonlinear] Could not interpolate the curvature in the initial state update "
      "method ...");
    return;
  }

  // Create the dynamic parameters
  Model::param_vector_t params(Model::param_vector_t::Zero());
  current_model_params_.setZero();
  current_model_params_(toUType(VehicleParamIds::curvature)) = current_curvature_k0_;

  double vx_target_0{};
  nonlinear_mpc_controller_ptr_->getSmoothVxAtDistance(current_s0_, vx_target_0);
  current_model_params_(toUType(VehicleParamIds::target_vx)) = vx_target_0;

  nonlinear_mpc_controller_ptr_->updateInitialStates_x0(x0_initial_states_);

  // Set the nonlinear mpc performance variables.
  nmpc_performance_vars_.nmpc_lateral_error = error_states[0];  // ey
  nmpc_performance_vars_.nmpc_yaw_error = error_states[1];      // eyaw
  nmpc_performance_vars_.long_velocity_measured = current_velocity_ptr_->twist.twist.linear.x;
  nmpc_performance_vars_.nmpc_curvature = current_curvature_k0_;
}

void NonlinearMPCNode::predictDelayedInitialStateBy_MPCPredicted_Inputs(
  Model::state_vector_t &x0_predicted)
{
  if (inputs_buffer_.empty())
  {
    return;
  }

  using ns_utils::toUType;
  Model::input_vector_t uk{Model::input_vector_t::Zero()};
  Model::param_vector_t params(Model::param_vector_t::Zero());

  /**
   * Find the first command in the queue that has been applied to the system.
   * */

  /**
   * Update the first control entering system just at that time.
   * */
  auto timestamp = this->now();

  if (auto it_first_cmd_to_appy = std::find_if(
      inputs_buffer_.cbegin(), inputs_buffer_.cend(), sCommandTimeStampFind(timestamp));
    it_first_cmd_to_appy != inputs_buffer_.end())
  {
    first_control_entering_system_ = *it_first_cmd_to_appy;
  }

  /**
   * Apply the controls in the input queue using the their time durations.
   * */
  for (auto it = inputs_buffer_.begin(); it != std::prev(inputs_buffer_.end());)
  {
    if (rclcpp::Time(it->stamp) < rclcpp::Time(timestamp))
    {
      it = inputs_buffer_.erase(it);  // erase return next iterator
    } else
    {
      // Compute dt to the next command.
      auto dt_to_next = (rclcpp::Time(std::next(it)->stamp) - rclcpp::Time(it->stamp)).seconds();

      // Extract the controls from the input buffer.
      uk(toUType(VehicleControlIds::u_vx)) = static_cast<double>(it->longitudinal.acceleration);
      uk(toUType(VehicleControlIds::u_steering)) =
        static_cast<double>(it->lateral.steering_tire_angle);

      // d stands for  delayed states.
      auto const &sd0 = x0_predicted(ns_utils::toUType(VehicleStateIds::s));
      double kappad0{};  // curvature placeholder
      double vxk{};      // to interpolate the target speed.

      // Estimate the curvature.  The spline data is updated in the onTrajectory().
      if (auto const &&could_interpolate = interpolator_curvature_pws.Interpolate(sd0, kappad0);
        !could_interpolate)
      {
        RCLCPP_ERROR(
          get_logger(),
          "[nonlinear_mpc - predict initial state]: spline interpolator failed to compute  the  "
          "coefficients ...");
        return;
      }

      nonlinear_mpc_controller_ptr_->getSmoothVxAtDistance(sd0, vxk);

      params(toUType(VehicleParamIds::curvature)) = kappad0;
      params(toUType(VehicleParamIds::target_vx)) = vxk;

      nonlinear_mpc_controller_ptr_->simulateOneStep(uk, params, dt_to_next, x0_predicted);

      // Saturate steering.
      //-- ['xw', 'yw', 'psi', 's', 'e_y', 'e_yaw', 'Vx', 'delta', 'ay']
      nonlinear_mpc_controller_ptr_->applyControlConstraints(uk);
      nonlinear_mpc_controller_ptr_->applyStateConstraints(x0_predicted);

      // increase the iterator.
      ++it;
    }
  }

  // Set the current_predicted_s0_.  states = [x, y, psi, s, ey, epsi, v, delta]
  current_predicted_s0_ = x0_predicted(3);
  nonlinear_mpc_controller_ptr_->setCurrent_s0_predicted(current_predicted_s0_);

  // Predict the curvature and Ackermann steering angle.
  interpolator_curvature_pws.Interpolate(current_predicted_s0_, current_predicted_curvature_p0_);

  current_feedforward_steering_ = std::atan(current_predicted_curvature_p0_ * wheel_base_);

  // Set the predicted model params.
  predicted_model_params_ = params;  // [curvature, predicted longitudinal speed vxk]
}

void NonlinearMPCNode::getAverageMPCcomputeTime(double &avg_compute_time_in_sec) const
{
  avg_compute_time_in_sec = average_mpc_solve_time_;
}

visualization_msgs::msg::MarkerArray NonlinearMPCNode::createPredictedTrajectoryMarkers(
  std::string const &ns, std::string const &frame_id, std::array<double, 2> xy0,
  Model::trajectory_data_t const &td) const
{
  visualization_msgs::msg::MarkerArray marker_array;
  size_t const &&nX = td.nX();

  if (td.X.empty())
  {
    return marker_array;
  }

  auto marker = createDefaultMarker(
    "map", current_trajectory_ptr_->header.stamp, ns, 0,
    visualization_msgs::msg::Marker::LINE_STRIP, createMarkerScale(0.3, 0.1, 0.1),
    createMarkerColor(1.0, 0.0, 0.0, static_cast<float>(1.0)));

  for (unsigned int k = 0; k < nX; ++k)
  {
    geometry_msgs::msg::Point p;
    p.x = td.X.at(k)(0) + xy0[0];  // add back current position x.
    p.y = td.X.at(k)(1) + xy0[1];
    p.z = 0.0;
    marker.points.emplace_back(p);
  }

  // Push the marker.
  marker_array.markers.emplace_back(marker);

  // Create pose.
  visualization_msgs::msg::Marker marker_poses;

  marker_poses.header.frame_id = frame_id;
  marker_poses.header.stamp = current_trajectory_ptr_->header.stamp;
  marker_poses.ns = ns + "/poses";
  marker_poses.lifetime = rclcpp::Duration::from_seconds(0.1);
  marker_poses.type = visualization_msgs::msg::Marker::ARROW;
  marker_poses.action = visualization_msgs::msg::Marker::ADD;
  marker_poses.scale.x = 0.2;
  marker_poses.scale.y = 0.2;
  marker_poses.scale.z = 0.2;
  marker_poses.color.a = 1.0;  // Don't forget to set the alpha!

  auto color = createMarkerColor(
    0.0, static_cast<float>(0.8), static_cast<float>(0.8), static_cast<float>(1.8));
  marker_poses.color = color;

  for (unsigned int k = 0; k < nX; ++k)
  {
    // insert poses.
    marker_poses.id = static_cast<int>(k);
    marker_poses.pose.position.x = td.X.at(k)(0) + xy0[0];  // add back current position x.;
    marker_poses.pose.position.y = td.X.at(k)(1) + xy0[1];

    marker_poses.pose.position.z = 0.0;

    auto const &&yaw_angle = ns_utils::wrapToPi(td.X.at(k)(2));
    marker_poses.pose.orientation = ns_nmpc_utils::getQuaternionFromYaw(yaw_angle);
    marker_array.markers.emplace_back(marker_poses);
  }

  return marker_array;
}

void NonlinearMPCNode::publishPredictedTrajectories(
  std::string const &ns, std::string const &header_id) const
{
  auto const &td = nonlinear_mpc_controller_ptr_->getCurrentTrajectoryData();

  // Get the starting point: vehicle position projection on the trajectory.
  // In order not to deal with big numbers, prepare x, y w.r.t x0, y0.
  //  double xw0 = current_trajectory_ptr_->points.at(0).pose.position.x;
  //  double yw0 = current_trajectory_ptr_->points.at(0).pose.position.y;
  std::array<double, 2> const xy0{
    current_COG_pose_ptr_->pose.position.x, current_COG_pose_ptr_->pose.position.y};

  // Create visualization array for the predicted trajectory.
  if (auto visualization_prediction_markers =
      createPredictedTrajectoryMarkers(ns, header_id, xy0, td);
    !visualization_prediction_markers.markers.empty())
  {
    pub_debug_predicted_traj_->publish(visualization_prediction_markers);
  }

  /** @brief publish predicted trajectory as the autoware trajectory message */
  TrajectoryMsg predicted_traj;

  predicted_traj.header.stamp = current_trajectory_ptr_->header.stamp;
  predicted_traj.header.frame_id = current_trajectory_ptr_->header.frame_id;

  createAutowarePlanningTrajectoryMsg(td, xy0, &predicted_traj);

  pub_predicted_traj_->publish(predicted_traj);
}

void NonlinearMPCNode::publishClosestPointMarker(const std::string &ns) const
{
  visualization_msgs::msg::Marker marker = createLocationMarker(
    debug_data_.current_closest_pose, current_trajectory_ptr_->header.stamp, ns);
  pub_closest_point_debug_marker_->publish(marker);
}

void NonlinearMPCNode::setCurrentCOGPose(geometry_msgs::msg::PoseStamped const &ps)
{
  /** @brief current pose of the vehicle */
  std::array<double, 2> current_pose_xy{ps.pose.position.x, ps.pose.position.y};

  double const &&vehicle_yaw_angle = tf2::getYaw(ps.pose.orientation);

  // Compute the COG pose.
  auto const &&tangent_vec_of_vehicle = ns_utils::getTangentVector(vehicle_yaw_angle);

  /**
   * @brief offset error computations from rear to the center of gravity.
   * p_cog = p_rear_axle + lr * unit_tangent.
   * */

  std::array<double, 2> cog_pose_xy{0.0, 0.0};
  cog_pose_xy[0] = current_pose_xy[0] + params_node_.lr * tangent_vec_of_vehicle[0];
  cog_pose_xy[1] = current_pose_xy[1] + params_node_.lr * tangent_vec_of_vehicle[1];

  // Create a new pose from the previous pose and change its x and y.
  geometry_msgs::msg::PoseStamped pose_temp;

  pose_temp.header = ps.header;
  pose_temp.pose = ps.pose;

  pose_temp.pose.position.x = cog_pose_xy[0];
  pose_temp.pose.position.y = cog_pose_xy[1];

  current_COG_pose_ptr_ = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_temp);
}

ControlCmdMsg NonlinearMPCNode::createControlCommand(
  double const &ax, double const &vx, double const &steering_rate, double const &steering_val)
{
  ControlCmdMsg ctrl_cmd;

  ctrl_cmd.lateral.steering_tire_rotation_rate = static_cast<float>(steering_rate);
  ctrl_cmd.lateral.steering_tire_angle = static_cast<float>(steering_val);

  ctrl_cmd.longitudinal.acceleration = static_cast<float>(ax);
  ctrl_cmd.longitudinal.speed = static_cast<float>(vx);

  return ctrl_cmd;
}

/**
 * Gets the values of distance to stop, current ego speed and target velocity at the next point.
 * */
std::array<double, 3> NonlinearMPCNode::getDistanceEgoTargetSpeeds() const
{
  auto const &&distance_to_stopping_point = calcStopDistance(*idx_prev_wp_ptr_);
  auto const &current_vel = current_velocity_ptr_->twist.twist.linear.x;
  auto const &target_vel =
    current_trajectory_ptr_->points.at(*idx_next_wp_ptr_).longitudinal_velocity_mps;

  // state machine toggle(argument -> [distance_to_stop, vx_current, vx_next])
  return std::array<double, 3>{
    distance_to_stopping_point, current_vel, static_cast<double>(target_vel)};
}

void NonlinearMPCNode::setErrorReport(Model::state_vector_t const &x)
{
  /** set the computed error */
  current_error_report_.lateral_deviation_read = x(ns_utils::toUType(VehicleStateIds::ey));
  current_error_report_.heading_angle_error_read = x(ns_utils::toUType(VehicleStateIds::eyaw));
  current_error_report_.steering_read = x(ns_utils::toUType(VehicleStateIds::steering));
  current_error_report_.curvature_read = current_predicted_curvature_p0_;
}

void NonlinearMPCNode::publishErrorReport(ErrorReportMsg &error_rpt_msg) const
{
  error_rpt_msg.stamp = this->now();
  pub_nmpc_error_report_->publish(error_rpt_msg);
}

rcl_interfaces::msg::SetParametersResult NonlinearMPCNode::onParameterUpdate(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try
  {
    update_param(parameters, "cdob_ctrl_period", params_node_.control_period);
    update_param(parameters, "use_cdob", params_node_.use_cdob);
    update_param(parameters, "use_dob", params_node_.use_dob);
  }
    // transaction succeeds, now assign values
  catch (const rclcpp::exceptions::InvalidParameterTypeException &e)
  {
    result.successful = false;
    result.reason = e.what();
  }

  for (const auto &param : parameters)
  {
    RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", param.get_type_name().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());
  }

  return result;
}
}  // namespace ns_mpc_nonlinear

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ns_mpc_nonlinear::NonlinearMPCNode)
