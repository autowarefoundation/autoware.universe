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
  pub_nmpc_performance_ =
    create_publisher<NonlinearMPCPerformanceMsg>("~/debug/nmpc_predicted_performance_vars", 1);

  // Initialize the subscribers.
  sub_trajectory_ = create_subscription<TrajectoryMsg>("~/input/reference_trajectory", rclcpp::QoS{1},
                                                       std::bind(&NonlinearMPCNode::onTrajectory, this, _1));

  sub_velocity_ = create_subscription<VelocityMsg>(
    "~/input/current_velocity", rclcpp::QoS{1}, std::bind(&NonlinearMPCNode::onVelocity, this, _1));

  sub_vehicle_steering_ = create_subscription<SteeringMeasuredMsg>("~/input/current_steering", rclcpp::QoS{1},
                                                                   std::bind(
                                                                     &NonlinearMPCNode::onSteeringMeasured,
                                                                     this, _1));

  sub_com_delay_ = create_subscription<DelayCompensationRefs>("~/input/current_com_delay", rclcpp::QoS{1},
                                                              std::bind(
                                                                &NonlinearMPCNode::onCommDelayCompensation,
                                                                this, _1));

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
  auto nonlinear_nmpc_core =
    ns_nmpc_interface::NonlinearMPCController(vehicle_model_ptr, data_nmpc_core, params_lpv, params_optimization);

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

  /**
   *  @brief assigns a Kalman filter to estimate the initial states.
   * */

  ns_data::ParamsFilters params_filters;
  loadFilterParameters(params_filters);

  if (params_node_.use_sqrt_version)
  {
    kalman_filter_ptr_ =
      std::make_unique<ns_filters::KalmanUnscentedSQRT>(vehicle_model_ptr, params_node_.control_period);
  } else
  {
    kalman_filter_ptr_ = std::make_unique<ns_filters::KalmanUnscented>(vehicle_model_ptr, params_node_.control_period);
  }
  kalman_filter_ptr_->updateParameters(params_filters);

  /**
   * @brief assigns an observer to predict disturbance input in the speed
   * tracking.
   */
  // observer_speed_ = ns_filters::SimpleDisturbanceInputObserver(params_node_.observer_gain_speed);

  // Vehicle motion finite state machine.
  vehicle_motion_fsm_ = ns_states::VehicleMotionFSM(params_node_.stop_state_entry_ego_speed,
                                                    params_node_.stop_state_entry_target_speed,
                                                    params_node_.stop_state_keep_stopping_dist,
                                                    params_node_.will_stop_state_dist);

  // Initialize the control input queue.
  inputs_buffer_common_ = std::deque<std::array<double, 4 >>(params_node_.input_delay_discrete_nsteps,
                                                             std::array<double, 4>());

  // Initialize the timer.
  initTimer(params_node_.control_period);

  // DEBUG
  ns_utils::print("\n\nVehicle parameters is loaded");
  ns_utils::print("Wheelbase : ", params_vehicle.wheel_base);
  ns_utils::print("lr to cog : ", params_vehicle.lr);

  ns_utils::print("Steering tau : ", params_vehicle.steering_tau);
  ns_utils::print("Speed tau : ", params_vehicle.speed_tau);
  ns_utils::print("Node input delay : ", params_node_.input_delay_time);
  ns_utils::print("Use delay model: ", params_vehicle.use_delay_model, "\n\n");

  // Check if optimization parameters are read properly.
  ns_utils::print("\nOptimization parameters: ");
  ns_utils::print("'\nState weights Q ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_optimization.Q));

  ns_utils::print("\nState weights QN ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_optimization.QN));

  ns_utils::print("\nControl weights R ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_optimization.R));

  ns_utils::print("\nJerk weights Rj ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_optimization.Rj));

  ns_utils::print("\nxupper and lower ");
  ns_eigen_utils::printEigenMat(params_optimization.xlower);
  ns_eigen_utils::printEigenMat(params_optimization.xupper);

  ns_utils::print("\nu_upper and lower ");
  ns_eigen_utils::printEigenMat(params_optimization.ulower);
  ns_eigen_utils::printEigenMat(params_optimization.uupper);

  ns_utils::print("\nScaling and scaling range, x, u ");
  ns_eigen_utils::printEigenMat(params_optimization.xmin_for_scaling);
  ns_eigen_utils::printEigenMat(params_optimization.xmax_for_scaling);

  ns_eigen_utils::printEigenMat(params_optimization.umin_for_scaling);
  ns_eigen_utils::printEigenMat(params_optimization.umax_for_scaling);

  ns_utils::print_container(params_optimization.scaling_range);
  ns_utils::print("Number of nonlinear items : ", params_lpv.num_of_nonlinearities);

  ns_utils::print("Lyapunov Matrices Xs");
  for (size_t k = 0; k < params_lpv.num_of_nonlinearities; ++k)
  {
    ns_eigen_utils::printEigenMat(params_lpv.lpvXcontainer[k]);
  }

  ns_utils::print("Lyapunov Matrices Ys");
  for (size_t k = 0; k < params_lpv.num_of_nonlinearities; ++k)
  {
    ns_eigen_utils::printEigenMat(params_lpv.lpvYcontainer[k]);
  }

  // Check the if the scaling parameters are computed.
  ns_utils::print("Scaling matrices and vectors Sx, Sx_inv, Su, Su_inv, Cx, Cu :");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_optimization.Sx));
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_optimization.Sx_inv));
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_optimization.Su));
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_optimization.Su_inv));
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_optimization.Cx));
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_optimization.Cu));

  ns_utils::print("Kalman filter parameters V, W, P : ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_filters.Vsqrt));
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_filters.Wsqrt));
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_filters.Psqrt));

  ns_utils::print("\nUnscented Kalman filter parameters alpha, beta, kappa : ", params_filters.ukf_alpha,
                  params_filters.ukf_beta, params_filters.ukf_kappa, "\n\n");

  // end of debug.
}

// Destructor.
NonlinearMPCNode::~NonlinearMPCNode()
{
  ControlCmdMsg stop_cmd = getStopControlCommand();
  publishControlCommands(stop_cmd);
}

void NonlinearMPCNode::initTimer(double period_s)
{
  auto timer_callback = std::bind(&NonlinearMPCNode::onTimer, this);

  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback) >>(this->get_clock(), period_ns,
                                                                             std::move(timer_callback),
                                                                             this->get_node_base_interface()->get_context());

  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

// Callbacks.
void NonlinearMPCNode::onTimer()
{
  RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(), "Control frequency %g",
                                 params_node_.control_frequency);

  // ns_utils::print("Control Frequency : ", params_node_.control_frequency);
  // ns_utils::print("Logger string : ", get_logger().get_name());
  // end of debug

  // Update the current pose.
  updateCurrentPose();

  // Check Data Stream
  is_data_ready_ = isDataReady();
  if (!is_data_ready_)
  {
    // Warnings are generated in the isDataReady() method.
    // Send stop command to the vehicle.

    auto &&stop_cmd = getStopControlCommand();
    publishControlsAndUpdateVars(stop_cmd);
    // publishPerformanceVariables(stop_cmd);
    return;
  }

  // If the previous control command is not assigned, assign.
  if (!is_ctrl_cmd_prev_initialized_)
  {
    ctrl_cmd_prev_ = getInitialControlCommand();
    is_ctrl_cmd_prev_initialized_ = true;

    publishControlsAndUpdateVars(ctrl_cmd_prev_);
    // publishPerformanceVariables(ctrl_cmd_prev_);
    return;
  }

  /**
   * @brief MPC loop : THIS SCOPE is important to update and maintain the MPC_CORE object. The
   * operations are in an order.
   * */
  double const &&timer_mpc_step = ns_utils::tic();

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

  current_fsm_state_ = vehicle_motion_fsm_.getCurrentState();
  // ns_utils::print("Finite state machine state numbers : ",
  // ns_states::as_integer(current_fsm_state_));

  RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(), "\n[mpc_nonlinear] %s",
                                 vehicle_motion_fsm_.fsmMessage().c_str());

  //    if (current_fsm_state_ == ns_states::motionStateEnums::isAtCompleteStop ||
  //        current_fsm_state_ == ns_states::motionStateEnums::isInEmergency)
  if (current_fsm_state_ == ns_states::motionStateEnums::isAtCompleteStop)
  {
    // Reset the input que. [ax, vx, steering_rate, steering]
    for (auto &value : inputs_buffer_common_)
    {
      value = std::array<double, 4>{0.0, 0.0, 0.0, 0.0};
    }

    kalman_filter_ptr_->reset();
    RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(), "\n[mpc_nonlinear] %s",
                                   vehicle_motion_fsm_.fsmMessage().c_str());

    auto &&control_cmd = getStopControlCommand();

    // Prevent reverse motion when stopped.
    if (current_velocity_ptr_->twist.twist.linear.x < EPS)
    {
      control_cmd.longitudinal.acceleration = 0.0;

      // Reset Kalman filter speed
      kalman_filter_ptr_->resetStateEstimate(ns_utils::toUType(VehicleStateIds::vx), 0.0);
    }

    publishControlsAndUpdateVars(control_cmd);
    // publishPerformanceVariables(control_cmd);
    return;
  }

  // Prepare the x0_predicted placeholder by fetching the initial states into it.
  x0_predicted_.setZero();
  nonlinear_mpc_controller_ptr_->getInitialState(x0_predicted_);

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
    if (auto const &&is_initialized =
        nonlinear_mpc_controller_ptr_->initializeTrajectories(interpolator_curvature_pws,
                                                              use_linear_initialization); !is_initialized)
    {
      // vehicle_motion_fsm_.setEmergencyFlag(true);

      RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
                                     "[mpc_nonlinear] Couldn't initialize the LPV controller ... %s \n",
                                     vehicle_motion_fsm_.fsmMessage().c_str());

      // Publish previous control command.
      publishControlsAndUpdateVars(ctrl_cmd_prev_);
      // publishPerformanceVariables(ctrl_cmd_prev_);
      return;
    }

    // is_moving_state_ = true;
    // ns_utils::print("the NMPC is initialized successfully  ...");

    // DEBUG
    // Publish the predicted trajectories.
    // publishPredictedTrajectories("predicted_trajectory",
    // current_trajectory_ptr_->header.frame_id); Debug
  }

  //  if (vehicle_motion_fsm_.isReinitializationRequired())
  //  {
  //    auto const &&is_reinitialized =
  //      nonlinear_mpc_controller_ptr_->reInitializeTrajectories(interpolator_curvature_pws);
  //
  //    RCLCPP_WARN_SKIPFIRST_THROTTLE(
  //      get_logger(), *get_clock(), (1000ms).count(),
  //      "[mpc_nonlinear] "
  //      "Trajectories are re-initialized %s \n",
  //      is_reinitialized ? "true" : "false");
  //
  //    if (is_reinitialized)
  //    {
  //      vehicle_motion_fsm_.setReinitializationFlag(false);
  //    }
  //  }

  /**
   * @brief
   * - Simulate the controls starting from the delayed initial state.
   * - Apply the predicted control to the vehicle model starting from predicted initial state and replace the
   * - measured initial state with the predicted initial state.
   */

  /**
   * All inputs of the NMPC [vx, steering] are applied and the system is simulated.
   * */
  nonlinear_mpc_controller_ptr_->simulateControlSequenceByPredictedInputs(x0_predicted_, interpolator_curvature_pws);

  // Model::input_vector_t u_model_solution_; // [velocity input - m/s, steering input - rad]
  // If we choose to use MPC. Get solution from OSQP into the traj_data_.
  // Prepare the solution vector place holder and set it zero before fetching it.
  u_solution_.setZero();

  // use the NMPC.
  // Solve the problem, update the trajectory_data.

  bool is_mpc_solved_tmp{true};
  is_mpc_solved_ = false;

  for (size_t k = 0; k < params_node_.number_of_sqp_iterations; ++k)
  {
    auto &&is_mpc_solved_k = nonlinear_mpc_controller_ptr_->solveNMPC_problem(interpolator_curvature_pws);
    is_mpc_solved_tmp = is_mpc_solved_tmp && is_mpc_solved_k;
  }

  // Store mpc solution result.
  is_mpc_solved_ = is_mpc_solved_tmp;

  if (!is_mpc_solved_)
  {
    // vehicle_motion_fsm_.setEmergencyFlag(true);

    RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
                                   "[nonlinear_mpc]: Could not solve the mpc problem ... %s \n",
                                   vehicle_motion_fsm_.fsmMessage().c_str());

    // ROS_ERROR("[nonlinear_mpc]: Could not solve the mpc problem ... ");

    // Publish stopping command.
    auto stop_cmd = getStopControlCommand();
    publishControlsAndUpdateVars(stop_cmd);

    return;
  }

  //    ns_utils::print("the NMPC problem is solved ...");

  // Get MPC controls [acc, steering rate]
  nonlinear_mpc_controller_ptr_->getControlSolutions(u_solution_);  // [acc, steering_rate]

  // If MPC is solved.
  nonlinear_mpc_controller_ptr_->shiftControls();

  // Compute MPC model predicted longitudinal speed by Euler integration.
  // auto const mpc_vx = current_velocity_ptr_->twist.twist.linear.x + u_solution_(0) * params_node_.control_period;
  auto const mpc_vx = nonlinear_mpc_controller_ptr_->getEstimatedVxControl();

  // Compute the steering rate by numerical differentiation.
  if (params_node_.use_dob && current_comm_delay_ptr_)
  {
    auto dob_steering_ff = current_comm_delay_ptr_->steering_dob;
    u_solution_(1) += dob_steering_ff;
  }

  auto const
    steering_rate = (u_solution_(1) - ctrl_cmd_prev_.lateral.steering_tire_angle) / params_node_.control_period;

  // createControlCommand(ax, vx, steer_rate, steer_val)
  ControlCmdMsg control_cmd{};

  // Set the control command.

  control_cmd = createControlCommand(u_solution_(0),  /* ax */
                                     mpc_vx,          /* vx input */
                                     steering_rate,   /* steering_rate */
                                     u_solution_(1)); /* steering input */

  // Publish the control command.
  publishControlsAndUpdateVars(control_cmd);

  // Publish NMPC performance variables.
  // publishPerformanceVariables(control_cmd);

  // Publish the predicted trajectories.
  publishPredictedTrajectories("predicted_trajectory", current_trajectory_ptr_->header.frame_id);

  // Maintain the input and steering_buffer for predicting the initial states.
  // [acc, vx, steering_rate, steering value]
  std::array<double, 4> all_control_cmds{control_cmd.longitudinal.acceleration,            // ax
                                         control_cmd.longitudinal.speed,                   // vx
                                         control_cmd.lateral.steering_tire_rotation_rate,  // steering rate
                                         control_cmd.lateral.steering_tire_angle           // steering angle
  };

  //  ns_utils::print("\nControls put in the que :");
  //
  //  ns_utils::print("ax control", all_control_cmds[0]);
  //  ns_utils::print("vx control", all_control_cmds[1]);
  //  ns_utils::print("steering rate control", all_control_cmds[2]);
  //  ns_utils::print("steering control", all_control_cmds[3]);

  // Store the first control signal to be applied to the vehicle in the kalman
  // control variable. input_buffer_[acc, vx, steering_rate, steering value]

  // u0_kalman_(0) = inputs_buffer_common_.front()[0];  // ax
  // u0_kalman_(1) = inputs_buffer_common_.front()[3];  // steering angle

  // Maintain the queue.
  inputs_buffer_common_.pop_front();
  inputs_buffer_common_.emplace_back(all_control_cmds);
  // end of NMPC loop.

  auto const &&current_mpc_solve_time_msec = ns_utils::toc(timer_mpc_step);

  // convert milliseconds to seconds.
  auto &&current_mpc_solve_time_sec = current_mpc_solve_time_msec * 1e-3;
  average_mpc_solve_time_ = ns_utils::exponentialMovingAverage(average_mpc_solve_time_, 20, current_mpc_solve_time_sec);

  // Set NMPC avg_mpc_computation_time.
  nonlinear_mpc_controller_ptr_->setCurrentAvgMPCComputationTime(average_mpc_solve_time_);

  ns_utils::print("\nOne step MPC step takes time to compute in milliseconds : ", current_mpc_solve_time_msec);
  ns_utils::print("Average MPC solve time takes time to compute in milliseconds : ",
                  average_mpc_solve_time_ * 1000, "\n");

  // ------------- END of the MPC loop. -------------
  // Set Debug Marker next waypoint
  debug_data_.current_closest_pose = current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose;

  // Publish the closest point of the trajectory to the vehicle.
  publishClosestPointMarker("closest_point");

  // DEBUG
  // ns_utils::print("Current use cdob parameter : ", params_node_.use_cdob);

  //  if (current_comm_delay_ptr_)
  //  {
  //    ns_utils::print("Current delay compensation steering ref : ",
  //                    current_comm_delay_ptr_->steering_compensation_ref);
  //
  //    ns_utils::print("Current delay compensation ey ref : ",
  //                    current_comm_delay_ptr_->lateral_deviation_error_compensation_ref);
  //
  //    ns_utils::print("Current delay compensation eyaw ref : ",
  //                    current_comm_delay_ptr_->heading_angle_error_compensation_ref);
  //  }


  //  ns_utils::print("Wheelbase : ", wheel_base_);
  //  ns_utils::print("\nCurrent Pose : ");
  //  ns_utils::print("x, y, z : ", current_pose_ptr_->pose.position.x, current_pose_ptr_->pose.position.y,
  //                  current_pose_ptr_->pose.position.z);

  //  auto const &td = nonlinear_mpc_controller_ptr_->getCurrentTrajectoryData();
  //  auto &&Xtemp = ns_eigen_utils::getTrajectory(td.X);
  //  auto &&Utemp = ns_eigen_utils::getTrajectory(td.U);
  //
  //  ns_utils::print("\n[in timer] Initialized LPV trajectories :  [x, y, psi, s, ey, epsi, vx, delta, vy]");
  //  ns_eigen_utils::printEigenMat(Xtemp.transpose());  //  [x, y, psi, s, ey, epsi, vx, delta, vy]
  //
  //  ns_utils::print("\n [in timer] Initialized LPV trajectories U : ");
  //  ns_eigen_utils::printEigenMat(Utemp.transpose());

  // end of DEBUG
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
    RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (5000ms).count(),
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
  const double zero_velocity = EPS;
  const double origin_velocity = current_trajectory_ptr_->points.at(prev_waypoint_index).longitudinal_velocity_mps;

  double stop_dist{};

  // Get distances vector from current trajectory computed in the NMPC core.
  std::vector<double> scomputed{};
  nonlinear_mpc_controller_ptr_->getPlannerTravelledDistanceVector(scomputed);

  // Find the zero velocity waypoint ahead.
  if (std::fabs(origin_velocity) > zero_velocity)
  {
    auto it = std::find_if(current_trajectory_ptr_->points.cbegin() + static_cast<int>(prev_waypoint_index),
                           current_trajectory_ptr_->points.cend(), [&zero_velocity](auto &point)
                           {
                             return std::fabs(point.longitudinal_velocity_mps) < zero_velocity;
                           });

    if (it == current_trajectory_ptr_->points.cend())
    {
      stop_dist = scomputed.back() - scomputed[prev_waypoint_index];

    } else
    {
      auto const
        &&index_of_zero_vel_point = static_cast<size_t>(std::distance(current_trajectory_ptr_->points.cbegin(), it));

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
    RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
                                   "[mpc_nonlinear] Waiting for the current pose ...");
    return false;
  }

  if (!current_trajectory_ptr_)
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
                                   "[mpc_nonlinear] Waiting for the current trajectory ... ");
    return false;
  }

  if (!current_velocity_ptr_)
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
                                   "[mpc_nonlinear] Waiting for the current speed ...");
    return false;
  }

  if (!current_steering_ptr_)
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
                                   "[mpc_nonlinear] Waiting for the current steering measurement ...");
    return false;
  }

  return true;
}

void NonlinearMPCNode::publishControlCommands(ControlCmdMsg &ctrl_cmd) const
{
  ctrl_cmd.stamp = this->now();
  pub_control_cmd_->publish(ctrl_cmd);
}

void NonlinearMPCNode::publishControlsAndUpdateVars(ControlCmdMsg &ctrl_cmd)
{
  // publish the control command.
  publishControlCommands(ctrl_cmd);

  // Publish NMPC performance variables.
  publishPerformanceVariables(ctrl_cmd);

  // Update the node variables
  ctrl_cmd_prev_ = ctrl_cmd;
  u_previous_solution_ = u_solution_;  // required for the disturbance observer

  // Update inputs for the Kalman model.
  u0_kalman_(0) = inputs_buffer_common_.front()[0];  // ax
  u0_kalman_(1) = inputs_buffer_common_.front()[3];  // steering angle
}

void NonlinearMPCNode::publishPerformanceVariables(ControlCmdMsg const &control_cmd)
{
  // Set nmpc performance variables.
  nmpc_performance_vars_.stamp = this->now();
  nmpc_performance_vars_.steering_angle_input = control_cmd.lateral.steering_tire_angle;
  nmpc_performance_vars_.steering_angle_rate_input = control_cmd.lateral.steering_tire_rotation_rate;

  nmpc_performance_vars_.acceleration_input = control_cmd.longitudinal.acceleration;
  nmpc_performance_vars_.long_velocity_input = control_cmd.longitudinal.speed;

  // Set the measured vehicle steering.
  double &&current_steering =
    is_data_ready_ ? static_cast<double>(current_steering_ptr_->steering_tire_angle) : 0.0;
  nmpc_performance_vars_.steering_angle_measured = current_steering;

  // Set the obj_val.
  double &&value_function_value =
    is_mpc_solved_ ? nonlinear_mpc_controller_ptr_->getObjectiveValue() : 0.0;
  nmpc_performance_vars_.nmpc_value = value_function_value;

  // Finite state machine state.
  nmpc_performance_vars_.fsm_state = as_integer(current_fsm_state_);

  // Average computation time.
  nmpc_performance_vars_.avg_nmpc_computation_time = average_mpc_solve_time_ * 1000;  // s->ms

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

ControlCmdMsg NonlinearMPCNode::getStopControlCommand()
{
  ControlCmdMsg cmd;

  // Steering values.
  cmd.lateral.steering_tire_angle = 0.0;
  cmd.lateral.steering_tire_rotation_rate = 0.0;

  // Longitudinal control values.
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = -1.5;

  return cmd;
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
  params_node_.use_acceleration_inputs = declare_parameter<bool>("use_acceleration_inputs", true);

  params_node_.use_kalman = declare_parameter<bool>("kalman_filters.use_kalman", true);
  params_node_.use_sqrt_version = declare_parameter<bool>("kalman_filters.use_sqrt_version", false);

  // Stop state parameters.
  params_node_.stop_state_entry_ego_speed = declare_parameter<double>("stop_state_entry_ego_speed", 0.2);

  params_node_.stop_state_entry_target_speed = declare_parameter<double>("stop_state_entry_target_speed", 0.5);
  params_node_.stop_state_keep_stopping_dist = declare_parameter<double>("stop_state_keep_stopping_dist", 0.5);
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

  if (params_node_.use_dob)
  {
    params_node_.use_cdob = true;  // DOB depends on the CDOB states, cannot be used alone.
  }
}

void NonlinearMPCNode::loadFilterParameters(ns_data::ParamsFilters &params_filters)
{
  // Kalman filter parameters.
  params_filters.Vsqrt.setZero();
  std::vector<double> temp(Model::state_dim);
  std::vector<double> default_vec{0.2, 0.2, 0.05, 0.2, 0.05, 0.02, 0.15, 0.03, 0.05};
  temp = declare_parameter<std::vector<double >>("kalman_filters.Vprocess", default_vec);
  params_filters.Vsqrt.diagonal() = Model::state_vector_t::Map(temp.data());

  params_filters.Wsqrt.setZero();
  temp.clear();
  temp.reserve(Model::state_dim);
  default_vec = std::vector<double>{0.4, 0.4, 0.08, 0.3, 0.15, 0.07, 0.01, 0.05, 0.2};
  temp = declare_parameter<std::vector<double >>("kalman_filters.Wmeasurement", default_vec);
  params_filters.Wsqrt.diagonal() = Model::state_vector_t::Map(temp.data());

  // Updated covariance matrix.
  params_filters.Psqrt.setZero();
  temp.clear();
  temp.reserve(Model::state_dim);
  default_vec = std::vector<double>{0.4, 0.4, 0.08, 0.3, 0.15, 0.07, 0.1, 0.07, 0.25};
  temp = declare_parameter<std::vector<double >>("kalman_filters.Pkalman", default_vec);
  params_filters.Psqrt.diagonal() = Model::state_vector_t::Map(temp.data());

  // UKF specific parameters.
  params_filters.ukf_alpha = declare_parameter("kalman_filters.alpha", 0.9);
  params_filters.ukf_beta = declare_parameter("kalman_filters.beta", 2.0);
}

void NonlinearMPCNode::loadVehicleParameters(ns_models::ParamsVehicle &params_vehicle)
{
  params_vehicle.wheel_base = wheel_base_;
  params_vehicle.lr = params_node_.lr;

  params_vehicle.steering_tau = declare_parameter<double>("steering_time_constant", 0.27);
  params_vehicle.speed_tau = declare_parameter<double>("speed_time_constant", 0.61);
  params_vehicle.use_delay_model = params_node_.use_delay_sim_model;
}

void NonlinearMPCNode::loadNMPCoreParameters(ns_data::data_nmpc_core_type_t &data_nmpc_core,
                                             ns_data::param_lpv_type_t &params_lpv,
                                             ns_data::ParamsOptimization &params_optimization)
{
  data_nmpc_core.wheel_base = wheel_base_;

  // mpc_timestep_dt is already set in the constructor.
  data_nmpc_core.input_delay_time = params_node_.input_delay_time;

  // Set reference speed scaling factor for a simple feed-forward control.
  data_nmpc_core.feedforward_speed_set_point_scale =
    declare_parameter<double>("feedforward_speed_set_point_scale", 1.0);


  // Load optimization parameters.
  // State and control weights. Q. Reads only the diagonal terms .
  std::vector<double> temp(Model::state_dim);
  std::vector<double> default_vec{0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0., 0.0};
  temp = declare_parameter<std::vector<double> >("state_weights", default_vec);
  params_optimization.Q.diagonal() = Model::state_vector_t::Map(temp.data());

  // QN
  temp.clear();
  temp.reserve(Model::state_dim);
  default_vec = std::vector<double>{0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0};
  temp = declare_parameter<std::vector<double> >("state_weights_terminal", default_vec);
  params_optimization.QN.diagonal() = Model::state_vector_t::Map(temp.data());

  // R - Control weights.
  temp.clear();
  temp.reserve(Model::input_dim);
  default_vec = std::vector<double>{0.001, 0.0001};
  temp = declare_parameter<std::vector<double> >("control_weights", default_vec);
  params_optimization.R.diagonal() = Model::input_vector_t::Map(temp.data());

  // Rj - Jerk weights.
  temp.clear();
  temp.reserve(Model::input_dim);
  default_vec = std::vector<double>{1., 1.};
  temp = declare_parameter<std::vector<double> >("jerk_weights", default_vec);
  params_optimization.Rj.diagonal() = Model::input_vector_t::Map(temp.data());

  // State and input bounds. xlower bound.
  temp.clear();
  temp.reserve(Model::state_dim);
  default_vec =
    std::vector<double>{-kInfinity, -kInfinity, -kInfinity, -kInfinity, -2.0, -1.0, 0.0, -0.69, -kInfinity};
  temp = declare_parameter<std::vector<double> >("xlower", default_vec);
  params_optimization.xlower = Model::state_vector_t::Map(temp.data());

  // State and input bounds. xupper bound.
  temp.clear();
  temp.reserve(Model::state_dim);
  default_vec =
    std::vector<double>{kInfinity, kInfinity, kInfinity, kInfinity, 2.0, 1.0, 25.0, 0.69, kInfinity};
  temp = declare_parameter<std::vector<double> >("xupper", default_vec);
  params_optimization.xupper = Model::state_vector_t::Map(temp.data());

  // State and input bounds. ulower bound.
  temp.clear();
  temp.reserve(Model::input_dim);
  default_vec = std::vector<double>{-30., -1.};
  temp = declare_parameter<std::vector<double> >("ulower", default_vec);
  params_optimization.ulower = Model::input_vector_t::Map(temp.data());

  temp.clear();
  temp.reserve(Model::input_dim);
  default_vec = std::vector<double>{30., 1.};
  temp = declare_parameter<std::vector<double> >("uupper", default_vec);
  params_optimization.uupper = Model::input_vector_t::Map(temp.data());

  // xmax bound: xmax is used for scaling, whereas xlower is related to whether there is an upper bound.
  temp.clear();
  temp.reserve(Model::state_dim);
  default_vec = std::vector<double>{-50., -50., -3.14, 0.0, -3.0, -1.0, 0.0, -0.69, -5.0};
  temp = declare_parameter<std::vector<double> >("xmin_for_scaling", default_vec);
  params_optimization.xmin_for_scaling = Model::state_vector_t::Map(temp.data());

  temp.clear();
  temp.reserve(Model::state_dim);
  default_vec = std::vector<double>{50., 50., 3.14, 40.0, 3.0, 1.0, 10.0, 0.69, 5.0};
  temp = declare_parameter<std::vector<double> >("xmax_for_scaling", default_vec);
  params_optimization.xmax_for_scaling = Model::state_vector_t::Map(temp.data());

  temp.clear();
  temp.reserve(Model::input_dim);
  default_vec = std::vector<double>{-50., -1.0};
  temp = declare_parameter("umin_for_scaling", default_vec);
  params_optimization.umin_for_scaling = Model::input_vector_t::Map(temp.data());

  temp.clear();
  temp.reserve(Model::input_dim);
  default_vec = std::vector<double>{50., 1.0};
  temp = declare_parameter<std::vector<double> >("umax_for_scaling", default_vec);
  params_optimization.umax_for_scaling = Model::input_vector_t::Map(temp.data());

  // Load the normalization scaling range.
  params_optimization.scaling_range =
    declare_parameter<std::vector<double >>("scaling_range", std::vector<double>{-1., 1.});

  // OSQP parameters
  params_optimization.osqp_warm_start = declare_parameter<bool>("osqp_warm_start", true);
  params_optimization.osqp_polishing = declare_parameter<bool>("osqp_polishing", true);
  params_optimization.osqp_scaling = declare_parameter<bool>("osqp_scaling", true);

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

  //	// If we use the first order steering and vx models, LPV Lyap matrices are different.
  //	if (params_node_.use_delay_sim_model)
  //	{
  //		labelX_tag = "Xd";  // delay Lyapunov matrices.
  //		labelY_tag = "Yd";
  //	}

  for (auto k = 0; k < params_lpv.num_of_nonlinearities; k++)
  {
    std::vector<double> tempX;
    tempX.reserve(nx * nx);

    std::vector<double> tempY;
    tempY.reserve(nx * nu);

    auto labelX = labelX_tag + std::to_string(k + 1);
    auto labelY = labelY_tag + std::to_string(k + 1);

    tempX = declare_parameter<std::vector<double >>(labelX);
    tempY = declare_parameter<std::vector<double >>(labelY);

    auto tempXmat = Model::state_matrix_X_t::Map(tempX.data());
    auto tempYmat = Model::input_matrix_Y_t::Map(tempY.data());

    params_lpv.lpvXcontainer.emplace_back(tempXmat);
    params_lpv.lpvYcontainer.emplace_back(tempYmat);
  }
}

/**
 *@brief : The state and control variables are scaled to a predefined interval which is typically [-1, 1].
 * xmax = a*xmax_s + b
 * xmin = a*xmin_s + b where xmin, xmax are the original magnitude and _s tag is the scaling interval.
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

  params.Su.diagonal() = (params.umax_for_scaling - params.umin_for_scaling) / (xuhatmax - xuhatmin);

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
    double const &&val_low = (params.xlower(k) >= kInfinity - EPS || params.xlower(k) <= -kInfinity + EPS)
                             ? params.xlower(k) : (params.xlower(k) - params.Cx(k)) * params.Sx_inv.diagonal()(k);

    double const &&val_up = (params.xupper(k) >= kInfinity - EPS || params.xupper(k) <= -kInfinity + EPS)
                            ? params.xupper(k) : (params.xupper(k) - params.Cx(k)) * params.Sx_inv.diagonal()(k);

    params.xlower_scaled(k) = val_low;

    params.xupper_scaled(k) = val_up;
  }

  for (Eigen::Index k = 0; k < Model::input_dim; ++k)
  {
    double const &&val_low = (params.ulower(k) >= kInfinity - EPS || params.ulower(k) <= -kInfinity + EPS)
                             ? params.ulower(k) : (params.ulower(k) - params.Cu(k)) * params.Su_inv.diagonal()(k);

    double const &&val_up = (params.uupper(k) >= kInfinity - EPS || params.uupper(k) <= -kInfinity + EPS)
                            ? params.uupper(k) : (params.uupper(k) - params.Cu(k)) * params.Su_inv.diagonal()(k);

    params.ulower_scaled(k) = val_low;
    params.uupper_scaled(k) = val_up;
  }

  // DEBUG
  // end of debug
}

// Callbacks.
void NonlinearMPCNode::onTrajectory(const TrajectoryMsg::SharedPtr msg)
{
  current_trajectory_ptr_ = msg;

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

  current_trajectory_size_ = msg->points.size();

  // Resample the planning trajectory and store in the MPCdataTrajectoryVectors.

  if (bool const &&is_resampled = resampleRawTrajectoriesToaFixedSize(); !is_resampled)
  {
    RCLCPP_ERROR(get_logger(), "[nonlinear_mpc] Could not resample the trajectory ...");
    return;
  }

  // DEBUG
  RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(), "In the node onTrajectory() ");

  //  ns_utils::print("The current number of trajectory is : ", current_trajectory_size_);
  // end of DEBUG
}

void NonlinearMPCNode::onVelocity(VelocityMsg::SharedPtr msg)
{
  current_velocity_ptr_ = msg;

  // DEBUG
  RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(), "In the node onVelocity() ");
  // end of DEBUG
}

void NonlinearMPCNode::onCommDelayCompensation(const DelayCompensationRefs::SharedPtr msg)
{
  current_comm_delay_ptr_ = msg;

  //	if (current_comm_delay_ptr_)
  //	{
  //		RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
  //												 "In lateral control, ey refs read
  //%4.2f
  //",
  //msg->lateral_deviation_error_compensation_ref);
  //
  //		RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
  //												 "In lateral control, eyaw refs read
  //%4.2f
  //",
  //msg->heading_angle_error_compensation_ref);
  //
  //		RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
  //												 "In lateral control, eyaw refs read
  //%4.2f
  //",
  //msg->steering_compensation_ref);
  //	}
}

void NonlinearMPCNode::onSteeringMeasured(SteeringMeasuredMsg::SharedPtr msg)
{
  current_steering_ptr_ = msg;

  // DEBUG
  RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(), "In the node onSteering() ");
  // end of DEBUG
}

bool NonlinearMPCNode::isValidTrajectory(const TrajectoryMsg &msg_traj) const
{
  bool const &&check_condition = std::all_of(std::cbegin(msg_traj.points), std::cend(msg_traj.points), [](auto point)
  {
    const auto &p = point.pose.position;
    const auto &o = point.pose.orientation;
    const auto &vx = point.longitudinal_velocity_mps;
    const auto &vy = point.lateral_velocity_mps;
    const auto &wxf = point.front_wheel_angle_rad;
    const auto &wxr = point.rear_wheel_angle_rad;

    return static_cast<bool>(isfinite(p.x) && isfinite(p.y) && isfinite(p.z) && isfinite(o.x)
                             && isfinite(o.y) && isfinite(o.z) && isfinite(o.w) && isfinite(vx) &&
                             isfinite(vy) && isfinite(wxf) &&
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
    RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
                                   "[mpc_nonlinear] MPCdataTrajectoryVectors are not assigned properly ...");

    return false;
  }

  // Add extra points to the end of the trajectory for monotonicity.
  //    if (will_stop_state_)
  //    {
  //        mpc_traj_raw.addExtraEndPoints(average_mpc_solve_time_);
  //    }

  nonlinear_mpc_controller_ptr_->setMPCtrajectoryRawVectorsPtr(mpc_traj_raw);

  // --------------- Fill the Eigen reference_map_sxyz --------------------------

  // !<-@brief smooth map reference of [MPC_MAP_SMOOTHER_OUT x 4]
  // !<-@brief [s, x, y, z ] size [MPC_MAP_SMOOTHER_IN x 4]
  map_matrix_in_t reference_map_sxyz(map_matrix_in_t::Zero());

  if (bool const &&is_resampled = makeFixedSizeMat_sxyz(mpc_traj_raw, reference_map_sxyz);
    !is_resampled)
  {
    RCLCPP_ERROR(get_logger(),
                 "[mpc_nonlinear - resampleRawTrajectoryToAFixedSize ] Could not interpolate the "
                 "coordinates ...");
    return false;
  }

  // ------------------- Smooth Trajectories ---------------------------------------
  // Create MPCtraj smooth_ref_traj.
  bool const &&is_smoothed = createSmoothTrajectoriesWithCurvature(mpc_traj_raw, reference_map_sxyz);

  // DEBUG
  //  ns_utils::print("Eigen resampled map : ");
  //  ns_eigen_utils::printEigenMat(reference_map_sxyz);
  //  ns_utils::print("Raw trajectory time vector : ");
  //  ns_utils::print_container(mpc_traj_raw.t);
  // end of debug

  return is_smoothed;
}

bool NonlinearMPCNode::makeFixedSizeMat_sxyz(const ns_data::MPCdataTrajectoryVectors &mpc_traj_raw,
                                             map_matrix_in_t &fixed_map_ref_sxyz)
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
  std::vector<double> s_fixed_size_coordinate = ns_utils::linspace(initial_distance, final_distance, map_in_fixed_size);

  /**
   * Make sure that, the base coordinates, s and t are not out of range of the derived coordinates.
   * */
  auto dif_s0 = initial_distance - s_fixed_size_coordinate[0];
  auto dif_send = final_distance - s_fixed_size_coordinate.back();

  ns_utils::print("diff s0, and diff send", dif_s0, dif_send);

  /**
   * @brief Create a piece-wise cubic interpolator for x and y.
   * */
  ns_splines::InterpolatingSplinePCG interpolator_spline_pws(3);  // piecewise

  /**
   * @brief Create a piece-wise linear interpolator for the rest of the coordinates.
   * */
  ns_splines::InterpolatingSplinePCG interpolator_linear(1);

  // Interpolated vector containers.
  std::vector<double> xinterp;
  std::vector<double> yinterp;
  std::vector<double> zinterp;

  // Resample the varying size raw trajectory into a fixed size trajectory points.
  auto const &&is_interpolated_x =
    interpolator_spline_pws.Interpolate(mpc_traj_raw.s, mpc_traj_raw.x, s_fixed_size_coordinate, xinterp);

  auto const &&is_interpolated_y =
    interpolator_spline_pws.Interpolate(mpc_traj_raw.s, mpc_traj_raw.y, s_fixed_size_coordinate, yinterp);

  auto const &&is_interpolated_z =
    interpolator_linear.Interpolate(mpc_traj_raw.s, mpc_traj_raw.z, s_fixed_size_coordinate, zinterp);

  /**
   * @brief save into the reference map which accepts the resampled fixed size coordinates.
   * */
  fixed_map_ref_sxyz.col(0) = map_vector_in_t::Map(s_fixed_size_coordinate.data());  // snew
  fixed_map_ref_sxyz.col(1) = map_vector_in_t::Map(xinterp.data());                  // xnew
  fixed_map_ref_sxyz.col(2) = map_vector_in_t::Map(yinterp.data());                  // ynew
  fixed_map_ref_sxyz.col(3) = map_vector_in_t::Map(zinterp.data());                  // znew

  // DEBUG
  //
  // ns_utils::print("mpc raw : s ");
  // ns_utils::print_container(mpc_traj_raw.s);
  // ns_utils::print("Fixed size s ");
  // ns_utils::print_container(s_fixed_size_coordinate);
  //
  // ns_utils::print("mpc raw : t ");
  // ns_utils::print_container(mpc_traj_raw.t);
  //
  // ns_utils::print("mpc raw : x ");
  // ns_utils::print_container(mpc_traj_raw.x);
  //
  // ns_utils::print("Fixed size : x ");
  // ns_utils::print_container(xinterp);
  //
  // ns_utils::print("mpc raw : y ");
  // ns_utils::print_container(mpc_traj_raw.y);
  //
  // ns_utils::print("Fixed size : y ");
  // ns_utils::print_container(yinterp);
  //
  // ns_utils::print("mpc raw : z ");
  // ns_utils::print_container(mpc_traj_raw.z);
  //
  // ns_utils::print("Fixed size : z ");
  // ns_utils::print_container(zinterp);
  //
  // ns_utils::print("mpc raw : yaw ");
  // ns_utils::print_container(mpc_traj_raw.yaw);
  //

  // ns_utils::print("mpc raw .vx ");
  // ns_utils::print_container(mpc_traj_raw.vx);

  // end of DEBUG

  return is_interpolated_x && is_interpolated_y && is_interpolated_z;
}

bool NonlinearMPCNode::createSmoothTrajectoriesWithCurvature(ns_data::MPCdataTrajectoryVectors const &mpc_traj_raw,
                                                             map_matrix_in_t const &fixed_map_ref_sxyz)
{


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
  auto const &&curvature = ns_eigen_utils::Curvature(rdot_interp, rddot_interp);

  // Create smooth MPCtraj given, s, x, y, v and curvature.
  std::vector<double> s_smooth_vect(interpolated_map.col(0).data(),
                                    interpolated_map.col(0).data() + map_out_mpc_size);

  std::vector<double> x_smooth_vect(interpolated_map.col(1).data(),
                                    interpolated_map.col(1).data() + map_out_mpc_size);

  std::vector<double> y_smooth_vect(interpolated_map.col(2).data(),
                                    interpolated_map.col(2).data() + map_out_mpc_size);

  std::vector<double> z_smooth_vect(interpolated_map.col(3).data(),
                                    interpolated_map.col(3).data() + map_out_mpc_size);

  std::vector<double> v_smooth_vect;

  // Prepare a linear interpolator.
  ns_splines::InterpolatingSplinePCG interpolator_linear(1);

  // Call signature (monotonic s or t, x(s, or t), new s or t series, interpolated x_hat)
  interpolator_linear.Interpolate(mpc_traj_raw.s, mpc_traj_raw.vx, s_smooth_vect, v_smooth_vect);

  // Smooth yaw.
  std::vector<double> yaw_smooth_vect(map_out_mpc_size);  // We do not use yaw as a reference.

  for (Eigen::Index k = 0; k < rdot_interp.rows(); ++k)
  {
    yaw_smooth_vect.at(k) = std::atan2(rdot_interp(k, 1), rdot_interp(k, 0));
  }

  // ns_utils::computeYawFromXY(x_smooth_vect, y_smooth_vect, yaw_smooth_vect);
  // ns_utils::interp1d_map_linear(
  // mpc_traj_raw.s, mpc_traj_raw.yaw, s_smooth_vect, yaw_smooth_vect, true);

  std::vector<double> curvature_smooth_vect(curvature.col(0).data(), curvature.col(0).data() + map_out_mpc_size);

  // Compute time and acceleration reference.
  // These are not smoothed, but they belong to the smooth MPCtraj.
  std::vector<double> t_smooth_vect(map_out_mpc_size);
  std::vector<double> acc_smooth_vect(map_out_mpc_size);

  // Insert the first elements.
  t_smooth_vect.at(0) = 0.0;

  // copy x,y, psi, vx of raw trajectory into their corresponding vectors.
  for (size_t k = 1; k < s_smooth_vect.size(); ++k)
  {
    double const &&ds = s_smooth_vect.at(k) - s_smooth_vect.at(k - 1);

    // used for trapezoidal integration rule.
    double const &&mean_v = (v_smooth_vect.at(k) + v_smooth_vect.at(k - 1)) / 2;
    double const &dv = v_smooth_vect.at(k) - v_smooth_vect.at(k - 1);  // dv = v[k]-v[k-1],

    double &&dt = ds / std::max(mean_v, 0.1);  // to prevent zero division.

    // this acceleration is implied by x,y,z and vx in the planner.
    double const &&acc_computed = dv / (EPS + dt);

    // Insert  t and acc,
    t_smooth_vect.at(k) = t_smooth_vect.at(k - 1) + dt;
    acc_smooth_vect.at(k) = acc_computed;
  }

  // Repeat the last acceleration.
  acc_smooth_vect.rend()[-1] = acc_smooth_vect.rend()[-2];  // emplace_back(acc_smooth_vect.back());

  // Convert smooth yaw to a monotonic series
  ns_utils::convertEulerAngleToMonotonic(&yaw_smooth_vect);

  // Create the rest of the smooth vectors; handled in MPCtraj class.
  mpc_traj_smoothed.setTrajectoryCoordinate('s', s_smooth_vect);
  mpc_traj_smoothed.setTrajectoryCoordinate('t', t_smooth_vect);
  mpc_traj_smoothed.setTrajectoryCoordinate('a', acc_smooth_vect);
  mpc_traj_smoothed.setTrajectoryCoordinate('x', x_smooth_vect);
  mpc_traj_smoothed.setTrajectoryCoordinate('y', y_smooth_vect);
  mpc_traj_smoothed.setTrajectoryCoordinate('z', z_smooth_vect);
  mpc_traj_smoothed.setTrajectoryCoordinate('v', v_smooth_vect);
  mpc_traj_smoothed.setTrajectoryCoordinate('w', yaw_smooth_vect);
  mpc_traj_smoothed.setTrajectoryCoordinate('c', curvature_smooth_vect);

  // Compute relative time and acceleration from the given data.
  if (current_fsm_state_ == ns_states::motionStateEnums::willStop)
  {
    mpc_traj_smoothed.addExtraEndPoints(average_mpc_solve_time_);
  }

  nonlinear_mpc_controller_ptr_->setMPCtrajectorySmoothVectorsPtr(mpc_traj_smoothed);

  // Verify size
  if (auto const &&size_of_mpc_smooth = mpc_traj_smoothed.size(); size_of_mpc_smooth == 0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "[mpc_nonlinear - resampleRawTrajectoryToAFixedSize ]  smooth "
      "MPCdataTrajectoryVectors  are not assigned properly ... ");
    return false;
  }

  /**
   *  @brief update curvature spline interpolator data. This interpolator is used all across the
   * node modules.
   * */

  if (auto const &&is_updated =
      interpolator_curvature_pws.Initialize(mpc_traj_smoothed.s, mpc_traj_smoothed.curvature);
    !is_updated)
  {
    RCLCPP_ERROR(get_logger(),
                 "[mpc_nonlinear - resampling] Could not update the point-wise curvature "
                 "interpolator_spline_pws  data ...");
    return false;
  }

  // DEBUG
  //
  // ns_utils::print("Interpolated map");
  // ns_eigen_utils::printEigenMat(interpolated_map);
  //
  // ns_utils::print("rdot");
  // ns_eigen_utils::printEigenMat(rdot_interp.topRows(10));
  // ns_eigen_utils::printEigenMat(rdot_interp.bottomRows(10));
  //
  // ns_utils::print("rddot");
  // ns_eigen_utils::printEigenMat(rdot_interp.topRows(10));
  // ns_eigen_utils::printEigenMat(rdot_interp.bottomRows(10));

  // ns_utils::print("MPC raw vs smooth data members, s, t, x, y, z, vx, acc");

  //  ns_utils::print("\n Raw s -------------- \n");
  //  ns_utils::print_container(mpc_traj_raw.s);

  //  ns_utils::print("\n Smooth s -------------- \n");
  //  ns_utils::print_container(mpc_traj_smoothed.s);

  //  ns_utils::print("\n Raw t -------------- \n");
  //  ns_utils::print_container(mpc_traj_raw.t);

  //  ns_utils::print("\n Smooth t -------------- \n");
  //  ns_utils::print_container(mpc_traj_raw.t);

  //  ns_utils::print("\n Raw x -------------- \n");
  //  ns_utils::print_container(mpc_traj_raw.x);

  //  ns_utils::print("\n Smooth x -------------- \n");
  //  ns_utils::print_container(mpc_traj_smoothed.x);

  //  ns_utils::print("\n Raw y -------------- \n");
  //  ns_utils::print_container(mpc_traj_raw.y);

  //  ns_utils::print("\n Smooth y -------------- \n");
  //  ns_utils::print_container(mpc_traj_smoothed.y);

  //  ns_utils::print("\n Raw z -------------- \n");
  //  ns_utils::print_container(mpc_traj_raw.z);

  //  ns_utils::print("\n Smooth z -------------- \n");
  //  ns_utils::print_container(mpc_traj_smoothed.z);

  // ns_utils::print("\n Reference yaw ------------ \n");
  // ns_utils::print_container(mpc_traj_raw.yaw);

  // ns_utils::print("\n Smooth yaw ------------ \n");
  // ns_utils::print_container(mpc_traj_smoothed.yaw);

  //  ns_utils::print("\nvx -------------- ");
  //  ns_utils::print_container(mpc_traj_smoothed.vx);
  //
  // ns_utils::print("\nacc -------------- ");
  // ns_utils::print_container(mpc_traj_smoothed.acc);
  //
  //  ns_utils::print("\ncurvature -------------- ");
  //  ns_utils::print_container(mpc_traj_smoothed.curvature);
  //
  // end of DEBUG

  return true;
}

void NonlinearMPCNode::findClosestPrevWayPointIdx()
{
  /**
   *  Create Vectors of Path Directions for each interval
   *  interval_vector_xy = {waypoint_1 - waypoint_0}_xy using dot products.
   * */

  // Prepare vector of projection distance values; projection of vehicle vectors onto the intervals
  std::vector<double> projection_distances_ds;

  auto f_projection_dist = [this](auto const &point_1, auto const &point_0) -> double
  {
    auto const &pose_0 = point_0.pose;
    auto const &pose_1 = point_1.pose;

    // vector of intervals
    std::array<double, 2> int_vec{pose_1.position.x - pose_0.position.x, pose_1.position.y - pose_0.position.y};

    // Compute the magnitude of path interval vector.
    double const &ds_mag = std::hypot(int_vec[0], int_vec[1]);

    // vector to vehicle from the origin waypoints
    // std::array<double, 2> vehicle_vec{
    // this->current_pose_ptr_->pose.position.x - pose_0.position.x,
    // this->current_pose_ptr_->pose.position.y - pose_0.position.y};

    std::array<double, 2> const &&vehicle_vec{
      this->current_COG_pose_ptr_->pose.position.x - pose_0.position.x,
      this->current_COG_pose_ptr_->pose.position.y - pose_0.position.y};

    // <-@brief ds = <p1, p2> / |p2|
    double const
      &&projection_distance_onto_interval = (int_vec[0] * vehicle_vec[0] + int_vec[1] * vehicle_vec[1]) / ds_mag;

    return projection_distance_onto_interval;
  };

  // The projection distances must be decreasing along the trajectory until the distance becomes
  // negative.
  std::vector<double> positive_projection_distances;

  for (size_t k = 1; k < current_trajectory_ptr_->points.size(); ++k)
  {
    auto const point_1 = current_trajectory_ptr_->points.at(k);
    auto const point_0 = current_trajectory_ptr_->points.at(k - 1);
    auto &&ds_proj = f_projection_dist(point_1, point_0);

    if (ds_proj > 0.)
    {
      positive_projection_distances.emplace_back(ds_proj);
    } else
    {
      break;
    }
  }

  // Fill the projection_distances vector.
  std::transform(current_trajectory_ptr_->points.cbegin() + 1, current_trajectory_ptr_->points.cend(),
                 current_trajectory_ptr_->points.cbegin(), std::back_inserter(projection_distances_ds),
                 f_projection_dist);

  // Lambda function to replace negative numbers with a large number.
  // auto fnc_check_if_negative = [](auto const &x) -> double
  // {
  // 	return x < 0 ? std::numeric_limits<double>::max() : x;
  // };

  /**
   * We compute the projection of vehicle vector to all the waypoints on their intervals, and get
   * the smallest positive distance which gives the waypoint that the vehicle has just left behind.
   * */

  //    std::vector<double> projections_distances_all_positive;
  //    std::transform(projection_distances_ds.cbegin(), projection_distances_ds.cend(),
  //                   std::back_inserter(projections_distances_all_positive),
  //                   fnc_check_if_negative);

  // Minimum of all positive distances and the index of the next waypoint.
  auto const it =
    std::min_element(positive_projection_distances.cbegin(), positive_projection_distances.cend());

  // Extract location of iterator idx and store in the class.
  size_t const &&temp_idx_prev_wp_ =
    static_cast<size_t>(std::distance(positive_projection_distances.cbegin(), it));

  idx_prev_wp_ptr_ = std::make_unique<size_t>(temp_idx_prev_wp_);

  // Distance of next waypoint to the vehicle, for anomaly detection.
  // double min_distance_ds = projections_distances_all_positive[*idx_prev_wp_ptr_];

  // Check if the computed closest point index is valid.
  if (*idx_prev_wp_ptr_ > current_trajectory_size_)
  {
    RCLCPP_ERROR(
      get_logger(),
      "[mpc_nonlinear] The computed closest point indices are out of trajectory size ...");
  }

  // Set the next waypoint index.
  size_t idx_next_wp_temp{};
  if (*idx_prev_wp_ptr_ < current_trajectory_size_ - 1)
  {
    // we are not at the last point.
    idx_next_wp_temp = *idx_prev_wp_ptr_ + 1;

  } else
  {
    // The current prev. waypoint id is the last point. Set the next waypoint as the last point.
    idx_next_wp_temp = current_trajectory_size_ - 1;
  }

  // Keep in the class object.
  idx_next_wp_ptr_ = std::make_unique<size_t>(idx_next_wp_temp);

  // DEBUG
  // ns_utils::print("\nPrevious and next index points idx : ", *idx_prev_wp_ptr_,
  // *idx_next_wp_ptr_); ns_utils::print("Current trajectory size : ",
  // current_trajectory_ptr_->points.size());
  //    ns_utils::print("Projection distances : \n");
  //    ns_utils::print_container(projection_distances_ds);

  //    ns_utils::print("Positive distances : \n");
  //    ns_utils::print_container(projections_distances_all_positive);

  //    auto x0 = current_trajectory_ptr_->points.at(0).pose.position.x;
  //    auto y0 = current_trajectory_ptr_->points.at(0).pose.position.y;
  //
  //    ns_utils::print("Current relative vehicle position x, y ",
  //    current_COG_pose_ptr_->pose.position.x - x0,
  //                    current_COG_pose_ptr_->pose.position.y - y0, "\n");

  //    for (auto const &point: current_trajectory_ptr_->points)
  //    {
  //        ns_utils::print("Current trajectory points x, y ", point.pose.position.x - x0,
  //        point.pose.position.y - y0);
  //    }

  // end of DEBUG
}

void NonlinearMPCNode::computeClosestPointOnTraj()
{
  // Create performance variables msg to track the performance of NMPC.
  nmpc_performance_vars_ = NonlinearMPCPerformanceMsg();

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

  // First get yaw angles of all three poses
  double const &&prev_yaw = tf2::getYaw(current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.orientation);

  double const &&next_yaw = tf2::getYaw(current_trajectory_ptr_->points.at(*idx_next_wp_ptr_).pose.orientation);

  // Previous waypoint to next waypoint
  double const &&dx_prev_to_next = current_trajectory_ptr_->points.at(*idx_next_wp_ptr_).pose.position.x -
                                   current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.position.x;

  double const &&dy_prev_to_next = current_trajectory_ptr_->points.at(*idx_next_wp_ptr_).pose.position.y -
                                   current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.position.y;

  double const &&dz_prev_to_next = current_trajectory_ptr_->points.at(*idx_next_wp_ptr_).pose.position.z -
                                   current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.position.z;

  double const &&dyaw_prev_to_next = ns_utils::angleDistance(next_yaw, prev_yaw);

  // Create a vector from p0 (prev) --> p1 (to next wp).
  std::vector<double> v_prev_to_next_wp{dx_prev_to_next, dy_prev_to_next};

  // Previous waypoint to the vehicle pose
  /**
   *   p0:previous waypoint ----> p1 next waypoint
   *   vector = p1 - p0. We project vehicle vector on this interval to
   * */

  double const &&dx_prev_to_vehicle = current_COG_pose_ptr_->pose.position.x -
                                      current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.position.x;

  double const &&dy_prev_to_vehicle = current_COG_pose_ptr_->pose.position.y -
                                      current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.position.y;

  // Vector from p0 to p_vehicle.
  std::vector<double> v_prev_to_vehicle{dx_prev_to_vehicle, dy_prev_to_vehicle};

  // Compute the length of v_prev_to_next_wp : vector from p0 --> p1.
  double const &&magnitude_p0_to_p1 = std::hypot(dx_prev_to_next, dy_prev_to_next);

  /**
   * Compute how far the car is away from p0 in direction. p_interp is the location of the
   * interpolated waypoint. This is the dot product normalized by the length of the interval.
   * a.b = |a|.|b|.cos(alpha) -- > |a|.cos(alpha) = a.b / |b| where b is the path interval.
   * */
  double const &&ds_distance_p0_to_p_interp =
    (dx_prev_to_next * dx_prev_to_vehicle + dy_prev_to_next * dy_prev_to_vehicle) / magnitude_p0_to_p1;

  // Get the current distance from the origin and keep in te current_s0_ variable.
  nonlinear_mpc_controller_ptr_->getRawDistanceAtIdx(*idx_prev_wp_ptr_, current_s0_);
  current_s0_ += ds_distance_p0_to_p_interp;

  nonlinear_mpc_controller_ptr_->setCurrent_s0(current_s0_);

  /**
   *  We use the following linear interpolation;
   *  pi = p0 + ratio_t * (p1 - p0)
   * */

  // clamp signature (val, lower, upper)
  double const &&ratio_t = ns_utils::clamp(ds_distance_p0_to_p_interp / magnitude_p0_to_p1, 0., 1.);

  // InterpolateInCoordinates pose.position and pose.orientation
  interpolated_traj_point.pose.position.x = current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.position.x +
                                            ratio_t * dx_prev_to_next;

  interpolated_traj_point.pose.position.y = current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.position.y +
                                            ratio_t * dy_prev_to_next;

  interpolated_traj_point.pose.position.z = current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).pose.position.z +
                                            ratio_t * dz_prev_to_next;

  // InterpolateInCoordinates the yaw angle of pi : interpolated waypoint
  double const &&interp_yaw_angle = ns_utils::angleDistance(prev_yaw + ratio_t * dyaw_prev_to_next);

  geometry_msgs::msg::Quaternion orient_msg = ns_utils::createOrientationMsgfromYaw(interp_yaw_angle);
  interpolated_traj_point.pose.orientation = orient_msg;

  // InterpolateInCoordinates the Vx longitudinal speed.
  double const &vx_prev = current_trajectory_ptr_->points.at(*idx_prev_wp_ptr_).longitudinal_velocity_mps;
  double const &vx_next = current_trajectory_ptr_->points.at(*idx_next_wp_ptr_).longitudinal_velocity_mps;
  double const &&vx_target = vx_prev + ratio_t * (vx_next - vx_prev);
  interpolated_traj_point.longitudinal_velocity_mps = vx_target;

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

  // DEBUG
  // ns_utils::print("\nCurrent s0, current t0 : ", current_s0_, current_t0_);
  // ns_utils::print("\nPrevious, current and next yaw_angles : ", prev_yaw, interp_yaw_angle,
  // next_yaw);

  // ns_utils::print("\nPrev, target, next trajectory speeds : ", vx_prev, vx_target, vx_next);
  // ns_utils::print("ds, p0p1_mag, ratio_ : ", ds_distance_p0_to_p_interp, magnitude_p0_to_p1,
  // ratio_t); ns_utils::print(" Current vehicle speed, target speed : ",
  // current_velocity_ptr_->twist.twist.linear.x, vx_target);

  // end of debug.
}

std::array<double, 2> NonlinearMPCNode::computeErrorStates()
{
  /**
   * @brief current closest point on the trajectory
   * */
  // std::array<double, 2> interpolated_smooth_traj_point_xy{
  // current_interpolated_traj_point_ptr_->pose.position.x,
  // current_interpolated_traj_point_ptr_->pose.position.y};

  /** @brief current pose of the vehicle */
  // std::array<double, 2> current_pose_xy{
  // current_pose_ptr_->pose.position.x, current_pose_ptr_->pose.position.y};

  std::array<double, 2> const current_pose_xy{current_COG_pose_ptr_->pose.position.x,
                                              current_COG_pose_ptr_->pose.position.y};

  // Get Yaw angles of the reference waypoint and the vehicle
  /**
   * The stored yaw angles is unwrapped before storing them which removes [-pi, pi] interval constraints. This is
   * necessary to be able to interpolate the angles given a trajectory.
   * */

  // std::array<double, 3> const target_xy_yaw = nonlinear_mpc_controller_ptr_->getSmooth_XYYawAtCurrentDistance();
  // auto reference_yaw_angle = ns_utils::angleDistance(target_xy_yaw[2]);

  /**
   * Since the reference trajectory yaw angles are unwrapped during the construction, we need to wrap them back
   * into [-pi, pi] interval as the vehicle yaw angle is in this range. The difference of angles will be taken
   * care of by the angle distance methods.
   */

  // reference_yaw_angle = ns_utils::angleDistance(reference_yaw_angle); // overloaded angle distance also wraps.
  // std::array<double, 2> interpolated_smooth_traj_point_xy{target_xy_yaw[0], target_xy_yaw[1]}; // [x, y]

  // We can also use the interpolated pose which yields cleaner error states.
  std::array<double, 2> interpolated_smooth_traj_point_xy{current_interpolated_traj_point_ptr_->pose.position.x,
                                                          current_interpolated_traj_point_ptr_->pose.position.y};

  auto reference_yaw_angle = tf2::getYaw(current_interpolated_traj_point_ptr_->pose.orientation);
  reference_yaw_angle = ns_utils::angleDistance(reference_yaw_angle);  // overloaded angle distance also wraps.

  double const &vehicle_yaw_angle = tf2::getYaw(current_COG_pose_ptr_->pose.orientation);

  // computeLateralError(trajectory_ref_xy, vehicle_xy).
  auto const &error_ey = ns_utils::computeLateralError(interpolated_smooth_traj_point_xy,
                                                       current_pose_xy, /*current_pose_xy for rear axle*/
                                                       vehicle_yaw_angle);

  // Compute yaw angle error.
  // double const &heading_yaw_error = 1.0 * ns_utils::wrapToPi(vehicle_yaw_angle - reference_yaw_angle);
  // auto const &heading_yaw_error =
  // autoware::common::helper_functions::wrap_angle(vehicle_yaw_angle - reference_yaw_angle);

  double heading_yaw_error = 1.0 * ns_utils::angleDistance(vehicle_yaw_angle, reference_yaw_angle);
  // heading_yaw_error = autoware::common::helper_functions::wrap_angle(heading_yaw_error);

  // Set nmpc_performance yaw angles.
  nmpc_performance_vars_.yaw_angle_measured = vehicle_yaw_angle;
  nmpc_performance_vars_.yaw_angle_target = reference_yaw_angle;
  nmpc_performance_vars_.yaw_angle_traj = tf2::getYaw(current_interpolated_traj_point_ptr_->pose.orientation);

  std::array<double, 2> const error_states{error_ey, 1.0 * heading_yaw_error};

//  /** set the computed error */
//  current_error_report_.lateral_deviation_read = error_ey;
//  current_error_report_.heading_angle_error_read = heading_yaw_error;
//  current_error_report_.steering_read = current_steering_ptr_->steering_tire_angle;
//  current_error_report_.curvature_read = current_curvature_k0_;

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
  x0_initial_states_(toUType(VehicleStateIds::yaw)) = tf2::getYaw(current_COG_pose_ptr_->pose.orientation);
  x0_initial_states_(toUType(VehicleStateIds::s)) = current_s0_;

  // Error model states.
  x0_initial_states_(toUType(VehicleStateIds::ey)) = error_states[0];  // e_y  : lateral tracking error.
  x0_initial_states_(toUType(VehicleStateIds::eyaw)) = error_states[1];  // e_psi : heading error

  auto const &vx_meas = current_velocity_ptr_->twist.twist.linear.x;
  x0_initial_states_(toUType(VehicleStateIds::vx)) = vx_meas;  // vx longitudinal speed state
  x0_initial_states_(toUType(VehicleStateIds::steering)) =
    static_cast<double>(current_steering_ptr_->steering_tire_angle);

  x0_initial_states_(toUType(VehicleStateIds::vy)) = current_velocity_ptr_->twist.twist.linear.y;

  //	// ns_utils::print("before using comm delay error refs...");
  //	if (params_node_.use_cdob && current_comm_delay_ptr_)
  //	{
  //		x0_initial_states_(4) =
  // current_comm_delay_ptr_->lateral_deviation_error_compensation_ref; 		x0_initial_states_(5)
  // =
  // current_comm_delay_ptr_->heading_angle_error_compensation_ref; 		x0_initial_states_(7)
  // = current_comm_delay_ptr_->steering_compensation_ref;
  //
  //		// ns_utils::print("NMPC using error references...");
  //	}

  // compute the current curvature and store it.
  if (auto const &&could_interpolate = interpolator_curvature_pws.Interpolate(current_s0_, current_curvature_k0_);
    !could_interpolate)
  {
    RCLCPP_ERROR(get_logger(),
                 "[mpc_nonlinear] Could not interpolate the curvature in the initial state update "
                 "method ...");
    return;
  }
  // x(8) is the last state placeholder is reserved for lateral acceleration or acc control.
  // x0_initial_states_(8) = current_curvature_k0_ * vx_meas * vx_meas;  // Vx**2 / R

  // Create the dynamic parameters
  Model::param_vector_t params(Model::param_vector_t::Zero());
  current_model_params_.setZero();
  current_model_params_(toUType(VehicleParamIds::curvature)) = current_curvature_k0_;

  double vx_target_0{};
  nonlinear_mpc_controller_ptr_->getSmoothVxAtDistance(current_s0_, vx_target_0);
  current_model_params_(toUType(VehicleParamIds::target_vx)) = vx_target_0;  // for modeling a virtual car tracking.

  // For Kalman filtering implementation.
  if (params_node_.use_kalman)
  {
    if (!kalman_filter_ptr_->isInitialized())
    {
      kalman_filter_ptr_->Initialize_xest0(x0_initial_states_);
    }

    // Prepare Kalman estimate state.
    x0_kalman_est_.setZero();

    // Add applied previous disturbance to the previous computed control.
    kalman_filter_ptr_->getStateEstimate(u0_kalman_,
                                         current_model_params_,
                                         x0_previous_initial_states_,
                                         x0_kalman_est_);

    // Set initial states of the NMPCcore trajectory container.
    //-- ['xw', 'yw', 'psi', 's', 'e_y', 'e_yaw', 'Vx', 'delta', 'ay']

    // Prevent negative speed Kalman estimate
    x0_kalman_est_(ns_utils::toUType(VehicleStateIds::vx)) =
      std::max(x0_kalman_est_(toUType(VehicleStateIds::vx)), 0.0);

    nonlinear_mpc_controller_ptr_->updateInitialStates_x0(x0_kalman_est_);
  }

    // if Kalman filter is not used
  else
  {
    nonlinear_mpc_controller_ptr_->updateInitialStates_x0(x0_initial_states_);
  }

  // Set the nonlinear mpc performance variables.
  // Set the performance variable msg.
  nmpc_performance_vars_.nmpc_lateral_error = error_states[0];
  nmpc_performance_vars_.nmpc_yaw_error = error_states[1];
  nmpc_performance_vars_.yaw_angle_ukf = x0_kalman_est_[2];

  nmpc_performance_vars_.long_velocity_measured = current_velocity_ptr_->twist.twist.linear.x;

  nmpc_performance_vars_.lateral_error_ukf = x0_kalman_est_(toUType(VehicleStateIds::ey));
  nmpc_performance_vars_.yaw_error_ukf = x0_kalman_est_(toUType(VehicleStateIds::eyaw));
  nmpc_performance_vars_.long_velocity_ukf = x0_kalman_est_(toUType(VehicleStateIds::vx));
  nmpc_performance_vars_.steering_angle_ukf = x0_kalman_est_(toUType(VehicleStateIds::steering));
  nmpc_performance_vars_.nmpc_curvature = current_curvature_k0_;

  // vy, or ay
  nmpc_performance_vars_.lateral_velocity = x0_kalman_est_(toUType(VehicleStateIds::vy));

  // DEBUG
  // ns_utils::print("Initial states : ");
  // ns_eigen_utils::printEigenMat(x0_initial_states_);
  // ns_utils::print("Kalman estimate: ");
  // ns_eigen_utils::printEigenMat(x0_kalman_est_);
  // end of DEBUG
}

void NonlinearMPCNode::predictDelayedInitialStateBy_MPCPredicted_Inputs(Model::state_vector_t &x0_predicted)
{
  using ns_utils::toUType;
  Model::input_vector_t uk{Model::input_vector_t::Zero()};
  Model::param_vector_t params(Model::param_vector_t::Zero());

  // Input buffer content : // [acc, vx, steering_rate, steering]
  for (auto const &all_controls : inputs_buffer_common_)
  {
    // Extract the controls from the input buffer.
    uk(toUType(VehicleControlIds::u_vx)) = all_controls[0];  // ax input
    uk(toUType(VehicleControlIds::u_steering)) = all_controls[3];  // steering input

    auto const &sd0 = x0_predicted(ns_utils::toUType(VehicleStateIds::s));  // d stands for  delayed states.
    double kappad0{};           // curvature placeholder
    double vxk{};               // to interpolate the target speed (virtual car speed).

    // set the interpolator re-using_coefficients=true. The spline data is updated in the onTime().
    if (auto const &&could_interpolate = interpolator_curvature_pws.Interpolate(sd0, kappad0);
      !could_interpolate)
    {
      RCLCPP_ERROR(get_logger(),
                   "[nonlinear_mpc - predict initial state]: spline interpolator failed to compute  the  "
                   "coefficients ...");
      return;
    }

    // Get target state estimate at the next sd0 distance.
    // double vxk;  // to interpolate the target speed (virtual car speed).
    nonlinear_mpc_controller_ptr_->getSmoothVxAtDistance(sd0, vxk);

    params(toUType(VehicleParamIds::curvature)) = kappad0;
    params(toUType(VehicleParamIds::target_vx)) = vxk;

    // Use kappa estimated to call simulate method of mpc. The delay time-step is
    // computed with the sampling time step: params_node_.control_period

    //    ns_utils::print("Before simulating predictive state, the state vector is:");
    //    ns_eigen_utils::printEigenMat(x0_predicted);
    //
    //    ns_utils::print("Controls sent to the simulator:");
    //    ns_eigen_utils::printEigenMat(uk);

    nonlinear_mpc_controller_ptr_->simulateOneStep(uk, params, params_node_.control_period, x0_predicted);

    //    ns_utils::print("After simulateOneStep predictive state, the state vector is:");
    //    ns_eigen_utils::printEigenMat(x0_predicted);

    // Saturate steering.
    //-- ['xw', 'yw', 'psi', 's', 'e_y', 'e_yaw', 'Vx', 'delta', 'ay']
    nonlinear_mpc_controller_ptr_->applyControlConstraints(uk);
    nonlinear_mpc_controller_ptr_->applyStateConstraints(x0_predicted);
  }

  // Set the current_predicted_s0_.  states = [x, y, psi, s, ey, epsi, v, delta]
  current_predicted_s0_ = x0_predicted(3);
  nonlinear_mpc_controller_ptr_->setCurrent_s0_predicted(current_predicted_s0_);

  // Predict the curvature and Ackermann steering angle.
  interpolator_curvature_pws.Interpolate(current_predicted_s0_, current_predicted_curvature_p0_);

  current_feedforward_steering_ = std::atan(current_predicted_curvature_p0_ * wheel_base_);

  // Set the predicted model params.
  predicted_model_params_ = params;  // [curvature, predicted longitudinal speed vxk]

  // Set the next predicted virtual car distance for integral control action.
  //    if (current_fsm_state_ == ns_states::motionStateEnums::isAtCompleteStop) {
  //        kalman_filter_.reset();
  //    }
  //    else
  //    {
  //        // Estimate the disturbance input for the physical system.
  //    }

  // DEBUG
  //  ns_utils::print("virtual car distance : ");
  //  ns_utils::print_container(virtual_car_distance_state_vect);
  // end of debug
}

void NonlinearMPCNode::predictDelayedInitialStateBy_TrajPlanner_Speeds(Model::state_vector_t &xd0)
{
  // Prepare the base vectors for the speed piecewise interpolator.
  std::vector<std::vector<double>> tbase_vx_base;
  nonlinear_mpc_controller_ptr_->getTimeSpeedVectsFromSmoothTraj(tbase_vx_base);

  // Get the current time.
  double td0 = current_t0_;

  Model::input_vector_t uk(Model::input_vector_t::Zero());
  Model::param_vector_t params(Model::param_vector_t::Zero());

  // Predict the delayed initial state given the input sequence.
  // Input buffer content : // [acc, vx, steering_rate, steering]
  for (auto const &all_controls : inputs_buffer_common_)
  {
    // Placeholders for the speed interpolation.
    double v0{};
    double v1{};

    // Extract the controls from the input buffer.
    uk(0) = all_controls[0];  // ax input
    uk(1) = all_controls[3];  // steering input

    auto sd0 = xd0(3);  // d is for  delayed states.
    double kappad0{};

    // set the interpolator re-using_coefficients=true. The spline data is updated in the onTime().
    auto could_interpolate = interpolator_curvature_pws.Interpolate(sd0, kappad0);

    if (!could_interpolate)
    {
      RCLCPP_ERROR(get_logger(),
                   "[nonlinear_mpc - predict initial state]: spline interpolator optimization. to compute"
                   " the  coefficients ...");
      return;
    }

    // InterpolateInCoordinates the target v0, v1 on the trajectory.
    ns_utils::interp1d_linear(
      tbase_vx_base[0], /*time vector*/
      tbase_vx_base[1], /*vx speed vector*/
      td0,              /*current simulation time*/
      v0);

    ns_utils::interp1d_linear(tbase_vx_base[0], tbase_vx_base[1],
                              td0 + params_node_.control_period, /*next simulation time*/
                              v1);

    params(0) = kappad0;
    params(1) = v0;

    // Use kappa estimated to call simulate method of mpc. The delay time-step
    // is computed with the sampling time step: params_ptr_->control_period
    nonlinear_mpc_controller_ptr_->simulateOneStepVariableSpeed(uk, params, v0, v1, params_node_.control_period, xd0);

    // Saturate steering.
    // xd0(7) = std::max(std::min(params_ptr_->xupper(7), xd0(7)), params_ptr_->xlower(7));
    nonlinear_mpc_controller_ptr_->applyStateConstraints(7, xd0);

    // Update the simulation time.
    td0 += params_node_.control_period;  // update the current simulation time.
  }

  // Set the current predicted trajectory.
  // Set the current predicted trajectory.
  current_predicted_s0_ = xd0(3);
  nonlinear_mpc_controller_ptr_->setCurrent_s0_predicted(current_predicted_s0_);

  // DEBUG
  // end of debug
}

void NonlinearMPCNode::getAverageMPCcomputeTime(double &avg_compute_time_in_sec) const
{
  avg_compute_time_in_sec = average_mpc_solve_time_;
}

visualization_msgs::msg::MarkerArray NonlinearMPCNode::createPredictedTrajectoryMarkers(std::string const &ns,
                                                                                        std::string const &frame_id,
                                                                                        std::array<double, 2> xy0,
                                                                                        Model::trajectory_data_t const &td) const
{
  visualization_msgs::msg::MarkerArray marker_array;
  size_t const &&nX = td.nX();

  if (td.X.empty())
  {
    return marker_array;
  }

  auto marker = createDefaultMarker("map", current_trajectory_ptr_->header.stamp, ns, 0,
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

  auto color = createMarkerColor(0.0, static_cast<float>(0.8), static_cast<float>(0.8), static_cast<float>(1.8));
  marker_poses.color = color;

  for (unsigned int k = 0; k < nX; ++k)
  {
    // insert poses.
    marker_poses.id = static_cast<int>(k);
    marker_poses.pose.position.x = td.X.at(k)(0) + xy0[0];  // add back current position x.;
    marker_poses.pose.position.y = td.X.at(k)(1) + xy0[1];

    marker_poses.pose.position.z = 0.0;

    auto const &&yaw_angle = ns_utils::wrapToPi(td.X.at(k)(2));
    marker_poses.pose.orientation = ns_utils::getQuaternionFromYaw(yaw_angle);
    marker_array.markers.emplace_back(marker_poses);
  }

  return marker_array;
}

void NonlinearMPCNode::publishPredictedTrajectories(std::string const &ns, std::string const &header_id) const
{
  auto const &td = nonlinear_mpc_controller_ptr_->getCurrentTrajectoryData();

  // Get the starting point: vehicle position projection on the trajectory.
  // In order not to deal with big numbers, prepare x, y w.r.t x0, y0.
  //  double xw0 = current_trajectory_ptr_->points.at(0).pose.position.x;
  //  double yw0 = current_trajectory_ptr_->points.at(0).pose.position.y;
  std::array<double, 2> const xy0{current_COG_pose_ptr_->pose.position.x, current_COG_pose_ptr_->pose.position.y};

  // Create visualization array for the predicted trajectory.
  if (auto visualization_prediction_markers = createPredictedTrajectoryMarkers(ns, header_id, xy0, td);
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
  visualization_msgs::msg::Marker
    marker = createLocationMarker(debug_data_.current_closest_pose, current_trajectory_ptr_->header.stamp, ns);
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

ControlCmdMsg NonlinearMPCNode::createControlCommand(double const &ax,
                                                     double const &vx,
                                                     double const &steering_rate,
                                                     double const &steering_val)
{
  ControlCmdMsg ctrl_cmd;

  ctrl_cmd.lateral.steering_tire_rotation_rate = static_cast<float>(steering_rate);
  ctrl_cmd.lateral.steering_tire_angle = static_cast<float>(steering_val);

  ctrl_cmd.longitudinal.acceleration = static_cast<float>(ax);
  ctrl_cmd.longitudinal.speed = static_cast<float>(vx);

  return ctrl_cmd;
}

void NonlinearMPCNode::estimateDisturbanceInput()
{
  // First compute the disturbance input before updating the initial states and parameters.
  // Get current input disturbance estimate.

  /**
   * @brief disturbance estimate.
   */

  // Get the rate of change of current speed dynamics
  // v_dot = f(current_speed, current_speed_input).
  auto const &current_speed = x0_predicted_(6);
  auto const &current_speed_input = u_previous_solution_(0);  // [vin, steering_input]
  double vdot{};

  // Get the rate of change of longitudinal speed.
  nonlinear_mpc_controller_ptr_->getSpeedDynamics_vdot(current_speed, current_speed_input, vdot);
}

/**
 * Gets the values of distance to stop, current ego speed and target velocity at the next point.
 * */
std::array<double, 3> NonlinearMPCNode::getDistanceEgoTargetSpeeds() const
{
  auto const &&distance_to_stopping_point = calcStopDistance(*idx_prev_wp_ptr_);
  auto const &current_vel = current_velocity_ptr_->twist.twist.linear.x;
  auto const &target_vel = current_trajectory_ptr_->points.at(*idx_next_wp_ptr_).longitudinal_velocity_mps;

  // state machine toggle(argument -> [distance_to_stop, vx_current, vx_next])
  return std::array<double, 3>{distance_to_stopping_point, current_vel, target_vel};
}

void NonlinearMPCNode::setErrorReport(Model::state_vector_t const &x)
{
  /** set the computed error */
  current_error_report_.lateral_deviation_read = x(ns_utils::toUType(VehicleStateIds::ey));
  current_error_report_.heading_angle_error_read = x(ns_utils::toUType(VehicleStateIds::eyaw));
  current_error_report_.steering_read = x(ns_utils::toUType(VehicleStateIds::steering));
  current_error_report_.curvature_read = current_predicted_curvature_p0_;
}

void NonlinearMPCNode::publishErrorReport(ErrorReportMsg &error_rpt_msg)
{
  error_rpt_msg.stamp = this->now();
  pub_nmpc_error_report_->publish(error_rpt_msg);
}

rcl_interfaces::msg::SetParametersResult NonlinearMPCNode::onParameterUpdate(const std::vector<rclcpp::Parameter> &parameters)
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
