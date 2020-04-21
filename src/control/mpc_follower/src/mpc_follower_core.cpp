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

#include "mpc_follower/mpc_follower_core.h"

#define DEG2RAD 3.1415926535 / 180.0
#define RAD2DEG 180.0 / 3.1415926535

MPCFollower::MPCFollower() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  pnh_.param<bool>("show_debug_info", show_debug_info_, false);
  pnh_.param<double>("ctrl_period", ctrl_period_, 0.03);
  pnh_.param<bool>("enable_path_smoothing", enable_path_smoothing_, true);
  pnh_.param<bool>("enable_yaw_recalculation", enable_yaw_recalculation_, false);
  pnh_.param<int>("path_filter_moving_ave_num", path_filter_moving_ave_num_, 35);
  pnh_.param<int>("curvature_smoothing_num", curvature_smoothing_num_, 35);
  pnh_.param<double>("traj_resample_dist", traj_resample_dist_, 0.1);  // [m]
  pnh_.param<double>("admisible_position_error", admisible_position_error_, 5.0);
  pnh_.param<double>("admisible_yaw_error", admisible_yaw_error_, M_PI_2);

  /* mpc parameters */
  double steer_lim_deg, steer_rate_lim_degs;
  pnh_.param<double>("steer_lim_deg", steer_lim_deg, 35.0);
  pnh_.param<double>("steer_rate_lim_degs", steer_rate_lim_degs, 150.0);
  pnh_.param<double>("/vehicle_info/wheel_base", wheelbase_, 2.9);
  steer_lim_ = steer_lim_deg * DEG2RAD;
  steer_rate_lim_ = steer_rate_lim_degs * DEG2RAD;

  /* vehicle model setup */
  pnh_.param("vehicle_model_type", vehicle_model_type_, std::string("kinematics"));
  if (vehicle_model_type_ == "kinematics") {
    double steer_tau;
    pnh_.param<double>("vehicle_model_steer_tau", steer_tau, 0.1);

    vehicle_model_ptr_ =
      std::make_shared<KinematicsBicycleModel>(wheelbase_, steer_lim_, steer_tau);
    ROS_INFO("[MPC] set vehicle_model = kinematics");
  } else if (vehicle_model_type_ == "kinematics_no_delay") {
    vehicle_model_ptr_ = std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase_, steer_lim_);
    ROS_INFO("[MPC] set vehicle_model = kinematics_no_delay");
  } else if (vehicle_model_type_ == "dynamics") {
    double mass_fl, mass_fr, mass_rl, mass_rr, cf, cr;
    pnh_.param<double>("mass_fl", mass_fl, 600);
    pnh_.param<double>("mass_fr", mass_fr, 600);
    pnh_.param<double>("mass_rl", mass_rl, 600);
    pnh_.param<double>("mass_rr", mass_rr, 600);
    pnh_.param<double>("cf", cf, 155494.663);
    pnh_.param<double>("cr", cr, 155494.663);

    vehicle_model_ptr_ = std::make_shared<DynamicsBicycleModel>(
      wheelbase_, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
    ROS_INFO("[MPC] set vehicle_model = dynamics");
  } else {
    ROS_ERROR("[MPC] vehicle_model_type is undefined");
  }

  /* QP solver setup */
  std::string qp_solver_type;
  pnh_.param("qp_solver_type", qp_solver_type, std::string("unconstraint_fast"));
  if (qp_solver_type == "unconstraint_fast") {
    qpsolver_ptr_ = std::make_shared<QPSolverEigenLeastSquareLLT>();
    ROS_INFO("[MPC] set qp solver = unconstraint_fast");
  } else if (qp_solver_type == "qpoases_hotstart") {
    // int max_iter;
    // pnh_.param("qpoases_max_iter", max_iter, int(500));
    // qpsolver_ptr_ = std::make_shared<QPSolverQpoasesHotstart>(max_iter);
    // ROS_INFO("[MPC] set qp solver = qpoases_hotstart");
  } else if (qp_solver_type == "osqp") {
    qpsolver_ptr_ = std::make_shared<QPSolverOSQP>();
  } else {
    ROS_ERROR("[MPC] qp_solver_type is undefined");
  }

  steer_cmd_prev_ = 0.0;
  lateral_error_prev_ = 0.0;
  yaw_error_prev_ = 0.0;
  raw_steer_cmd_prev_ = 0.0;
  raw_steer_cmd_pprev_ = 0.0;

  /* delay compensation */
  double delay_tmp;
  pnh_.param<double>("input_delay", delay_tmp, 0.0);
  const int delay_step = std::round(delay_tmp / ctrl_period_);
  mpc_param_.input_delay = delay_step * ctrl_period_;
  input_buffer_ = std::deque<double>(delay_step, 0.0);

  /* initialize lowpass filter */
  double steering_lpf_cutoff_hz, error_deriv_lpf_curoff_hz;
  pnh_.param<double>("steering_lpf_cutoff_hz", steering_lpf_cutoff_hz, 3.0);
  pnh_.param<double>("error_deriv_lpf_curoff_hz", error_deriv_lpf_curoff_hz, 5.0);
  lpf_steering_cmd_.initialize(ctrl_period_, steering_lpf_cutoff_hz);
  lpf_lateral_error_.initialize(ctrl_period_, error_deriv_lpf_curoff_hz);
  lpf_yaw_error_.initialize(ctrl_period_, error_deriv_lpf_curoff_hz);

  /* set up ros system */
  timer_control_ = nh_.createTimer(ros::Duration(ctrl_period_), &MPCFollower::timerCallback, this);
  pub_debug_steer_cmd_ = pnh_.advertise<autoware_vehicle_msgs::Steering>("debug/steering_cmd", 1);
  pub_ctrl_cmd_ =
    pnh_.advertise<autoware_control_msgs::ControlCommandStamped>("output/control_raw", 1);
  sub_ref_path_ =
    pnh_.subscribe("input/reference_trajectory", 1, &MPCFollower::callbackTrajectory, this);
  sub_current_vel_ =
    pnh_.subscribe("input/current_velocity", 1, &MPCFollower::callbackCurrentVelocity, this);
  sub_steering_ = pnh_.subscribe("input/current_steering", 1, &MPCFollower::callbackSteering, this);

  /* wait to get vehicle position */
  while (ros::ok()) {
    try {
      tf_buffer_.lookupTransform("map", "base_link", ros::Time::now(), ros::Duration(5.0));
      break;
    } catch (tf2::TransformException & ex) {
      ROS_INFO("[mpc_follower] is waitting to get map to base_link transform. %s", ex.what());
      continue;
    }
  }

  /* for debug */
  pub_debug_marker_ = pnh_.advertise<visualization_msgs::MarkerArray>("debug/markers", 10);
  pub_debug_mpc_calc_time_ = pnh_.advertise<std_msgs::Float32>("debug/mpc_calc_time", 1);
  pub_debug_values_ = pnh_.advertise<std_msgs::Float32MultiArray>("debug/debug_values", 1);

  /* dynamic reconfigure */
  dynamic_reconfigure::Server<mpc_follower::MPCFollowerConfig>::CallbackType dyncon_f =
    boost::bind(&MPCFollower::dynamicRecofCallback, this, _1, _2);
  dyncon_server_.setCallback(dyncon_f);
}

MPCFollower::~MPCFollower()
{
  autoware_control_msgs::ControlCommand stop_cmd = getStopControlCommand();
  publishCtrlCmd(stop_cmd);
}

void MPCFollower::timerCallback(const ros::TimerEvent & te)
{
  updateCurrentPose();

  if (!checkData()) {
    publishCtrlCmd(getStopControlCommand());
    return;
  }

  autoware_control_msgs::ControlCommand ctrl_cmd;
  const bool is_mpc_solved = calculateMPC(&ctrl_cmd);

  if (!is_mpc_solved) {
    ROS_WARN_DELAYED_THROTTLE(5.0, "[MPC] MPC is not solved. publish 0 velocity.");
    ctrl_cmd = getStopControlCommand();
  }

  publishCtrlCmd(ctrl_cmd);
}

bool MPCFollower::checkData()
{
  if (!vehicle_model_ptr_ || !qpsolver_ptr_) {
    ROS_INFO_COND(
      show_debug_info_, "[MPC] vehicle_model = %d, qp_solver = %d", vehicle_model_ptr_ != nullptr,
      qpsolver_ptr_ != nullptr);
    return false;
  }

  if (!current_pose_ptr_ || !current_velocity_ptr_ || !current_steer_ptr_) {
    ROS_INFO_COND(
      show_debug_info_, "[MPC] waiting data. pose = %d, velocity = %d,  steer = %d",
      current_pose_ptr_ != nullptr, current_velocity_ptr_ != nullptr,
      current_steer_ptr_ != nullptr);
    return false;
  }

  if (ref_traj_.size() == 0) {
    ROS_INFO_COND(show_debug_info_, "[MPC] trajectory size is zero.");
    return false;
  }

  return true;
}

bool MPCFollower::calculateMPC(autoware_control_msgs::ControlCommand * ctrl_cmd)
{
  auto start = std::chrono::system_clock::now();

  if (!ctrl_cmd) {
    return false;
  }

  int nearest_idx;
  double nearest_time, steer, lat_err, yaw_err;
  geometry_msgs::Pose nearest_pose;
  if (!getVar(&nearest_idx, &nearest_time, &nearest_pose, &steer, &lat_err, &yaw_err)) {
    return false;
  }

  /* define initial state for error dynamics */
  Eigen::VectorXd x0 = getInitialState(lat_err, yaw_err, steer);

  /* delay compensation */
  if (!updateStateForDelayCompensation(nearest_time, &x0)) {
    ROS_WARN_DELAYED_THROTTLE(
      1.0, "[MPC] updateStateForDelayCompensation failed. stop computation.");
    return false;
  }

  /* overwrite trajectory velocity with current velocity */
  MPCTrajectory ref_traj_actvel = calcActualVelocity(nearest_idx, ref_traj_);

  /* resample ref_traj with mpc sampling time */
  MPCTrajectory mpc_resampled_ref_traj;
  const double mpc_start_time = nearest_time + mpc_param_.input_delay;
  if (!resampleMPCTrajectoryByTime(mpc_start_time, ref_traj_actvel, &mpc_resampled_ref_traj)) {
    return false;
  }

  /* generate mpc matrix : predict equation Xec = Aex * x0 + Bex * Uex + Wex */
  MPCMatrix mpc_matrix = generateMPCMatrix(mpc_resampled_ref_traj);

  /* solve quadratic optimization */
  Eigen::VectorXd Uex;
  if (!executeOptimization(mpc_matrix, x0, &Uex)) {
    return false;
  }

  /* apply saturation and filter */
  const double u_saturated = std::max(std::min(Uex(0), steer_lim_), -steer_lim_);
  const double u_filtered = lpf_steering_cmd_.filter(u_saturated);

  /* set control command */
  const int prev_idx = std::max(0, static_cast<int>(nearest_idx) - 1);
  ctrl_cmd->steering_angle = u_filtered;
  ctrl_cmd->steering_angle_velocity = (Uex(1) - Uex(0)) / mpc_param_.prediction_dt;
  ctrl_cmd->velocity = ref_traj_.vx[nearest_idx];
  ctrl_cmd->acceleration =
    (ref_traj_.vx[nearest_idx] - ref_traj_.vx[prev_idx]) / mpc_param_.prediction_dt;

  /* save input to buffer for delay compensation*/
  input_buffer_.push_back(ctrl_cmd->steering_angle);
  input_buffer_.pop_front();
  raw_steer_cmd_pprev_ = raw_steer_cmd_prev_;
  raw_steer_cmd_prev_ = Uex(0);

  /* ---------- DEBUG ---------- */

  /* calculate predicted trajectory */
  Eigen::VectorXd Xex = mpc_matrix.Aex * x0 + mpc_matrix.Bex * Uex + mpc_matrix.Wex;
  MPCTrajectory debug_mpc_predicted_traj;
  for (int i = 0; i < mpc_param_.prediction_horizon; ++i) {
    const int DIM_X = vehicle_model_ptr_->getDimX();
    const double lat_error = Xex(i * DIM_X);
    const double yaw_error = Xex(i * DIM_X + 1);
    const double x =
      mpc_resampled_ref_traj.x[i] - std::sin(mpc_resampled_ref_traj.yaw[i]) * lat_error;
    const double y =
      mpc_resampled_ref_traj.y[i] + std::cos(mpc_resampled_ref_traj.yaw[i]) * lat_error;
    const double z = mpc_resampled_ref_traj.z[i];
    debug_mpc_predicted_traj.push_back(x, y, z, mpc_resampled_ref_traj.yaw[i] + yaw_error, 0, 0, 0);
  }

  /* publish for visualization */
  visualization_msgs::MarkerArray markers = MPCUtils::convertTrajToMarker(
    debug_mpc_predicted_traj, "predicted_trajectory", 0.99, 0.99, 0.99, 0.2,
    current_trajectory_ptr_->header.frame_id);
  pub_debug_marker_.publish(markers);

  /* publish debug values */
  {
    double curr_v = current_velocity_ptr_->twist.linear.x;
    double nearest_k = 0.0;
    LinearInterpolate::interpolate(ref_traj_.relative_time, ref_traj_.k, nearest_time, nearest_k);

    MPCTrajectory tmp_traj = ref_traj_;
    MPCUtils::calcTrajectoryCurvature(1, &tmp_traj);
    double curvature_raw = tmp_traj.k[nearest_idx];
    double steer_cmd = ctrl_cmd->steering_angle;

    std_msgs::Float32MultiArray debug_values;
    debug_values.data.push_back(steer_cmd);             // [0] final steering command (MPC + LPF)
    debug_values.data.push_back(Uex(0));                // [1] mpc calculation result
    debug_values.data.push_back(mpc_matrix.Urefex(0));  // [2] feedforward steering value
    debug_values.data.push_back(
      std::atan(nearest_k * wheelbase_));  // [3] feedforward steering value raw
    debug_values.data.push_back(steer);    // [4] current steering angle
    debug_values.data.push_back(lat_err);  // [5] lateral error
    debug_values.data.push_back(
      tf2::getYaw(current_pose_ptr_->pose.orientation));                 // [6] current_pose yaw
    debug_values.data.push_back(tf2::getYaw(nearest_pose.orientation));  // [7] nearest_pose yaw
    debug_values.data.push_back(yaw_err);                                // [8] yaw error
    debug_values.data.push_back(ctrl_cmd->velocity);                     // [9] command velocitys
    debug_values.data.push_back(current_velocity_ptr_->twist.linear.x);  // [10] measured velocity
    debug_values.data.push_back(
      curr_v * tan(steer_cmd) / wheelbase_);  // [11] angvel from steer comand (MPC assumes)
    debug_values.data.push_back(
      curr_v * tan(steer) / wheelbase_);  // [12] angvel from measured steer
    debug_values.data.push_back(
      curr_v * nearest_k);                       // [13] angvel from path curvature (Path angvel)
    debug_values.data.push_back(nearest_k);      // [14] nearest path curvature (used for control)
    debug_values.data.push_back(curvature_raw);  // [15] nearest path curvature (not smoothed)
    pub_debug_values_.publish(debug_values);
  }

  /* publish computing time */
  auto end = std::chrono::system_clock::now();
  double elapsed_ms =
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() * 1.0e-6;
  std_msgs::Float32 mpc_calc_time_msg;
  mpc_calc_time_msg.data = elapsed_ms;
  pub_debug_mpc_calc_time_.publish(mpc_calc_time_msg);

  return true;
}

bool MPCFollower::getVar(
  int * nearest_idx, double * nearest_time, geometry_msgs::Pose * nearest_pose, double * steer,
  double * lat_err, double * yaw_err)
{
  if (!MPCUtils::calcNearestPoseInterp(
        ref_traj_, current_pose_ptr_->pose, nearest_pose, nearest_idx, nearest_time)) {
    ROS_WARN_DELAYED_THROTTLE(
      5.0, "[MPC] calculateMPC: error in calculating nearest pose. stop mpc.");
    return false;
  }

  *steer = *current_steer_ptr_;
  *lat_err = MPCUtils::calcLateralError(current_pose_ptr_->pose, *nearest_pose);
  *yaw_err = MPCUtils::normalizeRadian(
    tf2::getYaw(current_pose_ptr_->pose.orientation) - tf2::getYaw(nearest_pose->orientation));
  /* check error limit */
  const double dist_err = MPCUtils::calcDist2d(current_pose_ptr_->pose, *nearest_pose);
  if (dist_err > admisible_position_error_) {
    ROS_WARN_DELAYED_THROTTLE(
      5.0, "[MPC] position error is over limit. error = %fm, limit: %fm", dist_err,
      admisible_position_error_);
    return false;
  }
  /* check yaw error limit */
  if (std::fabs(*yaw_err) > admisible_yaw_error_) {
    ROS_WARN_DELAYED_THROTTLE(
      5.0, "[MPC] yaw error is over limit. error = %fdeg, limit %fdeg", RAD2DEG * (*yaw_err),
      RAD2DEG * admisible_yaw_error_);
    return false;
  }
  /* check trajectory time length */
  if (
    *nearest_time + mpc_param_.input_delay + getPredictionTime() > ref_traj_.relative_time.back()) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MPC] path is too short for prediction.");
    return false;
  }
  return true;
}

bool MPCFollower::resampleMPCTrajectoryByTime(
  double ts, const MPCTrajectory & input, MPCTrajectory * output) const
{
  std::vector<double> mpc_time_v;
  for (int i = 0; i < mpc_param_.prediction_horizon; ++i) {
    mpc_time_v.push_back(ts + i * mpc_param_.prediction_dt);
  }
  if (!MPCUtils::linearInterpMPCTrajectory(input.relative_time, input, mpc_time_v, output)) {
    ROS_WARN_DELAYED_THROTTLE(
      1.0, "[MPC] calculateMPC: mpc resample error. stop mpc calculation. check code!");
    return false;
  }
  return true;
}

Eigen::VectorXd MPCFollower::getInitialState(
  const double & lat_err, const double & yaw_err, const double & steer)
{
  const int DIM_X = vehicle_model_ptr_->getDimX();
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(DIM_X);

  if (vehicle_model_type_ == "kinematics") {
    x0 << lat_err, yaw_err, steer;
  } else if (vehicle_model_type_ == "kinematics_no_delay") {
    x0 << lat_err, yaw_err;
  } else if (vehicle_model_type_ == "dynamics") {
    double dot_lat_err = (lat_err - lateral_error_prev_) / ctrl_period_;
    double dot_yaw_err = (yaw_err - yaw_error_prev_) / ctrl_period_;
    lateral_error_prev_ = lat_err;
    yaw_error_prev_ = yaw_err;
    dot_lat_err = lpf_lateral_error_.filter(dot_lat_err);
    dot_yaw_err = lpf_yaw_error_.filter(dot_yaw_err);
    x0 << lat_err, dot_lat_err, yaw_err, dot_yaw_err;
    ROS_INFO_COND(
      show_debug_info_, "[MPC] (before lpf) dot_lat_err = %f, dot_yaw_err = %f", dot_lat_err,
      dot_yaw_err);
    ROS_INFO_COND(
      show_debug_info_, "[MPC] (after lpf) dot_lat_err = %f, dot_yaw_err = %f", dot_lat_err,
      dot_yaw_err);
  } else {
    ROS_ERROR("vehicle_model_type is undefined");
  }
  return x0;
}

bool MPCFollower::updateStateForDelayCompensation(const double & start_time, Eigen::VectorXd * x)
{
  const int DIM_X = vehicle_model_ptr_->getDimX();
  const int DIM_U = vehicle_model_ptr_->getDimU();
  const int DIM_Y = vehicle_model_ptr_->getDimY();

  Eigen::MatrixXd Ad(DIM_X, DIM_X);
  Eigen::MatrixXd Bd(DIM_X, DIM_U);
  Eigen::MatrixXd Wd(DIM_X, 1);
  Eigen::MatrixXd Cd(DIM_Y, DIM_X);

  Eigen::MatrixXd x_curr = *x;
  double mpc_curr_time = start_time;
  for (unsigned int i = 0; i < input_buffer_.size(); ++i) {
    double k = 0.0;
    double v = 0.0;
    if (
      !LinearInterpolate::interpolate(ref_traj_.relative_time, ref_traj_.k, mpc_curr_time, k) ||
      !LinearInterpolate::interpolate(ref_traj_.relative_time, ref_traj_.vx, mpc_curr_time, v)) {
      ROS_ERROR(
        "[MPC] mpc resample error at delay compensation, stop mpc calculation. check code!");
      return false;
    }

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setVelocity(v);
    vehicle_model_ptr_->setCurvature(k);
    vehicle_model_ptr_->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, ctrl_period_);
    Eigen::MatrixXd ud = Eigen::MatrixXd::Zero(DIM_U, 1);
    ud(0, 0) = input_buffer_.at(i);  // for steering input delay
    x_curr = Ad * x_curr + Bd * ud + Wd;
    mpc_curr_time += ctrl_period_;
  }
  *x = x_curr;
  return true;
}

MPCTrajectory MPCFollower::calcActualVelocity(const int nearest, const MPCTrajectory & input)
{
  MPCTrajectory output = input;
  const double tau_velocity = 0.01;
  const double acc_lim_velocity = 3.0;
  MPCUtils::dynamicSmoothingVelocity(
    nearest, current_velocity_ptr_->twist.linear.x, acc_lim_velocity, tau_velocity, &output);
  const double predict_time = (mpc_param_.prediction_horizon + 1) * mpc_param_.prediction_dt +
                              mpc_param_.input_delay + ctrl_period_;
  const double t_end = output.relative_time.back() + predict_time;
  const double v_end = 0.0;
  output.vx.back() = v_end;  // set for end point
  output.push_back(
    output.x.back(), output.y.back(), output.z.back(), output.yaw.back(), v_end, output.k.back(),
    t_end);
  return output;
}

/*
 * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Urefex) + Uex' * R2ex * Uex
 * Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 */
MPCFollower::MPCMatrix MPCFollower::generateMPCMatrix(const MPCTrajectory & reference_trajectory)
{
  const int N = mpc_param_.prediction_horizon;
  const double DT = mpc_param_.prediction_dt;
  const int DIM_X = vehicle_model_ptr_->getDimX();
  const int DIM_U = vehicle_model_ptr_->getDimU();
  const int DIM_Y = vehicle_model_ptr_->getDimY();

  Eigen::MatrixXd Aex = Eigen::MatrixXd::Zero(DIM_X * N, DIM_X);
  Eigen::MatrixXd Bex = Eigen::MatrixXd::Zero(DIM_X * N, DIM_U * N);
  Eigen::MatrixXd Wex = Eigen::MatrixXd::Zero(DIM_X * N, 1);
  Eigen::MatrixXd Cex = Eigen::MatrixXd::Zero(DIM_Y * N, DIM_X * N);
  Eigen::MatrixXd Qex = Eigen::MatrixXd::Zero(DIM_Y * N, DIM_Y * N);
  Eigen::MatrixXd R1ex = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  Eigen::MatrixXd R2ex = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  Eigen::MatrixXd Urefex = Eigen::MatrixXd::Zero(DIM_U * N, 1);

  /* weight matrix depends on the vehicle model */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Eigen::MatrixXd Q_adaptive = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R_adaptive = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Q(0, 0) = mpc_param_.weight_lat_error;
  Q(1, 1) = mpc_param_.weight_heading_error;
  R(0, 0) = mpc_param_.weight_steering_input;

  Eigen::MatrixXd Ad(DIM_X, DIM_X);
  Eigen::MatrixXd Bd(DIM_X, DIM_U);
  Eigen::MatrixXd Wd(DIM_X, 1);
  Eigen::MatrixXd Cd(DIM_Y, DIM_X);
  Eigen::MatrixXd Uref(DIM_U, 1);

  /* predict dynamics for N times */
  for (int i = 0; i < N; ++i) {
    const double ep = 1.0e-3;
    const double sign_vx =
      reference_trajectory.vx[i] > ep ? 1 : (reference_trajectory.vx[i] < -ep ? -1 : 0);
    const double ref_k = reference_trajectory.k[i] * sign_vx;
    const double ref_vx = reference_trajectory.vx[i];
    const double ref_vx_squared = ref_vx * ref_vx;

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setVelocity(ref_vx);
    vehicle_model_ptr_->setCurvature(ref_k);
    vehicle_model_ptr_->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, DT);

    Q_adaptive = Q;
    R_adaptive = R;
    if (i == N - 1) {
      Q_adaptive(0, 0) = mpc_param_.weight_terminal_lat_error;
      Q_adaptive(1, 1) = mpc_param_.weight_terminal_heading_error;
    }
    Q_adaptive(1, 1) += ref_vx_squared * mpc_param_.weight_heading_error_squared_vel;
    R_adaptive(0, 0) += ref_vx_squared * mpc_param_.weight_steering_input_squared_vel;

    /* update mpc matrix */
    int idx_x_i = i * DIM_X;
    int idx_x_i_prev = (i - 1) * DIM_X;
    int idx_u_i = i * DIM_U;
    int idx_y_i = i * DIM_Y;
    if (i == 0) {
      Aex.block(0, 0, DIM_X, DIM_X) = Ad;
      Bex.block(0, 0, DIM_X, DIM_U) = Bd;
      Wex.block(0, 0, DIM_X, 1) = Wd;
    } else {
      Aex.block(idx_x_i, 0, DIM_X, DIM_X) = Ad * Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
      for (int j = 0; j < i; ++j) {
        int idx_u_j = j * DIM_U;
        Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) =
          Ad * Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
      }
      Wex.block(idx_x_i, 0, DIM_X, 1) = Ad * Wex.block(idx_x_i_prev, 0, DIM_X, 1) + Wd;
    }
    Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = Bd;
    Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = Cd;
    Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q_adaptive;
    R1ex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R_adaptive;

    /* get reference input (feed-forward) */
    vehicle_model_ptr_->calculateReferenceInput(Uref);
    if (std::fabs(Uref(0, 0)) < DEG2RAD * mpc_param_.zero_ff_steer_deg) {
      Uref(0, 0) = 0.0;  // ignore curvature noise
    }
    Urefex.block(i * DIM_U, 0, DIM_U, 1) = Uref;
  }

  /* add lateral jerk : weight for (v * {u(i) - u(i-1)} )^2 */
  for (int i = 0; i < N - 1; ++i) {
    const double v = reference_trajectory.vx[i];
    const double lateral_jerk_r = v * v * mpc_param_.weight_lat_jerk / (DT * DT);
    R2ex(i + 0, i + 0) += lateral_jerk_r;
    R2ex(i + 1, i + 0) -= lateral_jerk_r;
    R2ex(i + 0, i + 1) -= lateral_jerk_r;
    R2ex(i + 1, i + 1) += lateral_jerk_r;
  }

  addSteerWeightR(&R1ex);

  MPCMatrix m;
  m.Aex = Aex;
  m.Bex = Bex;
  m.Wex = Wex;
  m.Cex = Cex;
  m.Qex = Qex;
  m.R1ex = R1ex;
  m.R2ex = R2ex;
  m.Urefex = Urefex;
  return m;
}

/*
 * solve quadratic optimization.
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Urefex) + Uex' * R2ex * Uex
 *                , Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 * constraint matrix : lb < U < ub, lbA < A*U < ubA
 * current considered constraint
 *  - steering limit
 *  - steering rate limit
 *
 * (1)lb < u < ub && (2)lbA < Au < ubA --> (3)[lb, lbA] < [I, A]u < [ub, ubA]
 * (1)lb < u < ub ...
 * [-u_lim] < [ u0 ] < [u_lim]
 * [-u_lim] < [ u1 ] < [u_lim]
 *              ~~~
 * [-u_lim] < [ uN ] < [u_lim] (*N... DIM_U)
 * (2)lbA < Au < ubA ...
 * [prev_u0 - au_lim*ctp] < [   u0  ] < [prev_u0 + au_lim*ctp] (*ctp ... ctrl_period)
 * [    -au_lim * dt    ] < [u1 - u0] < [     au_lim * dt    ]
 * [    -au_lim * dt    ] < [u2 - u1] < [     au_lim * dt    ]
 *                            ~~~
 * [    -au_lim * dt    ] < [uN-uN-1] < [     au_lim * dt    ] (*N... DIM_U)
 */
bool MPCFollower::executeOptimization(
  const MPCMatrix & m, const Eigen::VectorXd & x0, Eigen::VectorXd * Uex)
{
  if (
    m.Aex.array().isNaN().any() || m.Bex.array().isNaN().any() || m.Cex.array().isNaN().any() ||
    m.Wex.array().isNaN().any() || m.Qex.array().isNaN().any() || m.R1ex.array().isNaN().any() ||
    m.R2ex.array().isNaN().any() || m.Urefex.array().isNaN().any()) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MPC] model matrix includes NaN, stop MPC.");
    return false;
  }

  if (
    m.Aex.array().isInf().any() || m.Bex.array().isInf().any() || m.Cex.array().isInf().any() ||
    m.Wex.array().isInf().any() || m.Qex.array().isInf().any() || m.R1ex.array().isInf().any() ||
    m.R2ex.array().isInf().any() || m.Urefex.array().isInf().any()) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MPC] model matrix includes Inf, stop MPC.");
    return false;
  }

  const int DIM_U_N = mpc_param_.prediction_horizon * vehicle_model_ptr_->getDimU();

  // cost function: 1/2 * Uex' * H * Uex + f' * Uex,  H = B' * C' * Q * C * B + R
  const Eigen::MatrixXd CB = m.Cex * m.Bex;
  const Eigen::MatrixXd QCB = m.Qex * CB;
  // Eigen::MatrixXd H = CB.transpose() * QCB + m.R1ex + m.R2ex; // This calculation is heavy. looking for a good way.
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(DIM_U_N, DIM_U_N);
  H.triangularView<Eigen::Upper>() = CB.transpose() * QCB;
  H.triangularView<Eigen::Upper>() += m.R1ex + m.R2ex;
  H.triangularView<Eigen::Lower>() = H.transpose();
  Eigen::MatrixXd f =
    (m.Cex * (m.Aex * x0 + m.Wex)).transpose() * QCB - m.Urefex.transpose() * m.R1ex;
  addSteerWeightF(&f);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(DIM_U_N, DIM_U_N);
  for (int i = 1; i < DIM_U_N; i++) {
    A(i, i - 1) = -1.0;
  }

  Eigen::VectorXd lb = Eigen::VectorXd::Constant(DIM_U_N, -steer_lim_);  // min steering angle
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(DIM_U_N, steer_lim_);   // max steering angle
  Eigen::VectorXd lbA =
    Eigen::VectorXd::Constant(DIM_U_N, -steer_rate_lim_ * mpc_param_.prediction_dt);
  Eigen::VectorXd ubA =
    Eigen::VectorXd::Constant(DIM_U_N, steer_rate_lim_ * mpc_param_.prediction_dt);
  lbA(0, 0) = raw_steer_cmd_prev_ - steer_rate_lim_ * ctrl_period_;
  ubA(0, 0) = raw_steer_cmd_prev_ + steer_rate_lim_ * ctrl_period_;

  auto t_start = std::chrono::system_clock::now();
  bool solve_result = qpsolver_ptr_->solve(H, f.transpose(), A, lb, ub, lbA, ubA, *Uex);
  auto t_end = std::chrono::system_clock::now();
  if (!solve_result) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MPC] qp solver error");
    return false;
  }

  double elapsed =
    std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count() * 1.0e-6;
  ROS_INFO_COND(show_debug_info_, "[MPC] qp solver calculation time = %f [ms]", elapsed);

  if (Uex->array().isNaN().any()) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[MPC] model Uex includes NaN, stop MPC. ");
    return false;
  }
  return true;
}

void MPCFollower::addSteerWeightR(Eigen::MatrixXd * R) const
{
  const int N = mpc_param_.prediction_horizon;
  const double DT = mpc_param_.prediction_dt;

  /* add steering rate : weight for (u(i) - u(i-1) / dt )^2 */
  for (int i = 0; i < N - 1; ++i) {
    const double steer_rate_r = mpc_param_.weight_steer_rate / (DT * DT);
    (*R)(i + 0, i + 0) += steer_rate_r;
    (*R)(i + 1, i + 0) -= steer_rate_r;
    (*R)(i + 0, i + 1) -= steer_rate_r;
    (*R)(i + 1, i + 1) += steer_rate_r;
  }
  if (N > 1) {
    // steer rate i = 0
    (*R)(0, 0) += mpc_param_.weight_steer_rate / (ctrl_period_ * ctrl_period_);
  }

  /* add steering acceleration : weight for { (u(i+1) - 2*u(i) + u(i-1)) / dt^2 }^2 */
  const double steer_acc_r = mpc_param_.weight_steer_acc / std::pow(DT, 4);
  const double steer_acc_r_cp1 = mpc_param_.weight_steer_acc / (std::pow(DT, 3) * ctrl_period_);
  const double steer_acc_r_cp2 =
    mpc_param_.weight_steer_acc / (std::pow(DT, 2) * std::pow(ctrl_period_, 2));
  const double steer_acc_r_cp4 = mpc_param_.weight_steer_acc / std::pow(ctrl_period_, 4);
  for (int i = 1; i < N - 1; ++i) {
    (*R)(i - 1, i - 1) += (steer_acc_r);
    (*R)(i - 1, i + 0) += (steer_acc_r * -2.0);
    (*R)(i - 1, i + 1) += (steer_acc_r);
    (*R)(i + 0, i - 1) += (steer_acc_r * -2.0);
    (*R)(i + 0, i + 0) += (steer_acc_r * 4.0);
    (*R)(i + 0, i + 1) += (steer_acc_r * -2.0);
    (*R)(i + 1, i - 1) += (steer_acc_r);
    (*R)(i + 1, i + 0) += (steer_acc_r * -2.0);
    (*R)(i + 1, i + 1) += (steer_acc_r);
  }
  if (N > 1) {
    // steer acc i = 1
    (*R)(0, 0) += steer_acc_r * 1.0 + steer_acc_r_cp2 * 1.0 + steer_acc_r_cp1 * 2.0;
    (*R)(1, 0) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
    (*R)(0, 1) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
    (*R)(1, 1) += steer_acc_r * 1.0;
    // steer acc i = 0
    (*R)(0, 0) += steer_acc_r_cp4 * 1.0;
  }
}

void MPCFollower::addSteerWeightF(Eigen::MatrixXd * f) const
{
  if (f->rows() < 2) {
    return;
  }

  const double DT = mpc_param_.prediction_dt;

  // steer rate for i = 0
  (*f)(0, 0) += -2.0 * mpc_param_.weight_steer_rate / (std::pow(DT, 2)) * 0.5;

  // const double steer_acc_r = mpc_param_.weight_steer_acc / std::pow(DT, 4);
  const double steer_acc_r_cp1 = mpc_param_.weight_steer_acc / (std::pow(DT, 3) * ctrl_period_);
  const double steer_acc_r_cp2 =
    mpc_param_.weight_steer_acc / (std::pow(DT, 2) * std::pow(ctrl_period_, 2));
  const double steer_acc_r_cp4 = mpc_param_.weight_steer_acc / std::pow(ctrl_period_, 4);

  // steer acc  i = 0
  (*f)(0, 0) += ((-2.0 * raw_steer_cmd_prev_ + raw_steer_cmd_pprev_) * steer_acc_r_cp4) * 0.5;

  // steer acc for i = 1
  (*f)(0, 0) += (-2.0 * raw_steer_cmd_prev_ * (steer_acc_r_cp1 + steer_acc_r_cp2)) * 0.5;
  (*f)(0, 1) += (2.0 * raw_steer_cmd_prev_ * steer_acc_r_cp1) * 0.5;
}

double MPCFollower::getPredictionTime() const
{
  return (mpc_param_.prediction_horizon - 1) * mpc_param_.prediction_dt + mpc_param_.input_delay +
         ctrl_period_;
}

void MPCFollower::callbackTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & msg)
{
  current_trajectory_ptr_ = std::make_shared<autoware_planning_msgs::Trajectory>(*msg);

  if (msg->points.size() < 3) {
    ROS_INFO_COND(show_debug_info_, "[MPC] received path size is < 3, not enough.");
    return;
  }

  MPCTrajectory mpc_traj_raw;        // received raw trajectory
  MPCTrajectory mpc_traj_resampled;  // resampled trajectory
  MPCTrajectory mpc_traj_smoothed;   // smooth fitltered trajectory

  /* resampling */
  MPCUtils::convertToMPCTrajectory(*current_trajectory_ptr_, &mpc_traj_raw);
  if (!MPCUtils::resampleMPCTrajectoryByDistance(
        mpc_traj_raw, traj_resample_dist_, &mpc_traj_resampled)) {
    ROS_WARN("spline error!!!!!!");
    return;
  }

  /* path smoothing */
  mpc_traj_smoothed = mpc_traj_resampled;
  int mpc_traj_resampled_size = static_cast<int>(mpc_traj_resampled.size());
  if (enable_path_smoothing_ && mpc_traj_resampled_size > 2 * path_filter_moving_ave_num_) {
    if (
      !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, mpc_traj_smoothed.x) ||
      !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, mpc_traj_smoothed.y) ||
      !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, mpc_traj_smoothed.yaw) ||
      !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, mpc_traj_smoothed.vx)) {
      ROS_INFO_COND(show_debug_info_, "[MPC] path callback: filtering error. stop filtering.");
      mpc_traj_smoothed = mpc_traj_resampled;
    }
  }

  /* calculate yaw angle */
  if (enable_yaw_recalculation_) {
    MPCUtils::calcTrajectoryYawFromXY(&mpc_traj_smoothed);
    MPCUtils::convertEulerAngleToMonotonic(&mpc_traj_smoothed.yaw);
  }

  /* calculate curvature */
  MPCUtils::calcTrajectoryCurvature(curvature_smoothing_num_, &mpc_traj_smoothed);

  /* add end point with vel=0 on traj for mpc prediction */
  const double t_end =
    mpc_traj_smoothed.relative_time.back() + getPredictionTime() + mpc_param_.input_delay + 100.0;
  const double v_end = 0.0;
  mpc_traj_smoothed.vx.back() = v_end;  // set for end point
  mpc_traj_smoothed.push_back(
    mpc_traj_smoothed.x.back(), mpc_traj_smoothed.y.back(), mpc_traj_smoothed.z.back(),
    mpc_traj_smoothed.yaw.back(), v_end, mpc_traj_smoothed.k.back(), t_end);

  if (!mpc_traj_smoothed.size()) {
    ROS_INFO_COND(show_debug_info_, "[MPC] path callback: trajectory size is undesired.");
    return;
  }

  ref_traj_ = mpc_traj_smoothed;

  /* publish debug marker */
  visualization_msgs::MarkerArray markers;
  std::string frame = msg->header.frame_id;
  markers =
    MPCUtils::convertTrajToMarker(mpc_traj_raw, "trajectory raw", 0.9, 0.5, 0.0, 0.05, frame);
  pub_debug_marker_.publish(markers);
  markers = MPCUtils::convertTrajToMarker(
    mpc_traj_resampled, "trajectory spline", 0.5, 0.1, 1.0, 0.05, frame);
  pub_debug_marker_.publish(markers);
  markers = MPCUtils::convertTrajToMarker(
    mpc_traj_smoothed, "trajectory average filtered", 0.0, 1.0, 0.0, 0.05, frame);
  pub_debug_marker_.publish(markers);
}

void MPCFollower::updateCurrentPose()
{
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN_DELAYED_THROTTLE(
      5.0, "[mpc_follower] cannot get map to base_link transform. %s", ex.what());
    return;
  }

  geometry_msgs::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  current_pose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(ps);
}

void MPCFollower::callbackSteering(const autoware_vehicle_msgs::Steering & msg)
{
  current_steer_ptr_ = std::make_shared<double>(msg.data);
}

void MPCFollower::callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  current_velocity_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(*msg);
}

autoware_control_msgs::ControlCommand MPCFollower::getStopControlCommand() const
{
  autoware_control_msgs::ControlCommand cmd;
  cmd.steering_angle = steer_cmd_prev_;
  cmd.steering_angle_velocity = 0.0;
  cmd.velocity = 0.0;
  cmd.acceleration = -1.5;
  return cmd;
}

void MPCFollower::publishCtrlCmd(const autoware_control_msgs::ControlCommand & ctrl_cmd)
{
  autoware_control_msgs::ControlCommandStamped cmd;
  cmd.header.frame_id = "base_link";
  cmd.header.stamp = ros::Time::now();
  cmd.control = ctrl_cmd;
  pub_ctrl_cmd_.publish(cmd);

  steer_cmd_prev_ = ctrl_cmd.steering_angle;

  autoware_vehicle_msgs::Steering s;
  s.data = ctrl_cmd.steering_angle;
  s.header = cmd.header;
  pub_debug_steer_cmd_.publish(s);
}
