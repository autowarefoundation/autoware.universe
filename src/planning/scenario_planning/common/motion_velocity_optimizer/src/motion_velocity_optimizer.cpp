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

#include <chrono>
#include <motion_velocity_optimizer/motion_velocity_optimizer.hpp>

// clang-format off
#define DEBUG_INFO(...) { if (show_debug_info_) {ROS_INFO(__VA_ARGS__); } }
#define DEBUG_WARN(...) { if (show_debug_info_) {ROS_WARN(__VA_ARGS__); } }
#define DEBUG_INFO_ALL(...) { if (show_debug_info_all_) {ROS_INFO(__VA_ARGS__); } }

// clang-format on
MotionVelocityOptimizer::MotionVelocityOptimizer() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  pnh_.param<double>("max_velocity", planning_param_.max_velocity, 20.0);  // 72.0 kmph
  pnh_.param<double>("max_accel", planning_param_.max_accel, 2.0);         // 0.11G
  pnh_.param<double>("min_decel", planning_param_.min_decel, -3.0);        // -0.2G

  pnh_.param<double>("max_lateral_accel", planning_param_.max_lateral_accel, 0.2);  //
  pnh_.param<double>(
    "decel_distance_before_curve", planning_param_.decel_distance_before_curve, 3.5);
  pnh_.param<double>("decel_distance_after_curve", planning_param_.decel_distance_after_curve, 0.0);
  pnh_.param<double>("min_curve_velocity", planning_param_.min_curve_velocity, 1.38);

  pnh_.param<double>("replan_vel_deviation", planning_param_.replan_vel_deviation, 3.0);
  pnh_.param<double>("engage_velocity", planning_param_.engage_velocity, 0.3);
  pnh_.param<double>("engage_acceleration", planning_param_.engage_acceleration, 0.1);
  pnh_.param<double>("engage_exit_ratio", planning_param_.engage_exit_ratio, 0.5);
  planning_param_.engage_exit_ratio =
    std::min(std::max(planning_param_.engage_exit_ratio, 0.0), 1.0);

  pnh_.param<double>("extract_ahead_dist", planning_param_.extract_ahead_dist, 200.0);
  pnh_.param<double>("extract_behind_dist", planning_param_.extract_behind_dist, 3.0);
  pnh_.param<double>("max_trajectory_length", planning_param_.max_trajectory_length, 200.0);
  pnh_.param<double>("min_trajectory_length", planning_param_.min_trajectory_length, 30.0);
  pnh_.param<double>("resample_time", planning_param_.resample_time, 10.0);
  pnh_.param<double>("resample_dt", planning_param_.resample_dt, 0.1);
  pnh_.param<double>(
    "min_trajectory_interval_distance", planning_param_.min_trajectory_interval_distance, 0.1);
  pnh_.param<double>(
    "stop_dist_to_prohibit_engage", planning_param_.stop_dist_to_prohibit_engage, 1.5);
  pnh_.param<double>("delta_yaw_threshold", planning_param_.delta_yaw_threshold, M_PI / 3.0);

  pnh_.param<bool>("show_debug_info", show_debug_info_, true);
  pnh_.param<bool>("show_debug_info_all", show_debug_info_all_, false);
  pnh_.param<bool>("show_figure", show_figure_, false);
  pnh_.param<bool>("publish_debug_trajs", publish_debug_trajs_, false);

  pnh_.param<double>("pseudo_jerk_weight", qp_param_.pseudo_jerk_weight, 1000.0);
  pnh_.param<double>("over_v_weight", qp_param_.over_v_weight, 100000.0);
  pnh_.param<double>("over_a_weight", qp_param_.over_a_weight, 1000.0);

  pub_trajectory_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1);
  pub_dist_to_stopline_ = pnh_.advertise<std_msgs::Float32>("distance_to_stopline", 1);
  sub_current_trajectory_ = pnh_.subscribe(
    "input/trajectory", 1, &MotionVelocityOptimizer::callbackCurrentTrajectory, this);
  sub_current_velocity_ = pnh_.subscribe(
    "/localization/twist", 1, &MotionVelocityOptimizer::callbackCurrentVelocity, this);
  sub_external_velocity_limit_ = pnh_.subscribe(
    "external_velocity_limit_mps", 1, &MotionVelocityOptimizer::callbackExternalVelocityLimit,
    this);

  /* dynamic reconfigure */
  dynamic_reconfigure::Server<
    motion_velocity_optimizer::MotionVelocityOptimizerConfig>::CallbackType dyncon_f =
    boost::bind(&MotionVelocityOptimizer::dynamicRecofCallback, this, _1, _2);
  dyncon_server_.setCallback(dyncon_f);

  /* debug */
  debug_closest_velocity_ = pnh_.advertise<std_msgs::Float32>("closest_velocity", 1);
  pub_trajectory_raw_ =
    pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/trajectory_raw", 1);
  pub_trajectory_vel_lim_ = pnh_.advertise<autoware_planning_msgs::Trajectory>(
    "debug/trajectory_external_velocity_limitted", 1);
  pub_trajectory_latcc_filtered_ =
    pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/trajectory_lateral_acc_filtered", 1);
  pub_trajectory_resampled_ =
    pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/trajectory_time_resampled", 1);

  const double timer_dt = 0.1;
  timer_ = nh_.createTimer(ros::Duration(timer_dt), &MotionVelocityOptimizer::timerCallback, this);

  /* wait to get vehicle position */
  while (ros::ok()) {
    try {
      tf_buffer_.lookupTransform("map", "base_link", ros::Time::now(), ros::Duration(5.0));
      break;
    } catch (tf2::TransformException & ex) {
      ROS_INFO(
        "[MotionVelocityOptimizer] is waitting to get map to base_link transform. %s", ex.what());
      continue;
    }
  }
}
MotionVelocityOptimizer::~MotionVelocityOptimizer() {}

void MotionVelocityOptimizer::publishTrajectory(
  const autoware_planning_msgs::Trajectory & trajectory) const
{
  pub_trajectory_.publish(trajectory);
}

void MotionVelocityOptimizer::callbackCurrentVelocity(
  const geometry_msgs::TwistStamped::ConstPtr msg)
{
  current_velocity_ptr_ = msg;
}
void MotionVelocityOptimizer::callbackCurrentTrajectory(
  const autoware_planning_msgs::Trajectory::ConstPtr msg)
{
  base_traj_raw_ptr_ = msg;
  run();
}
void MotionVelocityOptimizer::callbackExternalVelocityLimit(const std_msgs::Float32::ConstPtr msg)
{
  external_velocity_limit_ptr_ = msg;
}

void MotionVelocityOptimizer::updateCurrentPose()
{
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("[MotionVelocityOptimizer] cannot get map to base_link transform. %s", ex.what());
    return;
  }

  geometry_msgs::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  current_pose_ptr_ = boost::make_shared<geometry_msgs::PoseStamped>(ps);
}

void MotionVelocityOptimizer::run()
{
  auto t_start = std::chrono::system_clock::now();
  DEBUG_INFO("============================== run() start ==============================");

  updateCurrentPose();

  /* guard */
  if (
    !current_pose_ptr_ || !current_velocity_ptr_ || !base_traj_raw_ptr_ ||
    base_traj_raw_ptr_->points.empty()) {
    DEBUG_INFO(
      "wait topics : current_pose = %d, current_vel = %d, base_traj = %d, traj.size = %lu",
      (bool)current_pose_ptr_, (bool)current_velocity_ptr_, (bool)base_traj_raw_ptr_,
      base_traj_raw_ptr_->points.size());
    return;
  }

  /* calculate trajectory velocity */
  autoware_planning_msgs::Trajectory output = calcTrajectoryVelocity(*base_traj_raw_ptr_);

  /* publish message */
  output.header = base_traj_raw_ptr_->header;
  publishTrajectory(output);

  prev_output_ = output;

  auto t_end = std::chrono::system_clock::now();
  double elapsed_ms =
    std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count() * 1.0e-6;
  DEBUG_INFO("run: calculation time = %f [ms]", elapsed_ms);
  DEBUG_INFO("============================== run() end ==============================\n\n");
}

autoware_planning_msgs::Trajectory MotionVelocityOptimizer::calcTrajectoryVelocity(
  const autoware_planning_msgs::Trajectory & traj_input)
{
  /* Find the nearest point to reference_traj */
  int input_closest = vpu::calcClosestWaypoint(
    traj_input, current_pose_ptr_->pose, planning_param_.delta_yaw_threshold);
  if (input_closest < 0) {
    ROS_WARN("[velocity planner] cannot find closest waypoint for input trajectory");
    return prev_output_;
  }

  autoware_planning_msgs::Trajectory
    traj_extracted;  // extructed from traj_input around current_position
  autoware_planning_msgs::Trajectory
    traj_vel_limtted;  // velocity is limitted by external velocity limit
  autoware_planning_msgs::Trajectory
    traj_latacc_filtered;  // velocity is limitted by max lateral acceleration
  autoware_planning_msgs::Trajectory traj_resampled;  // resampled depending on the current_velocity
  autoware_planning_msgs::Trajectory output;          // velocity is optimized by qp solver

  /* Extract trajectory around self-position with desired forward-backwaed length*/
  if (!extractPathAroundIndex(traj_input, input_closest, /* out */ traj_extracted)) {
    return prev_output_;
  }

  /* Apply external velocity limit */
  externalVelocityLimitFilter(traj_extracted, /* out */ traj_vel_limtted);

  /* Lateral acceleration limt */
  if (!lateralAccelerationFilter(traj_vel_limtted, /* out */ traj_latacc_filtered)) {
    return prev_output_;
  }

  /* Resample trajectory with ego-velocity based interval distance */
  if (!resampleTrajectory(traj_latacc_filtered, /* out */ traj_resampled)) {
    return prev_output_;
  }
  int traj_resampled_closest = vpu::calcClosestWaypoint(
    traj_resampled, current_pose_ptr_->pose, planning_param_.delta_yaw_threshold);
  if (traj_resampled_closest < 0) {
    ROS_WARN("[velocity planner] cannot find closest waypoint for resampled trajectory");
    return prev_output_;
  }

  /* Change trajectory velocity to zero when current_velocity == 0 & stop_dist is close */
  preventMoveToCloseStopLine(traj_resampled_closest, traj_resampled);

  /* for reverse velocity */
  const bool is_reverse_velocity =
    (traj_resampled.points.at(traj_resampled_closest).twist.linear.x < 0.0);
  if (is_reverse_velocity) {
    vpu::multiplyConstantToTrajectoryVelocity(-1.0, /* out */ traj_resampled);
  }

  /* Calculate the closest point on the previously planned traj (used to get initial planning speed) */
  int prev_output_closest = vpu::calcClosestWaypoint(
    prev_output_, current_pose_ptr_->pose, planning_param_.delta_yaw_threshold);
  DEBUG_INFO(
    "[calcClosestWaypoint] for base_resampled : base_resampled.size() = %d, prev_planned_closest_ "
    "= %d",
    (int)traj_resampled.points.size(), prev_output_closest);

  /* Optimize velocity */
  optimizeVelocity(
    traj_resampled, traj_resampled_closest, prev_output_, prev_output_closest,
    /* out */ output);

  /* Max velocity filter for safety */
  vpu::maximumVelocityFilter(planning_param_.max_velocity, output);

  /* for negative velocity */
  if (is_reverse_velocity) {
    vpu::multiplyConstantToTrajectoryVelocity(-1.0, output);
  }

  /* Insert behind velocity for output's consistency */
  insertBehindVelocity(prev_output_closest, prev_output_, traj_resampled_closest, output);

  /* for debug */
  publishClosestVelocity(output.points.at(traj_resampled_closest).twist.linear.x);
  publishStopDistance(traj_resampled, traj_resampled_closest);
  if (publish_debug_trajs_) {
    pub_trajectory_raw_.publish(traj_extracted);
    pub_trajectory_vel_lim_.publish(traj_vel_limtted);
    pub_trajectory_latcc_filtered_.publish(traj_latacc_filtered);
    pub_trajectory_resampled_.publish(traj_resampled);
  }

  return output;
}

void MotionVelocityOptimizer::insertBehindVelocity(
  const int prev_output_closest, const autoware_planning_msgs::Trajectory & prev_output,
  const int output_closest, autoware_planning_msgs::Trajectory & output) const
{
  int j = std::max(prev_output_closest - 1, 0);
  for (int i = output_closest - 1; i >= 0; --i) {
    if (
      initialize_type_ == InitializeType::INIT ||
      initialize_type_ == InitializeType::LARGE_DEVIATION_REPLAN ||
      initialize_type_ == InitializeType::ENGAGING) {
      output.points.at(i).twist.linear.x = output.points.at(output_closest).twist.linear.x;
    } else {
      output.points.at(i).twist.linear.x = prev_output.points.at(j).twist.linear.x;
    }

    j = std::max(j - 1, 0);
  }
}

void MotionVelocityOptimizer::publishStopDistance(
  const autoware_planning_msgs::Trajectory & trajectory, const int closest) const
{
  /* stop distance calculation */
  int stop_idx = 0;
  const double stop_dist_lim = 50.0;
  double stop_dist = stop_dist_lim;
  if (vpu::searchZeroVelocityIdx(trajectory, stop_idx)) {
    stop_dist = vpu::calcLengthOnWaypoints(trajectory, closest, stop_idx);
  }
  stop_dist = closest > stop_idx ? stop_dist : -stop_dist;
  std_msgs::Float32 dist_to_stopline;
  dist_to_stopline.data = std::max(-stop_dist_lim, std::min(stop_dist_lim, stop_dist));
  pub_dist_to_stopline_.publish(dist_to_stopline);
}

bool MotionVelocityOptimizer::resampleTrajectory(
  const autoware_planning_msgs::Trajectory & input,
  autoware_planning_msgs::Trajectory & output) const
{
  std::vector<double> in_arclength;
  vpu::calcTrajectoryArclength(input, in_arclength);
  const double Nt = planning_param_.resample_time / std::max(planning_param_.resample_dt, 0.001);
  const double ds_nominal = std::max(
    current_velocity_ptr_->twist.linear.x * planning_param_.resample_dt,
    planning_param_.min_trajectory_interval_distance);
  const double Ns = planning_param_.min_trajectory_length / std::max(ds_nominal, 0.001);
  const double N = std::max(Nt, Ns);
  std::vector<double> out_arclength;
  double dist_i = 0.0;
  out_arclength.push_back(dist_i);
  bool is_endpoint_included = false;
  for (int i = 1; i <= N; ++i) {
    double ds = ds_nominal;
    if (i > Nt) {
      if (dist_i > planning_param_.min_trajectory_length)
        break;  // planning time is enough and planning distance is over min length.
      // if the planning time is not enough to see the desired distance, change the interval distance to see far.
      ds = std::max(1.0, ds_nominal);
    }
    dist_i += ds;
    if (dist_i > planning_param_.max_trajectory_length) {
      break;  // distance is over max.
    }
    if (dist_i >= in_arclength.back()) {
      is_endpoint_included = true;  // distance is over input endpoint.
      break;
    }
    out_arclength.push_back(dist_i);
  }
  if (!vpu::linearInterpTrajectory(in_arclength, input, out_arclength, output)) {
    ROS_WARN(
      "[motion_velocity_optimizer]: fail trajectory interpolation. size : in_arclength = %lu, "
      "input = %lu, "
      "out_arclength = %lu, output = %lu",
      in_arclength.size(), input.points.size(), out_arclength.size(), output.points.size());
    return false;
  }

  // add end point directly to consider the endpoint velocity.
  if (is_endpoint_included) {
    constexpr double ep_dist = 1.0E-3;
    if (vpu::calcDist2d(output.points.back(), input.points.back()) < ep_dist) {
      output.points.back() = input.points.back();
    } else {
      output.points.push_back(input.points.back());
    }
  }
  return true;
}

void MotionVelocityOptimizer::calcInitialMotion(
  const double & target_vel, const autoware_planning_msgs::Trajectory & reference_traj,
  const int reference_traj_closest, const autoware_planning_msgs::Trajectory & prev_output,
  const int prev_output_closest, double & initial_vel, double & initial_acc)
{
  const double vehicle_speed = std::fabs(current_velocity_ptr_->twist.linear.x);

  /* first time */
  if (prev_output.points.size() == 0 || prev_output_closest == -1) {
    initial_vel = vehicle_speed;
    initial_acc = 0.0;  // if possible, use actual vehicle acc & jerk value;
    initialize_type_ = InitializeType::INIT;
    return;
  }

  const double desired_vel = prev_output.points.at(prev_output_closest).twist.linear.x;
  const double vel_error = vehicle_speed - std::fabs(desired_vel);
  if (
    std::fabs(vel_error) >
    planning_param_.replan_vel_deviation /* when velocity tracking deviation is large */) {
    initial_vel = vehicle_speed;  // use current vehicle speed
    initial_acc = prev_output.points.at(prev_output_closest).accel.linear.x;
    DEBUG_WARN(
      "[calcInitialMotion] : Large deviation error for speed control. Use current speed for "
      "initial value, "
      "desired_vel = %f, vehicle_speed = %f, vel_error = %f, error_thr = %f",
      desired_vel, vehicle_speed, vel_error, planning_param_.replan_vel_deviation);
    initialize_type_ = InitializeType::LARGE_DEVIATION_REPLAN;
    return;
  }

  /* if current vehicle velocity is low && base_desired speed is high, use engage_velocity for engage vehicle */
  const double engage_vel_thr = planning_param_.engage_velocity * planning_param_.engage_exit_ratio;
  if (vehicle_speed < engage_vel_thr && target_vel > planning_param_.engage_velocity) {
    int idx = 0;
    const bool ret = vpu::searchZeroVelocityIdx(reference_traj, idx);
    const bool exist_stop_point = (idx >= reference_traj_closest) ? ret : false;

    const double stop_dist = vpu::calcDist2d(
      reference_traj.points.at(idx), reference_traj.points.at(reference_traj_closest));
    if (!exist_stop_point || stop_dist > planning_param_.stop_dist_to_prohibit_engage) {
      initial_vel = planning_param_.engage_velocity;
      initial_acc = planning_param_.engage_acceleration;
      DEBUG_INFO(
        "[calcInitialMotion] : vehicle speed is low (%3.3f [m/s]), but desired speed is high "
        "(%3.3f [m/s]). "
        "Use engage speed (%3.3f [m/s]) until vehicle speed reaches engage_vel_thr (%3.3f [m/s]), "
        "stop_dist = %3.3f",
        vehicle_speed, target_vel, planning_param_.engage_velocity, engage_vel_thr, stop_dist);
      initialize_type_ = InitializeType::ENGAGING;
      return;
    } else {
      DEBUG_INFO(
        "[calcInitialMotion] : engage condition, but stop point is close (dist = %3.3f). no "
        "engage.",
        stop_dist);
    }
  }

  /* normal update: use closest in prev_output */
  initial_vel = prev_output.points.at(prev_output_closest).twist.linear.x;
  initial_acc = prev_output.points.at(prev_output_closest).accel.linear.x;
  DEBUG_INFO(
    "[calcInitialMotion]: normal update initial_motion.vel = %f, acc = %f, vehicle_speed = %f, "
    "target_vel = %f",
    initial_vel, initial_acc, vehicle_speed, target_vel);
  initialize_type_ = InitializeType::NORMAL;
  return;
}

void MotionVelocityOptimizer::solveOptimization(
  const double initial_vel, const double initial_acc,
  const autoware_planning_msgs::Trajectory & input, const int closest,
  autoware_planning_msgs::Trajectory & output)
{
  auto ts = std::chrono::system_clock::now();

  output = input;

  if ((int)input.points.size() < closest) {
    ROS_WARN("[MotionVelocityOptimizer::solveOptimization] invalid closest.");
    return;
  }

  if (std::fabs(input.points.at(closest).twist.linear.x) < 0.1) {
    DEBUG_INFO(
      "[MotionVelocityOptimizer::solveOptimization] closest vmax < 0.1, keep stopping. return.");
    return;
  }

  const unsigned int N = input.points.size() - closest;

  if (N < 2) {
    return;
  }

  std::vector<double> interval_dist_arr;
  vpu::calcTrajectoryIntervalDistance(input, interval_dist_arr);

  std::vector<double> vmax(N, 0.0);
  for (unsigned int i = 0; i < N; ++i) {
    vmax.at(i) = input.points.at(i + closest).twist.linear.x;
  }

  Eigen::MatrixXd A =
    Eigen::MatrixXd::Zero(3 * N + 1, 4 * N);  // the matrix size depends on constraint numbers.

  std::vector<double> lower_bound(3 * N + 1, 0.0);
  std::vector<double> upper_bound(3 * N + 1, 0.0);

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(4 * N, 4 * N);
  std::vector<double> q(4 * N, 0.0);

  /*
   * x = [b0, b1, ..., bN, |  a0, a1, ..., aN, | delta0, delta1, ..., deltaN, | sigma0, sigme1, ..., sigmaN] in R^{4N}
   * b: velocity^2
   * a: acceleration
   * delta: 0 < bi < vmax^2 + delta
   * sigma: amin < ai - sigma < amax
   */

  const double amax = planning_param_.max_accel;
  const double amin = planning_param_.min_decel;
  const double smooth_weight = qp_param_.pseudo_jerk_weight;
  const double over_v_weight = qp_param_.over_v_weight;
  const double over_a_weight = qp_param_.over_a_weight;

  /* design objective function */
  for (unsigned int i = 0; i < N; ++i) {  // bi
    q[i] = -1.0;                          // |vmax^2 - b| -> minimize (-bi)
  }

  for (unsigned int i = N; i < 2 * N - 1;
       ++i) {  // pseudo jerk: d(ai)/ds -> minimize weight * (a1 - a0)^2 * curr_v^2
    unsigned int j = i - N;
    const double vel = std::max(std::fabs(current_velocity_ptr_->twist.linear.x), 1.0);
    const double w_x_dsinv =
      smooth_weight * (1.0 / std::max(interval_dist_arr.at(j), 0.0001)) * vel * vel;
    P(i, i) += w_x_dsinv;
    P(i, i + 1) -= w_x_dsinv;
    P(i + 1, i) -= w_x_dsinv;
    P(i + 1, i + 1) += w_x_dsinv;
  }

  for (unsigned int i = 2 * N; i < 3 * N; ++i) {  // over velocity cost
    P(i, i) += over_v_weight;
  }

  for (unsigned int i = 3 * N; i < 4 * N; ++i) {  // over acceleration cost
    P(i, i) += over_a_weight;
  }

  /* design constraint matrix */
  // 0 < b - delta < vmax^2
  // NOTE: The delta allows b to be negative. This is actully invalid because the definition is b=v^2.
  // But mathematically, the strict b>0 constraint may make the problem infeasible, such as the case of
  // v=0 & a<0. To avoid the infesibility, we allow b<0. The negative b is dealt as b=0 when it is
  // converted to v with sqrt. If the weight of delta^2 is large (the value of delta is very small),
  // b is almost 0, and is not a big problem.
  for (unsigned int i = 0; i < N; ++i) {
    const int j = 2 * N + i;
    A(i, i) = 1.0;   // b_i
    A(i, j) = -1.0;  // -delta_i
    upper_bound[i] = vmax[i] * vmax[i];
    lower_bound[i] = 0.0;
  }

  // amin < a - sigma < amax
  for (unsigned int i = N; i < 2 * N; ++i) {
    const int j = 2 * N + i;
    A(i, i) = 1.0;   // a_i
    A(i, j) = -1.0;  // -sigma_i
    upper_bound[i] = amax;
    lower_bound[i] = amin;
  }

  // b' = 2a
  for (unsigned int i = 2 * N; i < 3 * N - 1; ++i) {
    const unsigned int j = i - 2 * N;
    const double dsinv = 1.0 / std::max(interval_dist_arr.at(j), 0.0001);
    A(i, j) = -dsinv;
    A(i, j + 1) = dsinv;
    A(i, j + N) = -2.0;
    upper_bound[i] = 0.0;
    lower_bound[i] = 0.0;
  }

  // initial condition
  const double v0 = initial_vel;
  {
    const unsigned int i = 3 * N - 1;
    A(i, 0) = 1.0;  // b0
    upper_bound[i] = v0 * v0;
    lower_bound[i] = v0 * v0;

    A(i + 1, N) = 1.0;  // a0
    upper_bound[i + 1] = initial_acc;
    lower_bound[i + 1] = initial_acc;
  }

  auto tf1 = std::chrono::system_clock::now();
  double elapsed_ms1 =
    std::chrono::duration_cast<std::chrono::nanoseconds>(tf1 - ts).count() * 1.0e-6;

  /* execute optimization */
  auto ts2 = std::chrono::system_clock::now();
  std::tuple<std::vector<double>, std::vector<double>, int> result =
    qp_solver_.optimize(P, A, q, lower_bound, upper_bound);

  // [b0, b1, ..., bN, |  a0, a1, ..., aN, | delta0, delta1, ..., deltaN, | sigma0, sigme1, ..., sigmaN]
  const std::vector<double> optval = std::get<0>(result);

  /* get velocity & acceleration */
  for (int i = 0; i < closest; ++i) {
    double v = optval.at(0);
    output.points.at(i).twist.linear.x = std::sqrt(std::max(v, 0.0));
    output.points.at(i).accel.linear.x = optval.at(N);
  }
  for (unsigned int i = 0; i < N; ++i) {
    double v = optval.at(i);
    output.points.at(i + closest).twist.linear.x = std::sqrt(std::max(v, 0.0));
    output.points.at(i + closest).accel.linear.x = optval.at(i + N);
  }
  for (unsigned int i = N + closest; i < output.points.size(); ++i) {
    output.points.at(i).twist.linear.x = 0.0;
    output.points.at(i).accel.linear.x = 0.0;
  }

  DEBUG_INFO_ALL("[after optimize] idx, vel, acc, over_vel, over_acc ");
  for (unsigned int i = 0; i < N; ++i) {
    DEBUG_INFO_ALL(
      "i = %d, v: %f, vmax: %f a: %f, b: %f, delta: %f, sigma: %f\n", i, std::sqrt(optval.at(i)),
      vmax[i], optval.at(i + N), optval.at(i), optval.at(i + 2 * N), optval.at(i + 3 * N));
  }

  auto tf2 = std::chrono::system_clock::now();
  double elapsed_ms2 =
    std::chrono::duration_cast<std::chrono::nanoseconds>(tf2 - ts2).count() * 1.0e-6;
  DEBUG_INFO(
    "[optimization] initialization time = %f [ms], optimization time = %f [ms]", elapsed_ms1,
    elapsed_ms2);
}

void MotionVelocityOptimizer::optimizeVelocity(
  const autoware_planning_msgs::Trajectory & input, const int input_closest,
  const autoware_planning_msgs::Trajectory & prev_output, const int prev_output_closest,
  autoware_planning_msgs::Trajectory & output)
{
  const double target_vel = std::fabs(input.points.at(input_closest).twist.linear.x);

  /* calculate initial motion for planning */
  double initial_vel = 0.0;
  double initial_acc = 0.0;
  calcInitialMotion(
    target_vel, input, input_closest, prev_output, prev_output_closest,
    /* out */ initial_vel, initial_acc);

  autoware_planning_msgs::Trajectory optimized_traj;
  solveOptimization(initial_vel, initial_acc, input, input_closest, /* out */ optimized_traj);

  /* find stop point for stopVelocityFilter */
  int stop_idx_zero_vel = -1;
  bool stop_point_exists = vpu::searchZeroVelocityIdx(input, stop_idx_zero_vel);
  DEBUG_INFO(
    "[replan] : target_vel = %f, stop_idx_zero_vel = %d, input_closest = %d, stop_point_exists = "
    "%d",
    target_vel, stop_idx_zero_vel, input_closest, (int)stop_point_exists);

  /* for the endpoint of the trajectory */
  if (optimized_traj.points.size() > 0) {
    optimized_traj.points.back().twist.linear.x = 0.0;
  }

  /* set output trajectory */
  output = optimized_traj;

  DEBUG_INFO("[optimizeVelocity] : current_replanned.size() = %d", (int)output.points.size());
}

bool MotionVelocityOptimizer::lateralAccelerationFilter(
  const autoware_planning_msgs::Trajectory & input,
  autoware_planning_msgs::Trajectory & output) const
{
  if (input.points.size() == 0) {
    return false;
  }

  output = input;  // initialize

  if (input.points.size() < 3) {
    return true;  // cannot calculate lateral acc. do nothing.
  }

  /* Interpolate with constant interval distance for lateral acceleration calculation. */
  constexpr double points_interval = 0.1;  // [m]
  std::vector<double> in_arclength, out_arclength;
  vpu::calcTrajectoryArclength(input, in_arclength);
  for (double s = 0; s < in_arclength.back(); s += points_interval) {
    out_arclength.push_back(s);
  }
  if (!vpu::linearInterpTrajectory(in_arclength, input, out_arclength, output)) {
    ROS_WARN(
      "[motion_velocity_optimizer]: fail trajectory interpolation at lateral acceleraion filter.");
    return false;
  }
  output.points.back().twist = input.points.back().twist;  // keep the final speed.

  constexpr double curvature_calc_dist = 3.0;  // [m] calc curvature with 3m away points
  const unsigned int idx_dist = std::max((int)(curvature_calc_dist / points_interval), 1);

  /* Calculate curvature assuming the trajectory points interval is constant */
  std::vector<double> curvature_v;
  vpu::calcTrajectoryCurvatureFrom3Points(output, idx_dist, curvature_v);

  /*  Decrease speed according to lateral G */
  const int before_decel_index =
    static_cast<int>(std::round(planning_param_.decel_distance_before_curve / points_interval));
  const int after_decel_index =
    static_cast<int>(std::round(planning_param_.decel_distance_after_curve / points_interval));
  const double max_lateral_accel_abs = std::fabs(planning_param_.max_lateral_accel);

  const int output_size = static_cast<int>(output.points.size());
  for (int i = 0; i < output_size; ++i) {
    double curvature = 0.0;
    const int start = std::max(i - after_decel_index, 0);
    const int end = std::min(output_size, i + before_decel_index);
    for (int j = start; j < end; ++j) {
      curvature = std::max(curvature, std::fabs(curvature_v.at(j)));
    }
    const double v_curvature_max = std::max(
      std::sqrt(max_lateral_accel_abs / std::max(curvature, 1.0E-5)),
      planning_param_.min_curve_velocity);
    if (output.points.at(i).twist.linear.x > v_curvature_max) {
      output.points.at(i).twist.linear.x = v_curvature_max;
    }
  }
  return true;
}

bool MotionVelocityOptimizer::externalVelocityLimitFilter(
  const autoware_planning_msgs::Trajectory & input,
  autoware_planning_msgs::Trajectory & output) const
{
  output = input;
  if (!external_velocity_limit_acc_limited_ptr_) return false;

  vpu::maximumVelocityFilter(*external_velocity_limit_acc_limited_ptr_, output);
  DEBUG_INFO(
    "[External Velocity Limit] : limit_vel = %3.3f", *external_velocity_limit_acc_limited_ptr_);
  return true;
}

void MotionVelocityOptimizer::preventMoveToCloseStopLine(
  const int closest, autoware_planning_msgs::Trajectory & trajectory) const
{
  if (std::fabs(current_velocity_ptr_->twist.linear.x) < 0.1) {
    int stop_idx = 0;
    bool stop_point_exist = vpu::searchZeroVelocityIdx(trajectory, stop_idx);
    double dist_to_stopline = -1.0;
    if (stop_point_exist && stop_idx >= closest /* desired stop line is ahead of ego-vehicle */) {
      dist_to_stopline =
        vpu::calcDist2d(trajectory.points.at(stop_idx), trajectory.points.at(closest));
      if (dist_to_stopline < planning_param_.stop_dist_to_prohibit_engage) {
        vpu::setZeroVelocity(trajectory);
        DEBUG_INFO(
          "[preventMoveToVeryCloseStopLine] vehicle velocity is low, and stop point is very close. "
          "keep stopping."
          " curr_vel = %3.3f, dist_to_stopline = %3.3f < move_dist_min = %3.3f, stop_idx = %d, "
          "closest = %d",
          current_velocity_ptr_->twist.linear.x, dist_to_stopline,
          planning_param_.stop_dist_to_prohibit_engage, stop_idx, closest);
        return;
      }
    }
    DEBUG_INFO(
      "[preventMoveToVeryCloseStopLine] vehicle velocity is low, and stop point is far away. move."
      " curr_vel = %3.3f, dist_to_stopline = %3.3f < move_dist_min = %3.3f, stop_idx = %d, closest "
      "= %d",
      current_velocity_ptr_->twist.linear.x, dist_to_stopline,
      planning_param_.stop_dist_to_prohibit_engage, stop_idx, closest);
  }
}

bool MotionVelocityOptimizer::extractPathAroundIndex(
  const autoware_planning_msgs::Trajectory & input, const int index,
  autoware_planning_msgs::Trajectory & output) const
{
  const double ahead_length = planning_param_.extract_ahead_dist;
  const double behind_length = planning_param_.extract_behind_dist;

  if (input.points.size() == 0 || index < 0 || (int)input.points.size() - 1 < index) {
    ROS_WARN(
      "extractPathAroundIndex failed. input.points.size() = %lu, base_index = %d",
      input.points.size(), index);
    return false;
  }

  double dist_sum_tmp = 0.0;

  // calc ahead distance
  int ahead_index = input.points.size() - 1;
  for (int i = index; i < (int)input.points.size() - 1; ++i) {
    dist_sum_tmp += vpu::calcDist2d(input.points.at(i), input.points.at(i + 1));
    if (dist_sum_tmp > ahead_length) {
      ahead_index = i + 1;
      break;
    }
  }

  // calc behind distance
  dist_sum_tmp = 0.0;
  int behind_index = 0;
  for (int i = index; i > 0; --i) {
    dist_sum_tmp += vpu::calcDist2d(input.points.at(i), input.points[i - 1]);
    if (dist_sum_tmp > behind_length) {
      behind_index = i - 1;
      break;
    }
  }

  // extruct trajectory
  output.points.clear();
  for (int i = behind_index; i < ahead_index + 1; ++i) {
    output.points.push_back(input.points.at(i));
  }
  output.header = input.header;

  DEBUG_INFO(
    "[extractPathAroundIndex] : input.size() = %lu, extract_base_index = %d, output.size() = %lu",
    input.points.size(), index, output.points.size());
  return true;
}

void MotionVelocityOptimizer::publishClosestVelocity(const double & vel) const
{
  std_msgs::Float32 closest_velocity;
  closest_velocity.data = vel;
  debug_closest_velocity_.publish(closest_velocity);
}

void MotionVelocityOptimizer::updateExternalVelocityLimit(const double dt)
{
  if (!external_velocity_limit_ptr_) return;

  if (!external_velocity_limit_acc_limited_ptr_) {
    external_velocity_limit_acc_limited_ptr_ =
      boost::make_shared<double>(external_velocity_limit_ptr_->data);
    return;
  }

  double dv_raw = (external_velocity_limit_ptr_->data - *external_velocity_limit_acc_limited_ptr_);
  double dv_filtered =
    std::max(std::min(dv_raw, planning_param_.max_accel * dt), planning_param_.min_decel * dt);
  *external_velocity_limit_acc_limited_ptr_ += dv_filtered;
}

void MotionVelocityOptimizer::timerCallback(const ros::TimerEvent & e)
{
  const double dt = (e.current_expected - e.last_expected).toSec();
  updateExternalVelocityLimit(dt);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "motion_velocity_optimizer");
  MotionVelocityOptimizer obj;

  ros::spin();

  return 0;
}
