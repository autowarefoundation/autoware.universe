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

#include "simple_planning_simulator/simple_planning_simulator_core.hpp"

// clang-format off
#define DEBUG_INFO(...) { ROS_INFO(__VA_ARGS__); }

// clang-format on
Simulator::Simulator() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_), is_initialized_(false)
{
  /* simple_planning_simulator parameters */
  pnh_.param("loop_rate", loop_rate_, double(50.0));
  nh_.param("/vehicle_info/wheel_base", wheelbase_, double(2.7));
  pnh_.param("sim_steering_gear_ratio", sim_steering_gear_ratio_, double(15.0));
  pnh_.param("simulation_frame_id", simulation_frame_id_, std::string("base_link"));
  pnh_.param("map_frame_id", map_frame_id_, std::string("map"));
  pnh_.param("add_measurement_noise", add_measurement_noise_, bool(true));

  /* set pub sub topic name */
  pub_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("output/current_pose", 1);
  pub_twist_ = pnh_.advertise<geometry_msgs::TwistStamped>("output/current_twist", 1);
  pub_steer_ = nh_.advertise<autoware_vehicle_msgs::Steering>("/vehicle/status/steering", 1);
  pub_velocity_ = nh_.advertise<std_msgs::Float32>("/vehicle/status/velocity", 1);
  pub_turn_signal_ =
    nh_.advertise<autoware_vehicle_msgs::TurnSignal>("/vehicle/status/turn_signal", 1);
  pub_shift_ = nh_.advertise<autoware_vehicle_msgs::ShiftStamped>("/vehicle/status/shift", 1);
  sub_vehicle_cmd_ = pnh_.subscribe("input/vehicle_cmd", 1, &Simulator::callbackVehicleCmd, this);
  timer_simulation_ =
    nh_.createTimer(ros::Duration(1.0 / loop_rate_), &Simulator::timerCallbackSimulation, this);

  bool use_trajectory_for_z_position_source;
  pnh_.param(
    "use_trajectory_for_z_position_source", use_trajectory_for_z_position_source, bool(true));
  if (use_trajectory_for_z_position_source) {
    sub_trajectory_ = nh_.subscribe("base_trajectory", 1, &Simulator::callbackTrajectory, this);
  }

  /* set vehicle model parameters */
  double tread_length, angvel_lim, vel_lim, steer_lim, accel_rate, angvel_rate, steer_rate_lim,
    vel_time_delay, acc_time_delay, vel_time_constant, steer_time_delay, steer_time_constant,
    angvel_time_delay, angvel_time_constant, acc_time_constant;
  pnh_.param("tread_length", tread_length, double(1.0));
  pnh_.param("angvel_lim", angvel_lim, double(3.0));
  pnh_.param("vel_lim", vel_lim, double(10.0));
  pnh_.param("steer_lim", steer_lim, double(3.14 / 3.0));
  pnh_.param("accel_rate", accel_rate, double(1.0));
  pnh_.param("angvel_rate", angvel_rate, double(1.0));
  pnh_.param("steer_rate_lim", steer_rate_lim, double(0.3));
  pnh_.param("vel_time_delay", vel_time_delay, double(0.25));
  pnh_.param("vel_time_constant", vel_time_constant, double(0.6197));
  pnh_.param("steer_time_delay", steer_time_delay, double(0.24));
  pnh_.param("steer_time_constant", steer_time_constant, double(0.27));
  pnh_.param("angvel_time_delay", angvel_time_delay, double(0.2));
  pnh_.param("angvel_time_constant", angvel_time_constant, double(0.5));
  pnh_.param("acc_time_delay", acc_time_delay, double(0.3));
  pnh_.param("acc_time_constant", acc_time_constant, double(0.3));
  const double dt = 1.0 / loop_rate_;

  /* set vehicle model type */
  std::string vehicle_model_type_str;
  pnh_.param("vehicle_model_type", vehicle_model_type_str, std::string("IDEAL_TWIST"));
  ROS_INFO("vehicle_model_type = %s", vehicle_model_type_str.c_str());
  if (vehicle_model_type_str == "IDEAL_STEER") {
    vehicle_model_type_ = VehicleModelType::IDEAL_STEER;
    vehicle_model_ptr_ = std::make_shared<SimModelIdealSteer>(wheelbase_);
  } else if (vehicle_model_type_str == "DELAY_STEER") {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER;
    vehicle_model_ptr_ = std::make_shared<SimModelTimeDelaySteer>(
      vel_lim, steer_lim, accel_rate, steer_rate_lim, wheelbase_, dt, vel_time_delay,
      vel_time_constant, steer_time_delay, steer_time_constant);
  } else if (vehicle_model_type_str == "DELAY_STEER_ACC") {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER_ACC;
    vehicle_model_ptr_ = std::make_shared<SimModelTimeDelaySteerAccel>(
      vel_lim, steer_lim, accel_rate, steer_rate_lim, wheelbase_, dt, acc_time_delay,
      acc_time_constant, steer_time_delay, steer_time_constant);
  } else {
    ROS_ERROR("Invalid vehicle_model_type. Initialization failed.");
  }

  /* set normal distribution noises */
  int random_seed;
  pnh_.param("random_seed", random_seed, -1);
  if (random_seed >= 0) {
    rand_engine_ptr_ = std::make_shared<std::mt19937>(random_seed);
  } else {
    std::random_device seed;
    rand_engine_ptr_ = std::make_shared<std::mt19937>(seed());
  }
  double pos_noise_stddev, vel_noise_stddev, rpy_noise_stddev, angvel_noise_stddev,
    steer_noise_stddev;
  pnh_.param("pos_noise_stddev", pos_noise_stddev, 1e-2);
  pnh_.param("vel_noise_stddev", vel_noise_stddev, 1e-2);
  pnh_.param("rpy_noise_stddev", rpy_noise_stddev, 1e-4);
  pnh_.param("angvel_noise_stddev", angvel_noise_stddev, 1e-3);
  pnh_.param("steer_noise_stddev", steer_noise_stddev, 1e-4);
  pos_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, pos_noise_stddev);
  vel_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, vel_noise_stddev);
  rpy_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, rpy_noise_stddev);
  angvel_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, angvel_noise_stddev);
  steer_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, steer_noise_stddev);

  /* set initialize source */
  std::string initialize_source;
  pnh_.param("initialize_source", initialize_source, std::string("ORIGIN"));
  ROS_INFO_STREAM("initialize_source : " << initialize_source);
  if (initialize_source == "RVIZ") {
    sub_initialpose_ =
      nh_.subscribe("initialpose", 1, &Simulator::callbackInitialPoseWithCov, this);
  } else if (initialize_source == "NDT") {
    sub_initialpose_ = nh_.subscribe("ndt_pose", 1, &Simulator::callbackInitialPoseStamped, this);
  } else if (initialize_source == "GNSS") {
    sub_initialpose_ = nh_.subscribe("gnss_pose", 1, &Simulator::callbackInitialPoseStamped, this);
  } else if (initialize_source == "ORIGIN") {
    geometry_msgs::Pose p;
    p.orientation.w = 1.0;  // yaw = 0
    geometry_msgs::Twist t;
    setInitialState(p, t);  // initialize with 0 for all variables
  } else {
    ROS_WARN("initialize_source is undesired, setting error!!");
  }
  current_pose_.orientation.w = 1.0;

  closest_pos_z_ = 0.0;
}

void Simulator::callbackTrajectory(const autoware_planning_msgs::TrajectoryConstPtr & msg)
{
  current_trajectory_ptr_ = std::make_shared<autoware_planning_msgs::Trajectory>(*msg);
}
void Simulator::callbackInitialPoseWithCov(
  const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
{
  geometry_msgs::Twist initial_twist;  // initialized with zero for all components
  setInitialStateWithPoseTransform(*msg, initial_twist);
}

void Simulator::callbackInitialPoseStamped(const geometry_msgs::PoseStampedConstPtr & msg)
{
  geometry_msgs::Twist initial_twist;  // initialized with zero for all components
  setInitialStateWithPoseTransform(*msg, initial_twist);
}

void Simulator::timerCallbackSimulation(const ros::TimerEvent & e)
{
  if (!is_initialized_) {
    ROS_INFO_DELAYED_THROTTLE(3.0, "[simple_planning_simulator] waiting initial position...");
    return;
  }

  if (prev_update_time_ptr_ == nullptr) {
    prev_update_time_ptr_ = std::make_shared<ros::Time>(ros::Time::now());
  }

  /* calculate delta time */
  const double dt = (ros::Time::now() - *prev_update_time_ptr_).toSec();
  *prev_update_time_ptr_ = ros::Time::now();

  /* update vehicle dynamics */
  vehicle_model_ptr_->update(dt);

  /* save current vehicle pose & twist */
  current_pose_.position.x = vehicle_model_ptr_->getX();
  current_pose_.position.y = vehicle_model_ptr_->getY();
  closest_pos_z_ = getPosZFromTrajectory(
    current_pose_.position.x,
    current_pose_.position.y);  // update vehicle z position from trajectory
  current_pose_.position.z = closest_pos_z_;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = vehicle_model_ptr_->getYaw();
  current_twist_.linear.x = vehicle_model_ptr_->getVx();
  current_twist_.angular.z = vehicle_model_ptr_->getWz();

  if (add_measurement_noise_) {
    current_pose_.position.x += (*pos_norm_dist_ptr_)(*rand_engine_ptr_);
    current_pose_.position.y += (*pos_norm_dist_ptr_)(*rand_engine_ptr_);
    current_pose_.position.z += (*pos_norm_dist_ptr_)(*rand_engine_ptr_);
    roll += (*rpy_norm_dist_ptr_)(*rand_engine_ptr_);
    pitch += (*rpy_norm_dist_ptr_)(*rand_engine_ptr_);
    yaw += (*rpy_norm_dist_ptr_)(*rand_engine_ptr_);
    if (current_twist_.linear.x >= 0.0) {
      current_twist_.linear.x += (*vel_norm_dist_ptr_)(*rand_engine_ptr_);
    } else {
      current_twist_.linear.x -= (*vel_norm_dist_ptr_)(*rand_engine_ptr_);
    }
    current_twist_.angular.z += (*angvel_norm_dist_ptr_)(*rand_engine_ptr_);
  }

  current_pose_.orientation = getQuaternionFromRPY(roll, pitch, yaw);

  /* publish pose & twist */
  publishPoseTwist(current_pose_, current_twist_);
  publishTF(current_pose_);

  /* publish steering */
  autoware_vehicle_msgs::Steering steer_msg;
  steer_msg.header.frame_id = simulation_frame_id_;
  steer_msg.header.stamp = ros::Time::now();
  steer_msg.data = vehicle_model_ptr_->getSteer();
  if (add_measurement_noise_) {
    steer_msg.data += (*steer_norm_dist_ptr_)(*rand_engine_ptr_);
  }
  pub_steer_.publish(steer_msg);

  /* float info publishers */
  std_msgs::Float32 velocity_msg;
  velocity_msg.data = current_twist_.linear.x;
  pub_velocity_.publish(velocity_msg);

  autoware_vehicle_msgs::TurnSignal turn_signal_msg;
  turn_signal_msg.header.frame_id = simulation_frame_id_;
  turn_signal_msg.header.stamp = ros::Time::now();
  turn_signal_msg.data = autoware_vehicle_msgs::TurnSignal::NONE;
  pub_turn_signal_.publish(turn_signal_msg);

  autoware_vehicle_msgs::ShiftStamped shift_msg;
  shift_msg.header.frame_id = simulation_frame_id_;
  shift_msg.header.stamp = ros::Time::now();
  shift_msg.shift.data = current_twist_.linear.x >= 0.0 ? autoware_vehicle_msgs::Shift::DRIVE
                                                        : autoware_vehicle_msgs::Shift::REVERSE;
  pub_shift_.publish(shift_msg);
}

void Simulator::callbackVehicleCmd(const autoware_vehicle_msgs::VehicleCommandConstPtr & msg)
{
  current_vehicle_cmd_ptr_ = std::make_shared<autoware_vehicle_msgs::VehicleCommand>(*msg);

  if (
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER) {
    Eigen::VectorXd input(2);
    input << msg->control.velocity, msg->control.steering_angle;
    vehicle_model_ptr_->setInput(input);
  } else if (vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC) {
    Eigen::VectorXd input(3);
    double drive_shift = (msg->shift.data == autoware_vehicle_msgs::Shift::DRIVE) ? 1.0 : -1.0;
    input << msg->control.acceleration, msg->control.steering_angle, drive_shift;
    vehicle_model_ptr_->setInput(input);
  } else {
    ROS_WARN("[%s] : invalid vehicle_model_type_  error.", __func__);
  }
}

void Simulator::setInitialStateWithPoseTransform(
  const geometry_msgs::PoseStamped & pose_stamped, const geometry_msgs::Twist & twist)
{
  geometry_msgs::TransformStamped transform;
  getTransformFromTF(map_frame_id_, pose_stamped.header.frame_id, transform);
  geometry_msgs::Pose pose;
  pose.position.x = pose_stamped.pose.position.x + transform.transform.translation.x;
  pose.position.y = pose_stamped.pose.position.y + transform.transform.translation.y;
  pose.position.z = pose_stamped.pose.position.z + transform.transform.translation.z;
  pose.orientation = pose_stamped.pose.orientation;
  setInitialState(pose, twist);
}

void Simulator::setInitialStateWithPoseTransform(
  const geometry_msgs::PoseWithCovarianceStamped & pose, const geometry_msgs::Twist & twist)
{
  geometry_msgs::PoseStamped ps;
  ps.header = pose.header;
  ps.pose = pose.pose.pose;
  setInitialStateWithPoseTransform(ps, twist);
}

void Simulator::setInitialState(
  const geometry_msgs::Pose & pose, const geometry_msgs::Twist & twist)
{
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double yaw = tf2::getYaw(pose.orientation);
  const double vx = twist.linear.x;
  const double steer = 0.0;
  const double acc = 0.0;

  if (vehicle_model_type_ == VehicleModelType::IDEAL_STEER) {
    Eigen::VectorXd state(3);
    state << x, y, yaw;
    vehicle_model_ptr_->setState(state);
  } else if (vehicle_model_type_ == VehicleModelType::DELAY_STEER) {
    Eigen::VectorXd state(5);
    state << x, y, yaw, vx, steer;
    vehicle_model_ptr_->setState(state);
  } else if (vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC) {
    Eigen::VectorXd state(6);
    state << x, y, yaw, vx, steer, acc;
    vehicle_model_ptr_->setState(state);
  } else {
    ROS_WARN("undesired vehicle model type! Initialization failed.");
    return;
  }

  is_initialized_ = true;
}

void Simulator::getTransformFromTF(
  const std::string parent_frame, const std::string child_frame,
  geometry_msgs::TransformStamped & transform)
{
  while (1) {
    try {
      transform = tf_buffer_.lookupTransform(parent_frame, child_frame, ros::Time(0));
      break;
    } catch (tf2::TransformException & ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}

void Simulator::publishPoseTwist(
  const geometry_msgs::Pose & pose, const geometry_msgs::Twist & twist)
{
  ros::Time current_time = ros::Time::now();

  // simulatied pose
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = map_frame_id_;
  ps.header.stamp = current_time;
  ps.pose = pose;
  pub_pose_.publish(ps);

  geometry_msgs::TwistStamped ts;
  ts.header.frame_id = simulation_frame_id_;
  ts.header.stamp = current_time;
  ts.twist = twist;
  pub_twist_.publish(ts);
}

void Simulator::publishTF(const geometry_msgs::Pose & pose)
{
  ros::Time current_time = ros::Time::now();

  // send odom transform
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = map_frame_id_;
  odom_trans.child_frame_id = simulation_frame_id_;
  odom_trans.transform.translation.x = pose.position.x;
  odom_trans.transform.translation.y = pose.position.y;
  odom_trans.transform.translation.z = pose.position.z;
  odom_trans.transform.rotation = pose.orientation;
  tf_broadcaster_.sendTransform(odom_trans);
}

double Simulator::getPosZFromTrajectory(const double x, const double y)
{
  // calculae cloest point on trajectory
  /*
         write me...
  */
  if (current_trajectory_ptr_ != nullptr) {
    const double max_sqrt_dist = 100.0 * 100.0;
    double min_sqrt_dist = max_sqrt_dist;
    int index;
    bool found = false;
    for (size_t i = 0; i < current_trajectory_ptr_->points.size(); ++i) {
      const double dist_x = (current_trajectory_ptr_->points.at(i).pose.position.x - x);
      const double dist_y = (current_trajectory_ptr_->points.at(i).pose.position.y - y);
      double sqrt_dist = dist_x * dist_x + dist_y * dist_y;
      if (sqrt_dist < min_sqrt_dist) {
        min_sqrt_dist = sqrt_dist;
        index = i;
        found = true;
      }
    }
    if (found)
      return current_trajectory_ptr_->points.at(index).pose.position.z;
    else
      return 0;
  } else {
    return 0.0;
  }
}

geometry_msgs::Quaternion Simulator::getQuaternionFromRPY(
  const double & roll, const double & pitch, const double & yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}
