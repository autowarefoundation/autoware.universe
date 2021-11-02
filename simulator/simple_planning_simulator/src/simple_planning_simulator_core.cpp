// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include "simple_planning_simulator/simple_planning_simulator_core.hpp"

#include <autoware_utils/ros/update_param.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

Simulator::Simulator(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  /* simple_planning_simulator parameters */
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  wheelbase_ = vehicle_info.wheel_base_m;
  loop_rate_ = declare_parameter("loop_rate", 30.0);
  sim_steering_gear_ratio_ = declare_parameter("sim_steering_gear_ratio", 15.0);
  simulation_frame_id_ = declare_parameter("simulation_frame_id", "base_link");
  map_frame_id_ = declare_parameter("map_frame_id", "map");
  add_measurement_noise_ = declare_parameter("add_measurement_noise", false);
  use_trajectory_for_z_position_source_ =
    declare_parameter("use_trajectory_for_z_position_source", true);

  /* service */
  autoware_api_utils::ServiceProxyNodeInterface proxy(this);
  group_api_service_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_set_pose_ = proxy.create_service<autoware_external_api_msgs::srv::InitializePose>(
    "/api/simulator/set/pose", std::bind(&Simulator::serviceSetPose, this, _1, _2),
    rmw_qos_profile_services_default, group_api_service_);

  /* set pub sub topic name */
  pub_pose_ =
    create_publisher<geometry_msgs::msg::PoseStamped>("output/current_pose", rclcpp::QoS{1});
  pub_twist_ =
    create_publisher<geometry_msgs::msg::TwistStamped>("output/current_twist", rclcpp::QoS{1});
  pub_control_mode_ = create_publisher<autoware_vehicle_msgs::msg::ControlMode>(
    "output/control_mode", rclcpp::QoS{1});
  pub_steer_ = create_publisher<autoware_vehicle_msgs::msg::Steering>(
    "/vehicle/status/steering", rclcpp::QoS{1});
  pub_velocity_ = create_publisher<autoware_debug_msgs::msg::Float32Stamped>(
    "/vehicle/status/velocity", rclcpp::QoS{1});
  pub_turn_signal_ = create_publisher<autoware_vehicle_msgs::msg::TurnSignal>(
    "/vehicle/status/turn_signal", rclcpp::QoS{1});
  pub_shift_ = create_publisher<autoware_vehicle_msgs::msg::ShiftStamped>(
    "/vehicle/status/shift", rclcpp::QoS{1});
  pub_cov_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("output/cov", rclcpp::QoS{1});

  sub_vehicle_cmd_ = create_subscription<autoware_vehicle_msgs::msg::VehicleCommand>(
    "input/vehicle_cmd", rclcpp::QoS{1},
    std::bind(&Simulator::callbackVehicleCmd, this, std::placeholders::_1));

  sub_turn_signal_cmd_ = create_subscription<autoware_vehicle_msgs::msg::TurnSignal>(
    "input/turn_signal_cmd", rclcpp::QoS{1},
    [this](const autoware_vehicle_msgs::msg::TurnSignal::SharedPtr msg) {
      callbackTurnSignalCmd(msg);
    });
  sub_initialtwist_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/initial_twist", rclcpp::QoS{1},
    std::bind(&Simulator::callbackInitialTwistStamped, this, std::placeholders::_1));

  sub_engage_ = create_subscription<autoware_vehicle_msgs::msg::Engage>(
    "input/engage", rclcpp::QoS{1},
    std::bind(&Simulator::callbackEngage, this, std::placeholders::_1));
  sub_initialpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "input/initial_pose", rclcpp::QoS{1},
    std::bind(&Simulator::callbackInitialPoseWithCov, this, std::placeholders::_1));

  if (use_trajectory_for_z_position_source_) {
    sub_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
      "base_trajectory", rclcpp::QoS{1},
      std::bind(&Simulator::callbackTrajectory, this, std::placeholders::_1));
  }
  const double dt = 1.0 / loop_rate_;

  /* set param callback */
  set_param_res_ =
    this->add_on_set_parameters_callback(std::bind(&Simulator::onParameter, this, _1));

  /* Timer */
  {
    auto timer_callback = std::bind(&Simulator::timerCallbackSimulation, this);
    auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt));
    timer_simulation_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
      this->get_clock(), period, std::move(timer_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_simulation_, nullptr);
  }

  /* tf setting */
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
      std::shared_ptr<rclcpp::Node>(this, [](auto) {}));
  }

  /* set vehicle model parameters */
  {
    auto vehicle_model_type_str = declare_parameter("vehicle_model_type", "IDEAL_STEER");
    RCLCPP_INFO(get_logger(), "vehicle_model_type = %s", vehicle_model_type_str.c_str());
    auto vel_lim = declare_parameter("vel_lim", 50.0);
    auto steer_lim = declare_parameter("steer_lim", 1.0);
    auto accel_rate = declare_parameter("accel_rate", 10.0);
    auto steer_rate_lim = declare_parameter("steer_rate_lim", 5.0);
    auto vel_time_delay = declare_parameter("vel_time_delay", 0.25);
    auto vel_time_constant = declare_parameter("vel_time_constant", 0.5);
    auto steer_time_delay = declare_parameter("steer_time_delay", 0.3);
    auto steer_time_constant = declare_parameter("steer_time_constant", 0.3);
    auto acc_time_delay = declare_parameter("acc_time_delay", 0.1);
    auto acc_time_constant = declare_parameter("acc_time_constant", 0.1);
    simulator_engage_ = declare_parameter("initial_engage_state", true);
    auto deadzone_delta_steer = declare_parameter("deadzone_delta_steer", 0.0);

    if (vehicle_model_type_str == "IDEAL_STEER") {
      vehicle_model_type_ = VehicleModelType::IDEAL_STEER;
      vehicle_model_ptr_ = std::make_shared<SimModelIdealSteer>(wheelbase_);
    } else if (vehicle_model_type_str == "IDEAL_ACCEL") {
      vehicle_model_type_ = VehicleModelType::IDEAL_ACCEL;
      vehicle_model_ptr_ = std::make_shared<SimModelIdealAccel>(wheelbase_);
    } else if (vehicle_model_type_str == "DELAY_STEER") {
      vehicle_model_type_ = VehicleModelType::DELAY_STEER;
      vehicle_model_ptr_ = std::make_shared<SimModelTimeDelaySteer>(
        vel_lim, steer_lim, accel_rate, steer_rate_lim, wheelbase_, dt, vel_time_delay,
        vel_time_constant, steer_time_delay, steer_time_constant, deadzone_delta_steer);
    } else if (vehicle_model_type_str == "DELAY_STEER_ACC") {
      vehicle_model_type_ = VehicleModelType::DELAY_STEER_ACC;
      vehicle_model_ptr_ = std::make_shared<SimModelTimeDelaySteerAccel>(
        vel_lim, steer_lim, accel_rate, steer_rate_lim, wheelbase_, dt, acc_time_delay,
        acc_time_constant, steer_time_delay, steer_time_constant, deadzone_delta_steer);
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid vehicle_model_type. Initialization failed.");
    }
  }

  /* set normal distribution noises */
  {
    int random_seed = declare_parameter("random_seed", 1);
    if (random_seed >= 0) {
      rand_engine_ptr_ = std::make_shared<std::mt19937>(random_seed);
    } else {
      std::random_device seed;
      rand_engine_ptr_ = std::make_shared<std::mt19937>(seed());
    }
    auto pos_noise_stddev = declare_parameter("pos_noise_stddev", 0.01);
    auto vel_noise_stddev = declare_parameter("vel_noise_stddev", 0.0);
    auto rpy_noise_stddev = declare_parameter("rpy_noise_stddev", 0.0001);
    auto angvel_noise_stddev = declare_parameter("angvel_noise_stddev", 0.0);
    auto steer_noise_stddev = declare_parameter("steer_noise_stddev", 0.0001);
    pos_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, pos_noise_stddev);
    vel_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, vel_noise_stddev);
    rpy_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, rpy_noise_stddev);
    angvel_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, angvel_noise_stddev);
    steer_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, steer_noise_stddev);

    x_stddev_ = declare_parameter("x_stddev", 0.0001);
    y_stddev_ = declare_parameter("y_stddev", 0.0001);
  }

  current_pose_.orientation.w = 1.0;

  closest_pos_z_ = 0.0;
}

rcl_interfaces::msg::SetParametersResult Simulator::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    autoware_utils::updateParam(parameters, "x_stddev", x_stddev_);
    autoware_utils::updateParam(parameters, "y_stddev", y_stddev_);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

void Simulator::callbackTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  current_trajectory_ptr_ = msg;
}
void Simulator::callbackInitialPoseWithCov(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  geometry_msgs::msg::Twist initial_twist;  // initialized with zero for all components
  if (initial_twist_ptr_) {
    initial_twist = initial_twist_ptr_->twist;
  }
  // save initial pose
  initial_pose_with_cov_ptr_ = msg;
  setInitialStateWithPoseTransform(*initial_pose_with_cov_ptr_, initial_twist);
}

void Simulator::callbackInitialPoseStamped(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  geometry_msgs::msg::Twist initial_twist;  // initialized with zero for all components
  if (initial_twist_ptr_) {
    initial_twist = initial_twist_ptr_->twist;
  }
  // save initial pose
  initial_pose_ptr_ = msg;
  setInitialStateWithPoseTransform(*initial_pose_ptr_, initial_twist);
}

void Simulator::callbackInitialTwistStamped(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  // save initial pose
  initial_twist_ptr_ = msg;
  if (initial_pose_ptr_) {
    setInitialStateWithPoseTransform(*initial_pose_ptr_, initial_twist_ptr_->twist);
    // input twist to simulator's internal parameter
    current_pose_ = initial_pose_ptr_->pose;
    current_twist_ = initial_twist_ptr_->twist;
  } else if (initial_pose_with_cov_ptr_) {
    setInitialStateWithPoseTransform(*initial_pose_with_cov_ptr_, initial_twist_ptr_->twist);
  }
}

void Simulator::callbackEngage(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
{
  simulator_engage_ = msg->engage;
}

void Simulator::serviceSetPose(
  const autoware_external_api_msgs::srv::InitializePose::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::InitializePose::Response::SharedPtr response)
{
  geometry_msgs::msg::Twist initial_twist;  // initialized with zero for all components
  if (initial_twist_ptr_) {
    initial_twist = initial_twist_ptr_->twist;
  }
  // save initial pose
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  initial_pose_with_cov_ptr_ = std::make_shared<PoseWithCovarianceStamped>(request->pose);
  setInitialStateWithPoseTransform(*initial_pose_with_cov_ptr_, initial_twist);
  response->status = autoware_api_utils::response_success();
}

void Simulator::timerCallbackSimulation()
{
  if (!is_initialized_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), 3000 /*ms*/, "waiting initial position...");
    return;
  }

  if (prev_update_time_ptr_ == nullptr) {
    prev_update_time_ptr_ = std::make_shared<rclcpp::Time>(get_clock()->now());
  }

  /* calculate delta time */
  const double dt = (get_clock()->now() - *prev_update_time_ptr_).seconds();
  *prev_update_time_ptr_ = get_clock()->now();

  if (simulator_engage_) {
    /* update vehicle dynamics when simulator_engage_ is true */
    vehicle_model_ptr_->update(dt);
    /* set control mode */
    control_mode_.data = autoware_vehicle_msgs::msg::ControlMode::AUTO;
  } else {
    /* set control mode */
    control_mode_.data = autoware_vehicle_msgs::msg::ControlMode::MANUAL;
  }

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

  /* make current vehicle pose with covariance  */
  geometry_msgs::msg::PoseWithCovariance cov;
  cov.pose = current_pose_;
  cov.covariance[0 * 6 + 0] = x_stddev_;
  cov.covariance[1 * 6 + 1] = y_stddev_;

  if (simulator_engage_ && add_measurement_noise_) {
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

  /* publish twist & covariance & tf */
  publishTwist(current_twist_);
  publishPoseWithCov(cov);
  publishTF(current_pose_);

  /* publish steering */
  autoware_vehicle_msgs::msg::Steering steer_msg;
  steer_msg.header.frame_id = simulation_frame_id_;
  steer_msg.header.stamp = get_clock()->now();
  steer_msg.data = vehicle_model_ptr_->getSteer();
  if (add_measurement_noise_) {
    steer_msg.data += (*steer_norm_dist_ptr_)(*rand_engine_ptr_);
  }
  pub_steer_->publish(steer_msg);

  /* float info publishers */
  autoware_debug_msgs::msg::Float32Stamped velocity_msg;
  velocity_msg.stamp = this->now();
  velocity_msg.data = current_twist_.linear.x;
  pub_velocity_->publish(velocity_msg);

  autoware_vehicle_msgs::msg::TurnSignal turn_signal_msg;
  turn_signal_msg.header.frame_id = simulation_frame_id_;
  turn_signal_msg.header.stamp = get_clock()->now();
  turn_signal_msg.data = autoware_vehicle_msgs::msg::TurnSignal::NONE;
  if (current_turn_signal_cmd_ptr_) {
    const auto cmd = current_turn_signal_cmd_ptr_->data;
    // ignore invalid data such as cmd=999
    if (
      cmd == autoware_vehicle_msgs::msg::TurnSignal::LEFT ||
      cmd == autoware_vehicle_msgs::msg::TurnSignal::RIGHT ||
      cmd == autoware_vehicle_msgs::msg::TurnSignal::HAZARD) {
      turn_signal_msg.data = cmd;
    }
  }
  pub_turn_signal_->publish(turn_signal_msg);

  autoware_vehicle_msgs::msg::ShiftStamped shift_msg;
  shift_msg.header.frame_id = simulation_frame_id_;
  shift_msg.header.stamp = get_clock()->now();
  shift_msg.shift.data = current_twist_.linear.x >= 0.0
                           ? autoware_vehicle_msgs::msg::Shift::DRIVE
                           : autoware_vehicle_msgs::msg::Shift::REVERSE;
  pub_shift_->publish(shift_msg);

  /* publish control mode */
  pub_control_mode_->publish(control_mode_);
}

void Simulator::callbackVehicleCmd(
  const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr msg)
{
  current_vehicle_cmd_ptr_ = msg;

  if (
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER) {
    Eigen::VectorXd input(2);
    input << msg->control.velocity, msg->control.steering_angle;
    vehicle_model_ptr_->setInput(input);
  } else if (vehicle_model_type_ == VehicleModelType::IDEAL_ACCEL) {
    Eigen::VectorXd input(2);
    input << msg->control.acceleration, msg->control.steering_angle;
    vehicle_model_ptr_->setInput(input);
  } else if (vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC) {
    Eigen::VectorXd input(3);
    double drive_shift =
      (msg->shift.data == autoware_vehicle_msgs::msg::Shift::REVERSE) ? -1.0 : 1.0;
    input << msg->control.acceleration, msg->control.steering_angle, drive_shift;
    vehicle_model_ptr_->setInput(input);
  } else {
    RCLCPP_WARN(get_logger(), "[%s] : invalid vehicle_model_type_  error.", __func__);
  }
}
void Simulator::callbackTurnSignalCmd(
  const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg)
{
  current_turn_signal_cmd_ptr_ = msg;
}

void Simulator::setInitialStateWithPoseTransform(
  const geometry_msgs::msg::PoseStamped & pose_stamped, const geometry_msgs::msg::Twist & twist)
{
  geometry_msgs::msg::TransformStamped transform;
  getTransformFromTF(map_frame_id_, pose_stamped.header.frame_id, transform);
  geometry_msgs::msg::Pose pose;
  pose.position.x = pose_stamped.pose.position.x + transform.transform.translation.x;
  pose.position.y = pose_stamped.pose.position.y + transform.transform.translation.y;
  pose.position.z = pose_stamped.pose.position.z + transform.transform.translation.z;
  pose.orientation = pose_stamped.pose.orientation;
  setInitialState(pose, twist);
}

void Simulator::setInitialStateWithPoseTransform(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
  const geometry_msgs::msg::Twist & twist)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header = pose.header;
  ps.pose = pose.pose.pose;
  setInitialStateWithPoseTransform(ps, twist);
}

void Simulator::setInitialState(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & twist)
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
  } else if (vehicle_model_type_ == VehicleModelType::IDEAL_ACCEL) {
    Eigen::VectorXd state(4);
    state << x, y, yaw, vx;
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
    RCLCPP_WARN(get_logger(), "undesired vehicle model type! Initialization failed.");
    return;
  }

  is_initialized_ = true;
}

void Simulator::getTransformFromTF(
  const std::string parent_frame, const std::string child_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  while (rclcpp::ok()) {
    try {
      const auto time_point = tf2::TimePoint(std::chrono::milliseconds(0));
      transform = tf_buffer_->lookupTransform(parent_frame, child_frame, time_point);
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
  }
}

void Simulator::publishTwist(const geometry_msgs::msg::Twist & twist)
{
  rclcpp::Time current_time = get_clock()->now();
  geometry_msgs::msg::TwistStamped ts;
  ts.header.frame_id = simulation_frame_id_;
  ts.header.stamp = current_time;
  ts.twist = twist;
  pub_twist_->publish(ts);
}

void Simulator::publishPoseWithCov(const geometry_msgs::msg::PoseWithCovariance & cov)
{
  rclcpp::Time current_time = get_clock()->now();
  geometry_msgs::msg::PoseWithCovarianceStamped cs;
  cs.header.frame_id = map_frame_id_;
  cs.header.stamp = current_time;
  cs.pose = cov;
  pub_cov_->publish(cs);

  // TODO(kosuke_murakami): remove publishing pose when the v2 scenario sim migration is complete
  geometry_msgs::msg::PoseStamped ps;
  ps.header = cs.header;
  ps.pose = cov.pose;
  pub_pose_->publish(ps);
}

void Simulator::publishTF(const geometry_msgs::msg::Pose & pose)
{
  rclcpp::Time current_time = get_clock()->now();

  // send odom transform
  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = map_frame_id_;
  odom_trans.child_frame_id = simulation_frame_id_;
  odom_trans.transform.translation.x = pose.position.x;
  odom_trans.transform.translation.y = pose.position.y;
  odom_trans.transform.translation.z = pose.position.z;
  odom_trans.transform.rotation = pose.orientation;
  tf_broadcaster_->sendTransform(odom_trans);
}

double Simulator::getPosZFromTrajectory(const double x, const double y)
{
  // calculate closest point on trajectory
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
    if (found) {
      return current_trajectory_ptr_->points.at(index).pose.position.z;
    } else {
      return 0;
    }
  } else {
    return 0.0;
  }
}

geometry_msgs::msg::Quaternion Simulator::getQuaternionFromRPY(
  const double & roll, const double & pitch, const double & yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}
