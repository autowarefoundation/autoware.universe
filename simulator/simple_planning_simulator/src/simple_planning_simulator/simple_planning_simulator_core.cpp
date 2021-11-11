// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <chrono>
#include <algorithm>

#include "simple_planning_simulator/simple_planning_simulator_core.hpp"

#include "common/types.hpp"
#include "autoware_auto_tf2/tf2_autoware_auto_msgs.hpp"
#include "simple_planning_simulator/vehicle_model/sim_model.hpp"
#include "motion_common/motion_common.hpp"

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace
{
autoware_auto_vehicle_msgs::msg::VehicleKinematicState convert_baselink_to_com(
  const autoware_auto_vehicle_msgs::msg::VehicleKinematicState & in, const float32_t baselink_to_com)
{
  autoware_auto_vehicle_msgs::msg::VehicleKinematicState out = in;

  // TODO(Horibe) convert to CoM for vehicle_kinematic_state msg.
  const auto yaw = motion::motion_common::to_angle(out.state.pose.orientation);
  out.state.pose.position.x += static_cast<float64_t>(std::cos(yaw) * baselink_to_com);
  out.state.pose.position.y += static_cast<float64_t>(std::sin(yaw) * baselink_to_com);

  return out;
}

autoware_auto_vehicle_msgs::msg::VehicleKinematicState to_kinematic_state(
  const std::shared_ptr<SimModelInterface> vehicle_model_ptr)
{
  autoware_auto_vehicle_msgs::msg::VehicleKinematicState s;
  s.state.pose.position.x = vehicle_model_ptr->getX();
  s.state.pose.position.y = vehicle_model_ptr->getY();
  s.state.pose.orientation = motion::motion_common::from_angle(vehicle_model_ptr->getYaw());
  s.state.longitudinal_velocity_mps =
    static_cast<float32_t>(vehicle_model_ptr->getVx());
  s.state.lateral_velocity_mps = 0.0;
  s.state.acceleration_mps2 = static_cast<float32_t>(vehicle_model_ptr->getAx());
  s.state.heading_rate_rps = static_cast<float32_t>(vehicle_model_ptr->getWz());
  s.state.front_wheel_angle_rad =
    static_cast<float32_t>(vehicle_model_ptr->getSteer());
  s.state.rear_wheel_angle_rad = 0.0;
  return s;
}
}  // namespace

namespace simulation
{
namespace simple_planning_simulator
{

SimplePlanningSimulator::SimplePlanningSimulator(const rclcpp::NodeOptions & options)
: Node("simple_planning_simulator", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false)
{
  simulated_frame_id_ = declare_parameter("simulated_frame_id", "base_link");
  origin_frame_id_ = declare_parameter("origin_frame_id", "odom");
  add_measurement_noise_ = declare_parameter("add_measurement_noise", false);
  cg_to_rear_m_ = static_cast<float>(declare_parameter("vehicle.cg_to_rear_m", 1.5));

  using rclcpp::QoS;
  using std::placeholders::_1;

  sub_init_pose_ = create_subscription<PoseWithCovarianceStamped>(
    "/initialpose", QoS{1},
    std::bind(&SimplePlanningSimulator::on_initialpose, this, _1));
  sub_vehicle_cmd_ = create_subscription<VehicleControlCommand>(
    "input/vehicle_control_command", QoS{1},
    std::bind(&SimplePlanningSimulator::on_vehicle_cmd, this, _1));
  sub_ackermann_cmd_ = create_subscription<AckermannControlCommand>(
    "input/ackermann_control_command", QoS{1},
    std::bind(&SimplePlanningSimulator::on_ackermann_cmd, this, _1));
  sub_state_cmd_ = create_subscription<VehicleStateCommand>(
    "input/vehicle_state_command", QoS{1},
    std::bind(&SimplePlanningSimulator::on_state_cmd, this, _1));

  pub_state_report_ = create_publisher<VehicleStateReport>("output/vehicle_state_report", QoS{1});
  pub_current_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", QoS{1});
  pub_kinematic_state_ = create_publisher<VehicleKinematicState>("output/kinematic_state", QoS{1});
  pub_tf_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", QoS{1});

  timer_sampling_time_ms_ = static_cast<uint32_t>(declare_parameter("timer_sampling_time_ms", 25));
  on_timer_ = create_wall_timer(
    std::chrono::milliseconds(timer_sampling_time_ms_),
    std::bind(&SimplePlanningSimulator::on_timer, this));


  // set vehicle model type
  initialize_vehicle_model();

  // set initialize source
  const auto initialize_source = declare_parameter("initialize_source", "INITIAL_POSE_TOPIC");
  RCLCPP_INFO(this->get_logger(), "initialize_source : %s", initialize_source.c_str());
  if (initialize_source == "ORIGIN") {
    geometry_msgs::msg::Pose p;
    p.orientation.w = 1.0;                                 // yaw = 0
    set_initial_state(p, geometry_msgs::msg::Twist{});     // initialize with 0 for all variables
  } else if (initialize_source == "INITIAL_POSE_TOPIC") {
    // initialpose sub already exists. Do nothing.
  }


  // measurement noise
  {
    std::random_device seed;
    auto & m = measurement_noise_;
    m.rand_engine_ = std::make_shared<std::mt19937>(seed());
    float64_t pos_noise_stddev = declare_parameter("pos_noise_stddev", 1e-2);
    float64_t vel_noise_stddev = declare_parameter("vel_noise_stddev", 1e-2);
    float64_t rpy_noise_stddev = declare_parameter("rpy_noise_stddev", 1e-4);
    float64_t steer_noise_stddev = declare_parameter("steer_noise_stddev", 1e-4);
    m.pos_dist_ = std::make_shared<std::normal_distribution<>>(0.0, pos_noise_stddev);
    m.vel_dist_ = std::make_shared<std::normal_distribution<>>(0.0, vel_noise_stddev);
    m.rpy_dist_ = std::make_shared<std::normal_distribution<>>(0.0, rpy_noise_stddev);
    m.steer_dist_ = std::make_shared<std::normal_distribution<>>(0.0, steer_noise_stddev);
  }
}

void SimplePlanningSimulator::initialize_vehicle_model()
{
  const auto vehicle_model_type_str = declare_parameter("vehicle_model_type", "IDEAL_STEER_VEL");

  RCLCPP_INFO(this->get_logger(), "vehicle_model_type = %s", vehicle_model_type_str.c_str());

  const float64_t cg_to_front_m = declare_parameter("vehicle.cg_to_front_m", 1.5);
  const float64_t wheelbase = cg_to_front_m + static_cast<float64_t>(cg_to_rear_m_);
  const float64_t vel_lim = declare_parameter("vel_lim", 50.0);
  const float64_t vel_rate_lim = declare_parameter("vel_rate_lim", 7.0);
  const float64_t steer_lim = declare_parameter("steer_lim", 1.0);
  const float64_t steer_rate_lim = declare_parameter("steer_rate_lim", 5.0);
  const float64_t acc_time_delay = declare_parameter("acc_time_delay", 0.1);
  const float64_t acc_time_constant = declare_parameter("acc_time_constant", 0.1);
  const float64_t steer_time_delay = declare_parameter("steer_time_delay", 0.24);
  const float64_t steer_time_constant = declare_parameter("steer_time_constant", 0.27);

  if (vehicle_model_type_str == "IDEAL_STEER_VEL") {
    vehicle_model_type_ = VehicleModelType::IDEAL_STEER_VEL;
    vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerVel>(wheelbase);
  } else if (vehicle_model_type_str == "IDEAL_STEER_ACC") {
    vehicle_model_type_ = VehicleModelType::IDEAL_STEER_ACC;
    vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerAcc>(wheelbase);
  } else if (vehicle_model_type_str == "IDEAL_STEER_ACC_GEARED") {
    vehicle_model_type_ = VehicleModelType::IDEAL_STEER_ACC_GEARED;
    vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerAccGeared>(wheelbase);
  } else if (vehicle_model_type_str == "DELAY_STEER_ACC") {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER_ACC;
    vehicle_model_ptr_ = std::make_shared<SimModelDelaySteerAcc>(
      vel_lim, steer_lim, vel_rate_lim,
      steer_rate_lim, wheelbase,
      timer_sampling_time_ms_ / 1000.0, acc_time_delay, acc_time_constant, steer_time_delay,
      steer_time_constant);
  } else if (vehicle_model_type_str == "DELAY_STEER_ACC_GEARED") {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER_ACC_GEARED;
    vehicle_model_ptr_ = std::make_shared<SimModelDelaySteerAccGeared>(
      vel_lim, steer_lim, vel_rate_lim,
      steer_rate_lim, wheelbase,
      timer_sampling_time_ms_ / 1000.0, acc_time_delay, acc_time_constant, steer_time_delay,
      steer_time_constant);
  } else {
    throw std::invalid_argument("Invalid vehicle_model_type: " + vehicle_model_type_str);
  }
}

void SimplePlanningSimulator::on_timer()
{
  if (!is_initialized_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting initialization...");
    return;
  }

  // update vehicle dynamics
  {
    const float64_t dt = delta_time_.get_dt(get_clock()->now());
    vehicle_model_ptr_->update(dt);
  }

  // set current kinematic state
  current_kinematic_state_ = to_kinematic_state(vehicle_model_ptr_);

  if (add_measurement_noise_) {
    add_measurement_noise(current_kinematic_state_);
  }

  // publish vehicle state
  publish_kinematic_state(convert_baselink_to_com(current_kinematic_state_, cg_to_rear_m_));
  publish_state_report();
  publish_tf(current_kinematic_state_);
}

void SimplePlanningSimulator::on_initialpose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  // save initial pose
  geometry_msgs::msg::Twist initial_twist;
  geometry_msgs::msg::PoseStamped initial_pose;
  initial_pose.header = msg->header;
  initial_pose.pose = msg->pose.pose;
  set_initial_state_with_transform(initial_pose, initial_twist);
}

void SimplePlanningSimulator::on_vehicle_cmd(
  const autoware_auto_vehicle_msgs::msg::VehicleControlCommand::ConstSharedPtr msg)
{
  current_vehicle_cmd_ptr_ = msg;
  set_input(msg->front_wheel_angle_rad, msg->velocity_mps, msg->long_accel_mps2);
}

void SimplePlanningSimulator::on_ackermann_cmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  current_ackermann_cmd_ptr_ = msg;
  set_input(
    msg->lateral.steering_tire_angle, msg->longitudinal.speed,
    msg->longitudinal.acceleration);
}

void SimplePlanningSimulator::set_input(const float steer, const float vel, const float accel)
{
  Eigen::VectorXd input(vehicle_model_ptr_->getDimU());

  if (vehicle_model_type_ == VehicleModelType::IDEAL_STEER_VEL) {
    input << vel, steer;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_ACC ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC)
  {
    input << accel, steer;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_ACC_GEARED ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC_GEARED)
  {
    input << accel, steer;
  }
  vehicle_model_ptr_->setInput(input);
}

void SimplePlanningSimulator::on_state_cmd(
  const autoware_auto_vehicle_msgs::msg::VehicleStateCommand::ConstSharedPtr msg)
{
  current_vehicle_state_cmd_ptr_ = msg;

  if (vehicle_model_type_ == VehicleModelType::IDEAL_STEER_ACC_GEARED ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC_GEARED)
  {
    vehicle_model_ptr_->setGear(current_vehicle_state_cmd_ptr_->gear);
  }
}

void SimplePlanningSimulator::add_measurement_noise(VehicleKinematicState & state) const
{
  auto & n = measurement_noise_;
  state.state.pose.position.x += (*n.pos_dist_)(*n.rand_engine_);
  state.state.pose.position.y += (*n.pos_dist_)(*n.rand_engine_);
  state.state.longitudinal_velocity_mps += static_cast<float32_t>((*n.vel_dist_)(*n.rand_engine_));
  state.state.front_wheel_angle_rad += static_cast<float32_t>((*n.steer_dist_)(*n.rand_engine_));

  float32_t yaw = motion::motion_common::to_angle(state.state.pose.orientation);
  yaw += static_cast<float>((*n.rpy_dist_)(*n.rand_engine_));
  state.state.pose.orientation = motion::motion_common::from_angle(yaw);
}


void SimplePlanningSimulator::set_initial_state_with_transform(
  const geometry_msgs::msg::PoseStamped & pose_stamped, const geometry_msgs::msg::Twist & twist)
{
  auto transform = get_transform_msg(origin_frame_id_, pose_stamped.header.frame_id);
  geometry_msgs::msg::Pose pose;
  pose.position.x = pose_stamped.pose.position.x + transform.transform.translation.x;
  pose.position.y = pose_stamped.pose.position.y + transform.transform.translation.y;
  pose.position.z = pose_stamped.pose.position.z + transform.transform.translation.z;
  pose.orientation = pose_stamped.pose.orientation;
  set_initial_state(pose, twist);
}

void SimplePlanningSimulator::set_initial_state(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & twist)
{
  const float64_t x = pose.position.x;
  const float64_t y = pose.position.y;
  const float64_t yaw = ::motion::motion_common::to_angle(pose.orientation);
  const float64_t vx = twist.linear.x;
  const float64_t steer = 0.0;
  const float64_t accx = 0.0;

  Eigen::VectorXd state(vehicle_model_ptr_->getDimX());

  if (vehicle_model_type_ == VehicleModelType::IDEAL_STEER_VEL) {
    state << x, y, yaw;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_ACC ||
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_ACC_GEARED)
  {
    state << x, y, yaw, vx;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC_GEARED)
  {
    state << x, y, yaw, vx, steer, accx;
  }
  vehicle_model_ptr_->setState(state);

  is_initialized_ = true;
}

geometry_msgs::msg::TransformStamped SimplePlanningSimulator::get_transform_msg(
  const std::string parent_frame, const std::string child_frame)
{
  geometry_msgs::msg::TransformStamped transform;
  while (true) {
    try {
      const auto time_point = tf2::TimePoint(std::chrono::milliseconds(0));
      transform = tf_buffer_.lookupTransform(
        parent_frame, child_frame, time_point, tf2::durationFromSec(
          0.0));
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
  }
  return transform;
}

void SimplePlanningSimulator::publish_kinematic_state(
  const VehicleKinematicState & state)
{
  VehicleKinematicState msg = state;
  msg.header.frame_id = origin_frame_id_;
  msg.header.stamp = get_clock()->now();

  pub_kinematic_state_->publish(msg);
}

void SimplePlanningSimulator::publish_state_report()
{
  VehicleStateReport msg;
  msg.stamp = get_clock()->now();
  msg.mode = VehicleStateReport::MODE_AUTONOMOUS;
  if (current_vehicle_state_cmd_ptr_) {
    msg.gear = current_vehicle_state_cmd_ptr_->gear;
  }
  pub_state_report_->publish(msg);
}

void SimplePlanningSimulator::publish_tf(const VehicleKinematicState & state)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = get_clock()->now();
  tf.header.frame_id = origin_frame_id_;
  tf.child_frame_id = simulated_frame_id_;
  tf.transform.translation.x = state.state.pose.position.x;
  tf.transform.translation.y = state.state.pose.position.y;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation = state.state.pose.orientation;

  tf2_msgs::msg::TFMessage tf_msg{};
  tf_msg.transforms.emplace_back(std::move(tf));
  pub_tf_->publish(tf_msg);
}
}  // namespace simple_planning_simulator
}  // namespace simulation

RCLCPP_COMPONENTS_REGISTER_NODE(simulation::simple_planning_simulator::SimplePlanningSimulator)
