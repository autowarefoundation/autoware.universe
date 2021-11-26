// Copyright 2020-2021 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#include "lgsvl_interface/lgsvl_interface.hpp"

#include <common/types.hpp>
#include <helper_functions/float_comparisons.hpp>
#include <motion_common/motion_common.hpp>

#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/headlights_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/headlights_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/horn_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/horn_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/wipers_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/wipers_report.hpp"
#include <geometry_msgs/msg/quaternion.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;
using std::chrono_literals::operator""s;
namespace comp = autoware::common::helper_functions::comparisons;

namespace lgsvl_interface
{
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsReport;
using autoware_auto_vehicle_msgs::msg::HeadlightsCommand;
using autoware_auto_vehicle_msgs::msg::HeadlightsReport;
using autoware_auto_vehicle_msgs::msg::HornCommand;
using autoware_auto_vehicle_msgs::msg::HornReport;
using autoware_auto_vehicle_msgs::msg::WipersCommand;
using autoware_auto_vehicle_msgs::msg::WipersReport;

const std::unordered_map<WIPER_TYPE, WIPER_TYPE> LgsvlInterface::autoware_to_lgsvl_wiper{
  {WipersCommand::NO_COMMAND, static_cast<WIPER_TYPE>(VSD::WIPERS_OFF)},
  {WipersCommand::DISABLE, static_cast<WIPER_TYPE>(VSD::WIPERS_OFF)},
  {WipersCommand::ENABLE_LOW, static_cast<WIPER_TYPE>(VSD::WIPERS_LOW)},
  {WipersCommand::ENABLE_HIGH, static_cast<WIPER_TYPE>(VSD::WIPERS_HIGH)},
  {WipersCommand::ENABLE_CLEAN, static_cast<WIPER_TYPE>(VSD::WIPERS_OFF)},
};

const std::unordered_map<GEAR_TYPE, GEAR_TYPE> LgsvlInterface::autoware_to_lgsvl_gear{
  {VSC::GEAR_NO_COMMAND, static_cast<GEAR_TYPE>(VSD::GEAR_NEUTRAL)},
  {VSC::GEAR_DRIVE, static_cast<GEAR_TYPE>(VSD::GEAR_DRIVE)},
  {VSC::GEAR_REVERSE, static_cast<GEAR_TYPE>(VSD::GEAR_REVERSE)},
  {VSC::GEAR_PARK, static_cast<GEAR_TYPE>(VSD::GEAR_PARKING)},
  {VSC::GEAR_LOW, static_cast<GEAR_TYPE>(VSD::GEAR_LOW)},
  {VSC::GEAR_NEUTRAL, static_cast<GEAR_TYPE>(VSD::GEAR_NEUTRAL)},
};

const std::unordered_map<MODE_TYPE, MODE_TYPE> LgsvlInterface::autoware_to_lgsvl_mode{
  {VSC::MODE_NO_COMMAND, static_cast<MODE_TYPE>(VSD::VEHICLE_MODE_COMPLETE_MANUAL)},
  {VSC::MODE_AUTONOMOUS, static_cast<MODE_TYPE>(VSD::VEHICLE_MODE_COMPLETE_AUTO_DRIVE)},
  {VSC::MODE_MANUAL, static_cast<MODE_TYPE>(VSD::VEHICLE_MODE_COMPLETE_MANUAL)},
};

LgsvlInterface::LgsvlInterface(
  rclcpp::Node & node, const std::string & sim_cmd_topic, const std::string & sim_state_cmd_topic,
  const std::string & sim_state_report_topic, const std::string & sim_nav_odom_topic,
  const std::string & sim_veh_odom_topic, const std::string & kinematic_state_topic,
  const std::string & gear_report_topic, const std::string & steer_report_topic,
  const std::string & control_mode_report_topic, const std::string & twist_topic,
  const std::string & odom_topic, const std::string & sim_odom_child_frame,
  Table1D && throttle_table, Table1D && brake_table, Table1D && steer_table, bool publish_tf,
  bool publish_pose)
: m_throttle_table{throttle_table},
  m_brake_table{brake_table},
  m_steer_table{steer_table},
  m_logger{node.get_logger()}
{
  const auto check = [](const auto value, const auto ref) -> bool8_t {
    constexpr auto EPS = std::numeric_limits<decltype(value)>::epsilon();
    return comp::abs_gt(value, ref, EPS);
  };
  // check throttle table
  if (check(m_throttle_table.domain().front(), 0.0)) {
    throw std::domain_error{"Throttle table domain must be [0, ...)"};
  }
  if (
    check(m_throttle_table.range().front(), 0.0) || check(m_throttle_table.range().back(), 100.0)) {
    throw std::domain_error{"Throttle table range must go from 0 to 100"};
  }
  for (const auto val : m_throttle_table.range()) {
    if (val < 0.0) {
      throw std::domain_error{"Throttle table must map to nonnegative accelerations"};
    }
  }
  // Check brake table
  if (check(m_brake_table.domain().back(), 0.0)) {
    throw std::domain_error{"Brake table domain must be [..., 0)"};
  }
  if (check(m_brake_table.range().front(), 100.0) || check(m_brake_table.range().back(), 0.0)) {
    throw std::domain_error{"Brake table must go from 100 to 0"};
  }
  for (const auto val : m_brake_table.domain()) {
    if (val > 0.0) {
      throw std::domain_error{"Brake table must map negative accelerations to 0-100 values"};
    }
  }
  // Check steer table
  if (check(m_steer_table.range().front(), -100.0) || check(m_steer_table.range().back(), 100.0)) {
    throw std::domain_error{"Steer table must go from -100 to 100"};
  }
  if (
    (m_steer_table.domain().front() >= 0.0) ||                              // Should be negative...
    (m_steer_table.domain().back() <= 0.0) ||                               // to positive...
    check(m_steer_table.domain().back(), -m_steer_table.domain().front()))  // with symmetry
  {
    // Warn if steer domain is not equally straddling zero: could be right, but maybe not
    RCLCPP_WARN(node.get_logger(), "Steer table domain is not symmetric across zero. Is this ok?");
  }

  // Make publishers
  m_cmd_pub =
    node.create_publisher<lgsvl_msgs::msg::VehicleControlData>(sim_cmd_topic, rclcpp::QoS{10});
  m_state_pub =
    node.create_publisher<lgsvl_msgs::msg::VehicleStateData>(sim_state_cmd_topic, rclcpp::QoS{10});
  // publishers for Autoware.iv
  m_pub_control_mode_report =
    node.create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
      control_mode_report_topic, rclcpp::QoS{10});
  m_pub_gear_report = node.create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    gear_report_topic, rclcpp::QoS{10});
  m_pub_velocity = node.create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    twist_topic, rclcpp::QoS{10});
  m_pub_steer = node.create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    steer_report_topic, rclcpp::QoS{10});
  m_pub_odom = node.create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS{10});

  // Make subscribers
  if (!sim_nav_odom_topic.empty() && ("null" != sim_nav_odom_topic)) {
    m_nav_odom_sub = node.create_subscription<nav_msgs::msg::Odometry>(
      sim_nav_odom_topic, rclcpp::QoS{10},
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odometry(*msg); });
    // Ground truth state/pose publishers only work if there's a ground truth input
    m_kinematic_state_pub =
      node.create_publisher<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>(
        kinematic_state_topic, rclcpp::QoS{10});

    if (publish_pose) {
      m_pose_pub = node.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/vehicle/odom_pose", rclcpp::QoS{10});
    }

    if (publish_tf) {
      m_tf_pub = node.create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS{10});
    }
  }

  m_state_sub = node.create_subscription<lgsvl_msgs::msg::CanBusData>(
    sim_state_report_topic, rclcpp::QoS{10}, [this](lgsvl_msgs::msg::CanBusData::SharedPtr msg) {
      autoware_auto_vehicle_msgs::msg::VehicleStateReport state_report;
      // state_report.set__fuel(nullptr);  // no fuel status from LGSVL
      if (msg->left_turn_signal_active) {
        state_report.set__blinker(
          autoware_auto_vehicle_msgs::msg::VehicleStateReport::BLINKER_LEFT);
      } else if (msg->right_turn_signal_active) {
        state_report.set__blinker(
          autoware_auto_vehicle_msgs::msg::VehicleStateReport::BLINKER_RIGHT);
      } else {
        state_report.set__blinker(autoware_auto_vehicle_msgs::msg::VehicleStateReport::BLINKER_OFF);
      }

      if (msg->low_beams_active) {
        state_report.set__headlight(
          autoware_auto_vehicle_msgs::msg::VehicleStateReport::HEADLIGHT_ON);
        headlights_report().report = HeadlightsReport::ENABLE_LOW;
      } else if (msg->high_beams_active) {
        state_report.set__headlight(
          autoware_auto_vehicle_msgs::msg::VehicleStateReport::HEADLIGHT_HIGH);
        headlights_report().report = HeadlightsReport::ENABLE_HIGH;
      } else {
        state_report.set__headlight(
          autoware_auto_vehicle_msgs::msg::VehicleStateReport::HEADLIGHT_OFF);
        headlights_report().report = HeadlightsReport::DISABLE;
      }

      if (msg->wipers_active) {
        state_report.set__wiper(autoware_auto_vehicle_msgs::msg::VehicleStateReport::WIPER_LOW);
        wipers_report().report = WipersReport::ENABLE_LOW;
      } else {
        state_report.set__wiper(autoware_auto_vehicle_msgs::msg::VehicleStateReport::WIPER_OFF);
        wipers_report().report = WipersReport::DISABLE;
      }

      state_report.set__gear(static_cast<uint8_t>(msg->selected_gear));
      // state_report.set__mode();  // no mode status from LGSVL
      state_report.set__hand_brake(msg->parking_brake_active);
      // state_report.set__horn()  // no horn status from LGSVL
      on_state_report(state_report);

      // Autoware.iv interface
      {
        using autoware_auto_vehicle_msgs::msg::GearReport;
        GearReport gear_report;
        gear_report.stamp = msg->header.stamp;
        switch (msg->selected_gear) {
          case (lgsvl_msgs::msg::CanBusData::GEAR_DRIVE):
            gear_report.report = GearReport::DRIVE;
            break;
          case (lgsvl_msgs::msg::CanBusData::GEAR_REVERSE):
            gear_report.report = GearReport::REVERSE;
            break;
          case (lgsvl_msgs::msg::CanBusData::GEAR_LOW):
            gear_report.report = GearReport::LOW;
            break;
          case (lgsvl_msgs::msg::CanBusData::GEAR_NEUTRAL):
          case (lgsvl_msgs::msg::CanBusData::GEAR_PARKING):
          default:
            gear_report.report = GearReport::PARK;
            break;
        }
        m_pub_gear_report->publish(gear_report);
      }
      {
        autoware_auto_vehicle_msgs::msg::ControlModeReport mode_report;
        mode_report.stamp = msg->header.stamp;
        mode_report.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
        m_pub_control_mode_report->publish(mode_report);
      }
    });

  m_veh_odom_sub = node.create_subscription<lgsvl_msgs::msg::VehicleOdometry>(
    sim_veh_odom_topic, rclcpp::QoS{10}, [this](lgsvl_msgs::msg::VehicleOdometry::SharedPtr msg) {
      odometry().set__stamp(msg->header.stamp);
      odometry().set__velocity_mps(msg->velocity);
      odometry().set__rear_wheel_angle_rad(msg->rear_wheel_angle);
      odometry().set__front_wheel_angle_rad(msg->front_wheel_angle);
      // Autoware.iv interface
      {
        autoware_auto_vehicle_msgs::msg::SteeringReport steer;
        // Inverse steering angle as SVL uses positive angle for right steer instead of left
        steer.steering_tire_angle = -1 * msg->front_wheel_angle;
        m_pub_steer->publish(steer);
      }
    });

  // Setup Tf Buffer with listener
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(node.get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  // Initialize m_nav_base_in_child_frame with no offset until TF arrives
  m_nav_base_in_child_frame.header.frame_id = sim_odom_child_frame;

  m_nav_base_tf_timer = node.create_wall_timer(1s, [this, sim_odom_child_frame]() {
    if (m_tf_buffer->canTransform(sim_odom_child_frame, "nav_base", tf2::TimePointZero)) {
      // Cancel timer because we were able to find the transform
      m_nav_base_tf_timer->cancel();
      m_nav_base_tf_set = true;

      geometry_msgs::msg::TransformStamped nav_base_tf{};
      nav_base_tf =
        m_tf_buffer->lookupTransform(sim_odom_child_frame, "nav_base", tf2::TimePointZero);

      // Create Vehicle Kinematic State from the transform between nav_base and
      // the odometry child frame from the simulator
      m_nav_base_in_child_frame.state.pose.position.x = nav_base_tf.transform.translation.x;
      m_nav_base_in_child_frame.state.pose.position.y = nav_base_tf.transform.translation.y;
      m_nav_base_in_child_frame.state.pose.orientation = nav_base_tf.transform.rotation;
    } else {
      RCLCPP_ERROR(
        m_logger, "Transform from nav_base to %s is unavailable. Waiting...",
        sim_odom_child_frame.c_str());
    }
  });
}

////////////////////////////////////////////////////////////////////////////////
bool8_t LgsvlInterface::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  // Not implemented: API is not needed since everything is handled by subscription callbacks
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool8_t LgsvlInterface::send_state_command(
  const autoware_auto_vehicle_msgs::msg::VehicleStateCommand & msg)
{
  auto msg_corrected = msg;

  // Correcting blinker: it is shifted down by one,
  // as the first value BLINKER_NO_COMMAND does not exisit in LGSVL
  if (msg.blinker == VSC::BLINKER_NO_COMMAND) {
    msg_corrected.blinker = get_state_report().blinker;
  }
  msg_corrected.blinker--;

  // Correcting gears
  auto const gear_iter = autoware_to_lgsvl_gear.find(msg.gear);

  if (gear_iter != autoware_to_lgsvl_gear.end()) {
    msg_corrected.gear = gear_iter->second;
  } else {
    msg_corrected.gear = static_cast<uint8_t>(VSD::GEAR_DRIVE);
    RCLCPP_WARN(m_logger, "Unsupported gear value in state command, defaulting to Drive");
  }

  // Correcting wipers
  auto const wiper_iter = autoware_to_lgsvl_wiper.find(msg.wiper);

  if (wiper_iter != autoware_to_lgsvl_wiper.end()) {
    msg_corrected.wiper = wiper_iter->second;
  } else {
    msg_corrected.wiper = static_cast<uint8_t>(VSD::WIPERS_OFF);
    RCLCPP_WARN(m_logger, "Unsupported wiper value in state command, defaulting to OFF");
  }

  // Correcting mode
  auto const mode_iter = autoware_to_lgsvl_mode.find(msg.mode);

  if (mode_iter != autoware_to_lgsvl_mode.end()) {
    msg_corrected.mode = mode_iter->second;
  } else {
    msg_corrected.mode = static_cast<uint8_t>(VSD::VEHICLE_MODE_COMPLETE_MANUAL);
    RCLCPP_WARN(m_logger, "Unsupported mode value in state command, defaulting to COMPLETE MANUAL");
  }

  m_lgsvl_state.header.set__stamp(msg_corrected.stamp);
  m_lgsvl_state.set__blinker_state(msg_corrected.blinker);
  m_lgsvl_state.set__current_gear(msg_corrected.gear);
  m_lgsvl_state.set__vehicle_mode(msg_corrected.mode);
  m_lgsvl_state.set__hand_brake_active(msg_corrected.hand_brake);
  m_lgsvl_state.set__autonomous_mode_active(
    msg_corrected.mode == VSD::VEHICLE_MODE_COMPLETE_AUTO_DRIVE ? true : false);

  m_state_pub->publish(m_lgsvl_state);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool8_t LgsvlInterface::send_control_command(
  const autoware_auto_vehicle_msgs::msg::VehicleControlCommand & msg)
{
  autoware_auto_vehicle_msgs::msg::RawControlCommand raw_msg;
  raw_msg.stamp = msg.stamp;
  raw_msg.throttle = 0;
  raw_msg.brake = 0;

  using VSR = autoware_auto_vehicle_msgs::msg::VehicleStateReport;
  const auto directional_accel =
    get_state_report().gear == VSR::GEAR_REVERSE ? -msg.long_accel_mps2 : msg.long_accel_mps2;

  if (directional_accel >= decltype(msg.long_accel_mps2){}) {
    // TODO(c.ho)  cast to double...
    raw_msg.throttle =
      static_cast<decltype(raw_msg.throttle)>(m_throttle_table.lookup(directional_accel));
  } else {
    raw_msg.brake = static_cast<decltype(raw_msg.brake)>(m_brake_table.lookup(directional_accel));
  }
  raw_msg.front_steer =
    static_cast<decltype(raw_msg.front_steer)>(m_steer_table.lookup(msg.front_wheel_angle_rad));
  raw_msg.rear_steer = 0;

  return send_control_command(raw_msg);
}

////////////////////////////////////////////////////////////////////////////////
bool8_t LgsvlInterface::send_control_command(
  const autoware_auto_control_msgs::msg::AckermannControlCommand & msg)
{
  autoware_auto_vehicle_msgs::msg::RawControlCommand raw_msg;
  raw_msg.stamp = msg.stamp;
  raw_msg.throttle = 0;
  raw_msg.brake = 0;

  if (msg.longitudinal.acceleration >= decltype(msg.longitudinal.acceleration){}) {
    raw_msg.throttle = static_cast<decltype(raw_msg.throttle)>(
      m_throttle_table.lookup(msg.longitudinal.acceleration));
  } else {
    raw_msg.brake =
      static_cast<decltype(raw_msg.brake)>(m_brake_table.lookup(msg.longitudinal.acceleration));
  }
  raw_msg.front_steer = static_cast<decltype(raw_msg.front_steer)>(
    m_steer_table.lookup(msg.lateral.steering_tire_angle));
  raw_msg.rear_steer = 0;

  return send_control_command(raw_msg);
}

////////////////////////////////////////////////////////////////////////////////
bool8_t LgsvlInterface::send_control_command(
  const autoware_auto_vehicle_msgs::msg::RawControlCommand & msg)
{
  // Front steer semantically is z up, ccw positive, but LGSVL thinks its the opposite
  lgsvl_msgs::msg::VehicleControlData control_data;
  control_data.set__acceleration_pct(static_cast<float>(msg.throttle) / 100.f);
  control_data.set__braking_pct(static_cast<float>(msg.brake) / 100.f);
  control_data.set__target_wheel_angle(-static_cast<float>(msg.front_steer) / 100.f);
  // control_data.set__target_wheel_angular_rate();  // Missing angular rate in raw command
  // control_data.set__target_gear(); // Missing target gear in raw command
  m_cmd_pub->publish(control_data);
  return true;
}

bool8_t LgsvlInterface::handle_mode_change_request(
  autoware_auto_vehicle_msgs::srv::AutonomyModeChange_Request::SharedPtr request)
{
  // TODO(JWhitleyWork) Actually enable/disable this internally to
  // mimic a real vehicle and allow commanding the vehicle to be disabled
  (void)request;
  return true;
}

void LgsvlInterface::send_headlights_command(
  const autoware_auto_vehicle_msgs::msg::HeadlightsCommand & msg)
{
  /// lgsvl_msgs values are shifted down one from autoware_auto_vehicle_msgs values
  /// However, lgsvl_msgs values have no "NO_COMMAND" option so 0 is ignored
  auto shifted_command = msg.command;

  if (shifted_command > 0U) {
    shifted_command--;
  }

  m_lgsvl_state.set__headlight_state(shifted_command);
}

void LgsvlInterface::send_horn_command(const autoware_auto_vehicle_msgs::msg::HornCommand & msg)
{
  m_lgsvl_state.set__horn_active(msg.active);
}

void LgsvlInterface::send_wipers_command(const autoware_auto_vehicle_msgs::msg::WipersCommand & msg)
{
  /// lgsvl_msgs values are shifted down one from autoware_auto_vehicle_msgs values
  /// However, lgsvl_msgs values have no "NO_COMMAND" option so 0 is ignored
  auto shifted_command = msg.command;

  if (shifted_command > 0U) {
    shifted_command--;
  }

  m_lgsvl_state.set__wiper_state(shifted_command);
}

////////////////////////////////////////////////////////////////////////////////
void LgsvlInterface::on_odometry(const nav_msgs::msg::Odometry & msg)
{
  if (!m_odom_set) {
    m_odom_zero.x = msg.pose.pose.position.x;
    m_odom_zero.y = msg.pose.pose.position.y;
    m_odom_zero.z = msg.pose.pose.position.z;
    m_odom_set = true;
  }

  decltype(msg.pose.pose.orientation) q{msg.pose.pose.orientation};
  // Aligning odom frame at the vehicle position at the start to remove the initial odom->bl offset
  const auto px = msg.pose.pose.position.x - m_odom_zero.x;
  const auto py = msg.pose.pose.position.y - m_odom_zero.y;
  const auto pz = msg.pose.pose.position.z - m_odom_zero.z;
  {
    // Create a TF which represents the odometry observation
    geometry_msgs::msg::TransformStamped tf{};
    tf.header = msg.header;
    tf.child_frame_id = msg.child_frame_id;
    tf.transform.translation.x = px;
    tf.transform.translation.y = py;
    tf.transform.translation.z = pz;
    tf.transform.rotation = q;

    // Only create vehicle kinematic state when required tf is available
    if (m_nav_base_tf_set) {
      autoware_auto_vehicle_msgs::msg::VehicleKinematicState vse_t{};

      // Apply a transform representing the odometry observation of the original
      // child frame in the odometry parent frame to the VehicleKinematicState
      // representing the position of nav_base in the odometry child frame
      // The resulting VehicleKinematicState (vse_t) represents the position of nav_base
      // in the odometry message's parent frame
      motion::motion_common::doTransform(m_nav_base_in_child_frame, vse_t, tf);

      // Set header timestamp to timestamp of odometry message
      vse_t.header.stamp = msg.header.stamp;

      // Get values from vehicle odometry
      vse_t.state.longitudinal_velocity_mps = get_odometry().velocity_mps;
      // Steer semantically is z up, ccw positive, but LGSVL thinks its the opposite
      vse_t.state.front_wheel_angle_rad = -get_odometry().front_wheel_angle_rad;
      vse_t.state.rear_wheel_angle_rad = -get_odometry().rear_wheel_angle_rad;
      if (
        state_report().gear == autoware_auto_vehicle_msgs::msg::VehicleStateReport::GEAR_REVERSE) {
        vse_t.state.longitudinal_velocity_mps *= -1.0f;
      }

      vse_t.state.lateral_velocity_mps =
        static_cast<decltype(vse_t.state.lateral_velocity_mps)>(msg.twist.twist.linear.y);
      // TODO(jitrc): populate with correct value when acceleration is available from simulator
      vse_t.state.acceleration_mps2 = 0.0F;
      vse_t.state.heading_rate_rps =
        static_cast<decltype(vse_t.state.heading_rate_rps)>(msg.twist.twist.angular.z);

      m_kinematic_state_pub->publish(vse_t);
    }

    if (m_tf_pub) {
      tf2_msgs::msg::TFMessage tf_msg{};
      tf_msg.transforms.emplace_back(std::move(tf));
      m_tf_pub->publish(tf_msg);
    }
  }

  if (m_pose_pub) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose{};
    pose.header = msg.header;
    pose.pose.pose.orientation = q;
    pose.pose.pose.position.x = px;
    pose.pose.pose.position.y = py;
    pose.pose.pose.position.z = pz;

    constexpr auto EPS = std::numeric_limits<float64_t>::epsilon();
    if (
      std::fabs(msg.pose.covariance[COV_X]) > EPS || std::fabs(msg.pose.covariance[COV_Y]) > EPS ||
      std::fabs(msg.pose.covariance[COV_Z]) > EPS || std::fabs(msg.pose.covariance[COV_RX]) > EPS ||
      std::fabs(msg.pose.covariance[COV_RY]) > EPS ||
      std::fabs(msg.pose.covariance[COV_RZ]) > EPS) {
      pose.pose.covariance = {
        COV_X_VAR,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, COV_Y_VAR,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        COV_Z_VAR,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, COV_RX_VAR, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        COV_RY_VAR, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, COV_RZ_VAR};
    } else {
      pose.pose.covariance = msg.pose.covariance;
    }

    m_pose_pub->publish(pose);
  }
  // Autoware.iv interface
  {
    m_pub_odom->publish(msg);
    autoware_auto_vehicle_msgs::msg::VelocityReport velocity;
    velocity.longitudinal_velocity = static_cast<float>(msg.twist.twist.linear.x);
    velocity.lateral_velocity = 0.0F;
    velocity.heading_rate = static_cast<float>(msg.twist.twist.angular.z);
    m_pub_velocity->publish(velocity);
  }
}

////////////////////////////////////////////////////////////////////////////////
void LgsvlInterface::on_state_report(
  const autoware_auto_vehicle_msgs::msg::VehicleStateReport & msg)
{
  auto corrected_report = msg;

  // in autoware_auto_vehicle_msgs::msg::VehicleStateCommand 1 is drive, 2 is reverse,
  // https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/9744f6dc/src/messages/autoware_auto_vehicle_msgs/msg/VehicleStateCommand.msg#L32
  // in lgsvl 0 is drive and 1 is reverse
  // https://github.com/lgsvl/simulator/blob/cb937deb8e633573f6c0cc76c9f451398b8b9eff/Assets/Scripts/Sensors/VehicleStateSensor.cs#L70

  // Find autoware gear via inverse mapping
  const auto value_same = [&msg](const auto & kv) -> bool {  // also do some capture
    return msg.gear == kv.second;
  };
  const auto it =
    std::find_if(autoware_to_lgsvl_gear.begin(), autoware_to_lgsvl_gear.end(), value_same);

  if (it != autoware_to_lgsvl_gear.end()) {
    corrected_report.gear = it->first;
  } else {
    corrected_report.gear = msg.GEAR_NEUTRAL;
    RCLCPP_WARN(m_logger, "Invalid gear value in state report from LGSVL simulator");
  }

  // Correcting blinker value, they are shifted up by one,
  // as the first value BLINKER_NO_COMMAND does not exisit in LGSVL
  // not setting  VSC::BLINKER_NO_COMMAND, when get.state_report.blinker == msg.blinker
  // instead reporting true blinker status
  corrected_report.blinker++;

  state_report() = corrected_report;
}

}  // namespace lgsvl_interface
