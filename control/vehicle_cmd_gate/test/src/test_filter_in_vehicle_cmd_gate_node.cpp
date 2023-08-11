// Copyright 2021 Tier IV, Inc.
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

#include "../../src/vehicle_cmd_gate.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#define PRINT_LINE() std::cout << "File: " << __FILE__ << ", Line: " << __LINE__ << std::endl;

// global params
const std::vector<double> reference_speed_points = {5., 10., 15., 20.};
const std::vector<double> lon_acc_lim = {1.5, 1.0, 0.8, 0.6};
const std::vector<double> lon_jerk_lim = {1.4, 0.9, 0.7, 0.5};
const std::vector<double> lat_acc_lim = {2.0, 1.6, 1.2, 0.8};
const std::vector<double> lat_jerk_lim = {1.7, 1.3, 0.9, 0.6};
const std::vector<double> actual_steer_diff_lim = {0.5, 0.4, 0.2, 0.1};
const double wheelbase = 2.89;

using vehicle_cmd_gate::VehicleCmdGate;

using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using tier4_control_msgs::msg::GateMode;
using tier4_external_api_msgs::msg::Emergency;
using tier4_external_api_msgs::msg::Heartbeat;
using EngageMsg = autoware_auto_vehicle_msgs::msg::Engage;

class PubSubNode : public rclcpp::Node
{
public:
  PubSubNode() : Node{"test_vehicle_cmd_gate_filter_pubsub"}
  {
    sub_cmd_ = create_subscription<AckermannControlCommand>(
      "output/control_cmd", rclcpp::QoS{1},
      [this](const AckermannControlCommand::ConstSharedPtr msg) {
        cmd_history_.push_back(msg);
        cmd_received_times_.push_back(now());
      });

    rclcpp::QoS qos{1};
    qos.transient_local();

    pub_external_emergency_stop_heartbeat_ =
      create_publisher<Heartbeat>("input/external_emergency_stop_heartbeat", qos);
    pub_engage_ = create_publisher<EngageMsg>("input/engage", qos);
    pub_gate_mode_ = create_publisher<GateMode>("input/gate_mode", qos);
    pub_odom_ = create_publisher<Odometry>("/localization/kinematic_state", qos);
    pub_acc_ = create_publisher<AccelWithCovarianceStamped>("input/acceleration", qos);
    pub_steer_ = create_publisher<SteeringReport>("input/steering", qos);
    pub_operation_mode_ = create_publisher<OperationModeState>("input/operation_mode", qos);
    pub_mrm_state_ = create_publisher<MrmState>("input/mrm_state", qos);

    pub_auto_control_cmd_ =
      create_publisher<AckermannControlCommand>("input/auto/control_cmd", qos);
    pub_auto_turn_indicator_cmd_ =
      create_publisher<TurnIndicatorsCommand>("input/auto/turn_indicators_cmd", qos);
    pub_auto_hazard_light_cmd_ =
      create_publisher<HazardLightsCommand>("input/auto/hazard_lights_cmd", qos);
    pub_auto_gear_cmd_ = create_publisher<GearCommand>("input/auto/gear_cmd", qos);
  }

  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_cmd_;

  rclcpp::Publisher<Heartbeat>::SharedPtr pub_external_emergency_stop_heartbeat_;
  rclcpp::Publisher<EngageMsg>::SharedPtr pub_engage_;
  rclcpp::Publisher<GateMode>::SharedPtr pub_gate_mode_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr pub_acc_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steer_;
  rclcpp::Publisher<OperationModeState>::SharedPtr pub_operation_mode_;
  rclcpp::Publisher<MrmState>::SharedPtr pub_mrm_state_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_auto_control_cmd_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr pub_auto_turn_indicator_cmd_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr pub_auto_hazard_light_cmd_;
  rclcpp::Publisher<GearCommand>::SharedPtr pub_auto_gear_cmd_;

  std::vector<AckermannControlCommand::ConstSharedPtr> cmd_history_;
  std::vector<rclcpp::Time> cmd_received_times_;

  // publish except for the control_cmd
  void publishDefaultTopicsNoSpin(const AckermannControlCommand & control_cmd)
  {
    {
      Heartbeat msg;
      msg.stamp = now();
      pub_external_emergency_stop_heartbeat_->publish(msg);
    }
    {
      EngageMsg msg;
      msg.stamp = now();
      msg.engage = true;
      pub_engage_->publish(msg);
    }
    {
      GateMode msg;
      msg.data = GateMode::AUTO;
      pub_gate_mode_->publish(msg);
    }
    {
      Odometry msg;  // initialized for zero pose and twist
      msg.header.frame_id = "baselink";
      msg.header.stamp = now();
      msg.pose.pose.orientation.w = 1.0;
      msg.twist.twist.linear.x = control_cmd.longitudinal.speed;  // ego moves as commanded.
      pub_odom_->publish(msg);
    }
    {
      AccelWithCovarianceStamped msg;
      msg.header.frame_id = "baselink";
      msg.header.stamp = now();
      msg.accel.accel.linear.x = control_cmd.longitudinal.acceleration;  // ego moves as commanded.
      pub_acc_->publish(msg);
    }
    {
      SteeringReport msg;
      msg.stamp = now();
      msg.steering_tire_angle = control_cmd.lateral.steering_tire_angle;  // ego moves as commanded.
      pub_steer_->publish(msg);
    }
    {
      OperationModeState msg;
      msg.stamp = now();
      msg.mode = OperationModeState::AUTONOMOUS;
      pub_operation_mode_->publish(msg);
    }
    {
      MrmState msg;
      msg.stamp = now();
      msg.state = MrmState::NORMAL;
      msg.behavior = MrmState::NONE;
      pub_mrm_state_->publish(msg);
    }
    {
      TurnIndicatorsCommand msg;
      msg.stamp = now();
      msg.command = TurnIndicatorsCommand::DISABLE;
      pub_auto_turn_indicator_cmd_->publish(msg);
    }
    {
      HazardLightsCommand msg;
      msg.stamp = now();
      msg.command = HazardLightsCommand::DISABLE;
      pub_auto_hazard_light_cmd_->publish(msg);
    }
    {
      GearCommand msg;
      msg.stamp = now();
      msg.command = GearCommand::DRIVE;
      pub_auto_gear_cmd_->publish(msg);
    }
  }

  void publishControlCommand(AckermannControlCommand msg)
  {
    msg.stamp = now();
    pub_auto_control_cmd_->publish(msg);
  }
};

struct CmdParam
{
  double max;
  double freq;
  double bias;
  CmdParam() {}
  CmdParam(double m, double f, double b) : max(m), freq(f), bias(b) {}
};

struct CmdParams
{
  CmdParam steering;
  CmdParam velocity;
  CmdParam acceleration;
  CmdParams() {}
  CmdParams(CmdParam s, CmdParam v, CmdParam a) : steering(s), velocity(v), acceleration(a) {}
};

class ControlCmdGenerator
{
public:
  ControlCmdGenerator() {}

  // used for sin wave command generation
  CmdParams p_;

  using Clock = std::chrono::high_resolution_clock;
  std::chrono::time_point<Clock> start_time_{Clock::now()};

  // generate ControlCommand with sin wave format.
  // TODO(Horibe): implement steering_rate and jerk command.
  AckermannControlCommand calcSinWaveCommand(bool reset_clock = false)
  {
    if (reset_clock) {
      start_time_ = Clock::now();
    }

    const auto dt_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start_time_);
    const auto dt_s = dt_ns.count() / 1e9;

    const auto sinWave = [&](auto amp, auto freq, auto bias) {
      return amp * std::sin(2.0 * M_PI * freq * dt_s + bias);
    };

    AckermannControlCommand cmd;
    cmd.lateral.steering_tire_angle = sinWave(p_.steering.max, p_.steering.freq, p_.steering.bias);
    cmd.longitudinal.speed = sinWave(p_.velocity.max, p_.velocity.freq, p_.velocity.bias);
    cmd.longitudinal.acceleration =
      sinWave(p_.acceleration.max, p_.acceleration.freq, p_.acceleration.bias);

    return cmd;
  }
};

std::shared_ptr<VehicleCmdGate> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};

  const auto vehicle_cmd_gate_dir =
    ament_index_cpp::get_package_share_directory("vehicle_cmd_gate");
  const auto vehicle_info_util_dir =
    ament_index_cpp::get_package_share_directory("vehicle_info_util");

  node_options.arguments(
    {"--ros-args", "--params-file", vehicle_cmd_gate_dir + "/config/vehicle_cmd_gate.param.yaml",
     "--ros-args", "--params-file", vehicle_info_util_dir + "/config/vehicle_info.param.yaml"});

  const auto override = [&](const auto s, const std::vector<double> v) {
    node_options.append_parameter_override<std::vector<double>>(s, v);
  };

  node_options.append_parameter_override("wheel_base", wheelbase);
  override("nominal.reference_speed_points", reference_speed_points);
  override("nominal.reference_speed_points", reference_speed_points);
  override("nominal.lon_acc_lim", lon_acc_lim);
  override("nominal.lon_jerk_lim", lon_jerk_lim);
  override("nominal.lat_acc_lim", lat_acc_lim);
  override("nominal.lat_jerk_lim", lat_jerk_lim);
  override("nominal.actual_steer_diff_lim", actual_steer_diff_lim);

  return std::make_shared<VehicleCmdGate>(node_options);
}

void checkFilter(
  const std::vector<AckermannControlCommand::ConstSharedPtr> & cmd_history,
  const std::vector<rclcpp::Time> & received_times)
{
  if (cmd_history.size() != received_times.size()) {
    throw std::logic_error("cmd history and received times must have same size. Check code.");
  }

  const auto max_lon_acc_lim = *std::max_element(lon_acc_lim.begin(), lon_acc_lim.end());
  const auto max_lon_jerk_lim = *std::max_element(lon_jerk_lim.begin(), lon_jerk_lim.end());
  const auto max_lat_acc_lim = *std::max_element(lat_acc_lim.begin(), lat_acc_lim.end());
  const auto max_lat_jerk_lim = *std::max_element(lat_jerk_lim.begin(), lat_jerk_lim.end());

  const auto v0 = cmd_history.front()->longitudinal.speed;
  auto prev_lat_acc =
    v0 * v0 * std::tan(cmd_history.front()->lateral.steering_tire_angle) / wheelbase;
  for (size_t i = 1; i < cmd_history.size(); ++i) {
    const auto dt = (received_times.at(i) - received_times.at(i - 1)).seconds();
    const auto lon_vel = cmd_history.at(i)->longitudinal.speed;
    const auto lon_acc = cmd_history.at(i)->longitudinal.acceleration;
    const auto lon_jerk = (lon_acc - cmd_history.at(i - 1)->longitudinal.acceleration) / dt;
    const auto lat_acc =
      lon_vel * lon_vel * std::tan(cmd_history.at(i)->lateral.steering_tire_angle) / wheelbase;
    const auto lat_jerk = (lat_acc - prev_lat_acc) / dt;

    // output command must be smaller than maximum limit.
    // TODO(Horibe): check for each velocity range.
    if (std::abs(lon_vel) > 0.01) {
      EXPECT_LT(lon_acc, max_lon_acc_lim);
      EXPECT_LT(lon_jerk, max_lon_jerk_lim)
        << "curr_acc = " << lon_acc
        << ", prev_acc = " << cmd_history.at(i - 1)->longitudinal.acceleration << ", dt = " << dt;
      EXPECT_LT(lat_acc, max_lat_acc_lim)
        << "lon_vel = " << lon_vel
        << ", steer_angle = " << cmd_history.at(i)->lateral.steering_tire_angle
        << ", wheelbase = " << wheelbase;
      EXPECT_LT(lat_jerk, max_lat_jerk_lim)
        << "lat_acc = " << lat_acc << ", prev_lat_acc = " << prev_lat_acc << ", dt = " << dt;
    }
    prev_lat_acc = lat_acc;
  }
}
class TestFixture : public ::testing::TestWithParam<CmdParams>
{
protected:
  void SetUp() override
  {
    // rclcpp::init(0, nullptr);
    vehicle_cmd_gate_node_ = generateNode();
    cmd_generator_.p_ = GetParam();
  }

  void TearDown() override
  {
    // rclcpp::shutdown();
  }

  PubSubNode pub_sub_node_;
  std::shared_ptr<VehicleCmdGate> vehicle_cmd_gate_node_;
  ControlCmdGenerator cmd_generator_;
};

TEST_P(TestFixture, CheckFilterForSinCmd)
{
  [[maybe_unused]] auto a = std::system("ros2 node list");
  [[maybe_unused]] auto b = std::system("ros2 node info /test_vehicle_cmd_gate_filter_pubsub");
  [[maybe_unused]] auto c = std::system("ros2 node info /vehicle_cmd_gate");

  for (size_t i = 0; i < 100; ++i) {
    const bool reset_clock = (i == 0);
    const auto cmd = cmd_generator_.calcSinWaveCommand(reset_clock);
    pub_sub_node_.publishControlCommand(cmd);
    pub_sub_node_.publishDefaultTopicsNoSpin(cmd);
    for (int i = 0; i < 20; ++i) {
      rclcpp::spin_some(pub_sub_node_.get_node_base_interface());
      rclcpp::spin_some(vehicle_cmd_gate_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds{1LL});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
  }

  std::cerr << "cmd_received_times_.size() = " << pub_sub_node_.cmd_received_times_.size()
            << std::endl;
  checkFilter(pub_sub_node_.cmd_history_, pub_sub_node_.cmd_received_times_);
};

CmdParams p1 = {/*steer*/ {10, 1, 0}, /*velocity*/ {10, 1.2, 1}, /*acc*/ {5, 1.5, 2}};
INSTANTIATE_TEST_SUITE_P(TestParam1, TestFixture, ::testing::Values(p1));
