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
#include <string>
#include <vector>
#include <iostream>
#include <cstdlib>

#define PRINT_LINE() std::cout << "File: " << __FILE__ << ", Line: " << __LINE__ << std::endl;

using vehicle_cmd_gate::VehicleCmdGate;

using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using tier4_control_msgs::msg::GateMode;
using tier4_external_api_msgs::msg::Emergency;
using tier4_external_api_msgs::msg::Heartbeat;
using nav_msgs::msg::Odometry;
using EngageMsg = autoware_auto_vehicle_msgs::msg::Engage;

class PubSubNode : public rclcpp::Node
{
public:
  PubSubNode() : Node{"test_vehicle_cmd_gate_filter_pubsub"}
  {
    sub_cmd_ = create_subscription<AckermannControlCommand>(
      "output/control_cmd", rclcpp::QoS{1}, [this](const AckermannControlCommand::SharedPtr msg) {
        current_cmd_ = msg;
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

  AckermannControlCommand::SharedPtr current_cmd_;
  std::vector<rclcpp::Time> cmd_received_times_;

  // publish except for the control_cmd
  void publishDefaultTopicsNoSpin()
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
      pub_odom_->publish(msg);
    }
    {
      AccelWithCovarianceStamped msg;
      msg.header.frame_id = "baselink";
      msg.header.stamp = now();
      msg.accel.accel.linear.x = 1.0;
      pub_acc_->publish(msg);
    }
    {
      SteeringReport msg;
      msg.stamp = now();
      msg.steering_tire_angle = 0.0;
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
  ControlCmdGenerator(){};

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
  // node_options.append_parameter_override("algorithm_type", "JerkFiltered");
  const auto vehicle_cmd_gate_dir =
    ament_index_cpp::get_package_share_directory("vehicle_cmd_gate");
  const auto vehicle_info_util_dir =
    ament_index_cpp::get_package_share_directory("vehicle_info_util");

  node_options.arguments(
    {"--ros-args", "--params-file", vehicle_cmd_gate_dir + "/config/vehicle_cmd_gate.param.yaml",
     "--ros-args", "--params-file", vehicle_info_util_dir + "/config/vehicle_info.param.yaml"});

  return std::make_shared<VehicleCmdGate>(node_options);
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

    void TearDown() override {
      // rclcpp::shutdown();
    }

  PubSubNode pub_sub_node_;
  std::shared_ptr<VehicleCmdGate> vehicle_cmd_gate_node_;
  ControlCmdGenerator cmd_generator_;
};

TEST_P(TestFixture, CheckFilterForSinCmd) {
  [[maybe_unused]] auto a = std::system("ros2 node list");
  [[maybe_unused]] auto b = std::system("ros2 node info /test_vehicle_cmd_gate_filter_pubsub");
  [[maybe_unused]] auto c = std::system("ros2 node info /vehicle_cmd_gate");

  for (size_t i = 0; i < 10; ++i) {
    const bool reset_clock = (i == 0);
    const auto cmd = cmd_generator_.calcSinWaveCommand(reset_clock);
    pub_sub_node_.publishDefaultTopicsNoSpin();
    pub_sub_node_.publishControlCommand(cmd);
    for (int i = 0; i < 20; ++i) {
      rclcpp::spin_some(pub_sub_node_.get_node_base_interface());
      rclcpp::spin_some(vehicle_cmd_gate_node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds{1LL});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
  }

  std::cerr << "cmd_received_times_.size() = " << pub_sub_node_.cmd_received_times_.size() << std::endl;
};

CmdParams p1 = {/*steer*/ {10, 1, 0}, /*velocity*/ {10, 1.2, 1}, /*acc*/ {5, 1.5, 2}};
INSTANTIATE_TEST_SUITE_P(TestParam1, TestFixture, ::testing::Values(p1));
