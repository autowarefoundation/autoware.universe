// Copyright 2022 The Autoware Foundation
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

#include "dummy_node.hpp"
#include "fake_test_node/fake_test_node.hpp"
#include "gtest/gtest.h"

#include <memory>
#include <sstream>
#include <string>
#include <vector>

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::HandBrakeCommand;
using autoware_auto_vehicle_msgs::msg::HandBrakeReport;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsReport;
using autoware_auto_vehicle_msgs::msg::HeadlightsCommand;
using autoware_auto_vehicle_msgs::msg::HeadlightsReport;
using autoware_auto_vehicle_msgs::msg::HornCommand;
using autoware_auto_vehicle_msgs::msg::HornReport;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;
using autoware_auto_vehicle_msgs::msg::WipersCommand;
using autoware_auto_vehicle_msgs::msg::WipersReport;
using VehicleOdometry = nav_msgs::msg::Odometry;
using autoware_auto_vehicle_msgs::srv::AutonomyModeChange;
class TestBaseInterfacePublisher : public rclcpp::Node
{
public:
  TestBaseInterfacePublisher() : Node("test_pub", rclcpp::NodeOptions{}) {}
};

TEST(TestBaseInterface, SmokeTest)
{
  rclcpp::init(0, nullptr);
  auto test_node_1 = std::make_shared<autoware::vehicle::interface ::DummyExceptionNode>();
  auto test_node_2 = std::make_shared<autoware::vehicle::interface ::DummyInterfaceNode>();
  rclcpp::spin_some(test_node_1);
  rclcpp::spin_some(test_node_2);
  const auto wait_time{std::chrono::milliseconds{100LL}};
  std::this_thread::sleep_for(wait_time);
  rclcpp::shutdown();
}

TEST(TestBaseInterface, TestNotImplemented)
{
  rclcpp::init(0, nullptr);
  auto test_node = std::make_shared<autoware::vehicle::interface ::DummyExceptionNode>();
  EXPECT_THROW(test_node->send_gear_command(GearCommand()), std::runtime_error);
  EXPECT_THROW(test_node->send_hand_brake_command(HandBrakeCommand()), std::runtime_error);
  EXPECT_THROW(test_node->send_hazard_lights_command(HazardLightsCommand()), std::runtime_error);
  EXPECT_THROW(test_node->send_headlights_command(HeadlightsCommand()), std::runtime_error);
  EXPECT_THROW(test_node->send_horn_command(HornCommand()), std::runtime_error);
  EXPECT_THROW(test_node->send_wipers_command(WipersCommand()), std::runtime_error);
  EXPECT_THROW(
    test_node->send_turn_indicators_command(TurnIndicatorsCommand()), std::runtime_error);
  rclcpp::shutdown();
}

TEST_F(FakeNodeFixture, TestSub)
{
  auto test_node = std::make_shared<autoware::vehicle::interface ::DummyInterfaceNode>();
  auto cmd_pub = create_publisher<AckermannControlCommand>("control_cmd");
  auto gear_pub = create_publisher<GearCommand>("gear_cmd");
  auto hand_pub = create_publisher<HandBrakeCommand>("hand_brake_cmd");
  auto haza_pub = create_publisher<HazardLightsCommand>("hazard_lights_cmd");
  auto head_pub = create_publisher<HeadlightsCommand>("headlights_cmd");
  auto horn_pub = create_publisher<HornCommand>("horn_cmd");
  auto wipe_pub = create_publisher<WipersCommand>("wipers_cmd");
  auto turn_pub = create_publisher<TurnIndicatorsCommand>("turn_indicators_cmd");
  auto mode_srv = get_fake_node()->create_client<AutonomyModeChange>("autonomy_mode");

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto max_wait_time{std::chrono::seconds{10LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!test_node->all_received()) {
    cmd_pub->publish(AckermannControlCommand());
    gear_pub->publish(GearCommand());
    hand_pub->publish(HandBrakeCommand());
    haza_pub->publish(HazardLightsCommand());
    head_pub->publish(HeadlightsCommand());
    horn_pub->publish(HornCommand());
    wipe_pub->publish(WipersCommand());
    turn_pub->publish(TurnIndicatorsCommand());
    mode_srv->async_send_request(std::make_shared<ModeChangeRequest>());
    rclcpp::spin_some(test_node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      std::stringstream ss;
      std::vector<std::string> no_recv_names;
      test_node->get_no_receive_names(no_recv_names);
      for (const auto & name : no_recv_names) {
        ss << name << ", ";
      }
      FAIL() << "These messages are not received: " << ss.str();
    }
  }
  SUCCEED();
}

TEST_F(FakeNodeFixture, TestPub)
{
  GearReport::SharedPtr gear_report{};
  HandBrakeReport::SharedPtr handbrake_report{};
  HazardLightsReport::SharedPtr hazard_lights_report{};
  HeadlightsReport::SharedPtr headlights_report{};
  HornReport::SharedPtr horn_report{};
  WipersReport::SharedPtr wipers_report{};
  TurnIndicatorsReport::SharedPtr turn_indicators_report{};
  VehicleOdometry::SharedPtr odometry{};
  SteeringReport::SharedPtr steering_report{};
  VelocityReport::SharedPtr velocity_report{};

  auto test_node = std::make_shared<autoware::vehicle::interface ::DummyInterfaceNode>();
  auto gear_sub = create_subscription<GearReport>(
    "gear_report", *test_node,
    [&gear_report](const GearReport::SharedPtr msg) { gear_report = msg; });
  auto hand_sub = create_subscription<HandBrakeReport>(
    "hand_brake_report", *test_node,
    [&handbrake_report](const HandBrakeReport::SharedPtr msg) { handbrake_report = msg; });
  auto haza_sub = create_subscription<HazardLightsReport>(
    "hazard_lights_report", *test_node,
    [&hazard_lights_report](const HazardLightsReport::SharedPtr msg) {
      hazard_lights_report = msg;
    });
  auto head_sub = create_subscription<HeadlightsReport>(
    "headlights_report", *test_node,
    [&headlights_report](const HeadlightsReport::SharedPtr msg) { headlights_report = msg; });
  auto horn_sub = create_subscription<HornReport>(
    "horn_report", *test_node,
    [&horn_report](const HornReport::SharedPtr msg) { horn_report = msg; });
  auto wipe_sub = create_subscription<WipersReport>(
    "wipers_report", *test_node,
    [&wipers_report](const WipersReport::SharedPtr msg) { wipers_report = msg; });
  auto turn_sub = create_subscription<TurnIndicatorsReport>(
    "turn_indicators_report", *test_node,
    [&turn_indicators_report](const TurnIndicatorsReport::SharedPtr msg) {
      turn_indicators_report = msg;
    });
  auto odom_sub = create_subscription<VehicleOdometry>(
    "odom", *test_node, [&odometry](const VehicleOdometry::SharedPtr msg) { odometry = msg; });
  auto steering_sub = create_subscription<SteeringReport>(
    "steering_report", *test_node,
    [&steering_report](const SteeringReport::SharedPtr msg) { steering_report = msg; });
  auto velocity_sub = create_subscription<VelocityReport>(
    "velocity_report", *test_node,
    [&velocity_report](const VelocityReport::SharedPtr msg) { velocity_report = msg; });

  auto all_received = [&]() {
    return gear_report && handbrake_report && hazard_lights_report && headlights_report &&
           horn_report && wipers_report && turn_indicators_report && odometry && steering_report &&
           velocity_report;
  };
  auto get_no_receive_names = [&](std::vector<std::string> & names) {
    if (!gear_report) {
      names.push_back("GearReport");
    }
    if (!handbrake_report) {
      names.push_back("HandBrakeReport");
    }
    if (!hazard_lights_report) {
      names.push_back("HazardLightsReport");
    }
    if (!headlights_report) {
      names.push_back("HeadlightsReport");
    }
    if (!horn_report) {
      names.push_back("HornReport");
    }
    if (!wipers_report) {
      names.push_back("WipersReport");
    }
    if (!turn_indicators_report) {
      names.push_back("TurnIndicatorsReport");
    }
    if (!odometry) {
      names.push_back("VehicleOdometry");
    }
    if (!steering_report) {
      names.push_back("SteeringReport");
    }
    if (!velocity_report) {
      names.push_back("VelocityReport");
    }
  };

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto max_wait_time{std::chrono::seconds{10LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!all_received()) {
    rclcpp::spin_some(test_node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      std::stringstream ss;
      std::vector<std::string> no_recv_names;
      get_no_receive_names(no_recv_names);
      for (const auto & name : no_recv_names) {
        ss << name << ", ";
      }
      FAIL() << "These messages are not received: " << ss.str();
    }
  }
  SUCCEED();
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
