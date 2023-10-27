// Copyright 2023- Autoware Foundation
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

#include "../src/pose_instability_detector.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>

#include <memory>
#include <string>
#include <vector>

class TestPoseInstabilityDetector : public ::testing::Test
{
  using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
  using Odometry = nav_msgs::msg::Odometry;
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

protected:
  void SetUp() override
  {
    const std::string yaml_path =
      "../../install/pose_instability_detector/share/pose_instability_detector/config/"
      "pose_instability_detector.param.yaml";

    rcl_params_t * params_st = rcl_yaml_node_struct_init(rcl_get_default_allocator());
    if (!rcl_parse_yaml_file(yaml_path.c_str(), params_st)) {
      std::cerr << "Failed to parse yaml file : " << yaml_path << std::endl;
      std::exit(1);
    }

    const rclcpp::ParameterMap param_map = rclcpp::parameter_map_from(params_st, "");
    rclcpp::NodeOptions node_options;
    for (const auto & param_pair : param_map) {
      for (const auto & param : param_pair.second) {
        node_options.parameter_overrides().push_back(param);
      }
    }
    node_ = std::make_shared<PoseInstabilityDetector>(node_options);
    executor_.add_node(node_);

    // internal test publisher
    odometry_publisher_ = node_->create_publisher<Odometry>("~/input/odometry", 10);
    twist_publisher_ = node_->create_publisher<TwistWithCovarianceStamped>("~/input/twist", 10);

    // internal test subscriber
    diagnostic_subscriber_ = node_->create_subscription<DiagnosticArray>(
      "/diagnostics", 10,
      std::bind(&TestPoseInstabilityDetector::CallbackDiagnostic, this, std::placeholders::_1));

    rcl_yaml_node_struct_fini(params_st);
  }

  void TearDown() override { executor_.remove_node(node_); }

  void SendOdometryMessage(
    const builtin_interfaces::msg::Time timestamp, const double x, const double y, const double z)
  {
    Odometry message{};
    message.header.stamp = timestamp;
    message.pose.pose.position.x = x;
    message.pose.pose.position.y = y;
    message.pose.pose.position.z = z;
    odometry_publisher_->publish(message);
  }

  void SendTwistMessage(
    const builtin_interfaces::msg::Time timestamp, const double x, const double y, const double z)
  {
    TwistWithCovarianceStamped message{};
    message.header.stamp = timestamp;
    message.twist.twist.linear.x = x;
    message.twist.twist.linear.y = y;
    message.twist.twist.linear.z = z;
    twist_publisher_->publish(message);
  }

  void CallbackDiagnostic(const DiagnosticArray::ConstSharedPtr msg)
  {
    received_diagnostic_array_ = *msg;
    received_diagnostic_array_flag_ = true;
  }

  bool IsDiagnosticStatusLevel(
    const DiagnosticStatus & diagnostic_status, const int32_t level) const
  {
    return diagnostic_status.level == level;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<PoseInstabilityDetector> node_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr twist_publisher_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostic_subscriber_;
  DiagnosticArray received_diagnostic_array_;
  bool received_diagnostic_array_flag_ = false;
};

TEST_F(TestPoseInstabilityDetector, test_normal_behavior)  // NOLINT
{
  // send the first odometry message
  builtin_interfaces::msg::Time timestamp{};
  timestamp.sec = 0;
  timestamp.nanosec = 0;
  SendOdometryMessage(timestamp, 0.0, 0.0, 0.0);

  // process the above message (by timer_callback)
  received_diagnostic_array_flag_ = false;
  while (!received_diagnostic_array_flag_) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // send the twist message
  timestamp.sec = 1;
  timestamp.nanosec = 0;
  SendTwistMessage(timestamp, 1.0, 0.0, 0.0);

  // send the second odometry message
  timestamp.sec = 2;
  timestamp.nanosec = 0;
  SendOdometryMessage(timestamp, 1.0, 0.0, 0.0);

  // process the above messages (by timer_callback)
  received_diagnostic_array_flag_ = false;
  while (!received_diagnostic_array_flag_) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // check result
  const diagnostic_msgs::msg::DiagnosticStatus & diagnostic_status =
    received_diagnostic_array_.status[0];
  EXPECT_TRUE(diagnostic_status.level == diagnostic_msgs::msg::DiagnosticStatus::OK);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
