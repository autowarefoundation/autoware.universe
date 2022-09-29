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


#include "gtest/gtest.h"
#include "cdc_test_node.hpp"
#include "cdc_test_utils.hpp"

TEST_F(FakeNodeFixture, nodeTestTemplate)
{
  // Data to test
  DelayCompensationRefs::SharedPtr compensation_msg{nullptr};
  bool is_comp_msg_received = false;

  // Node
  std::shared_ptr<CommDelayNode> node = makeComDelayComNode();

  // Publishers
  rclcpp::Publisher<VelocityMsg>::SharedPtr vel_pub =
    this->create_publisher<VelocityMsg>("communication_delay_compensator/input/current_odometry");

  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("communication_delay_compensator/input/steering_state");

//  rclcpp::Publisher<ErrorReportMsg>::SharedPtr long_error_pub_ =
//    this->create_publisher<ErrorReportMsg>("communication_delay_compensator/input/long_errors");

  rclcpp::Publisher<ErrorReportMsg>::SharedPtr lat_error_pub_ =
    this->create_publisher<ErrorReportMsg>("communication_delay_compensator/input/lat_errors");

  rclcpp::Publisher<ControlCmdMsg>::SharedPtr control_pub_ =
    this->create_publisher<ControlCmdMsg>("communication_delay_compensator/input/control_cmd");

  // Subscribers
  rclcpp::Subscription<DelayCompensationRefs>::SharedPtr
    comm_ref_sub_ = this->create_subscription<DelayCompensationRefs>(
    "communication_delay_compensator/output/communication_delay_compensation_refs", *this->get_fake_node(),
    [&compensation_msg, &is_comp_msg_received](const DelayCompensationRefs::SharedPtr msg)
    {
      compensation_msg = msg;
      is_comp_msg_received = true;
    });

  // Broadcast transform
  auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  /**
   * Publish dummy msgs
   * */

  VelocityMsg odom_msg;
  SteeringReport steer_msg;
  ControlCmdMsg control_msg;
  ErrorReportMsg error_msg;

  odom_msg.header.stamp = node->now();
  odom_msg.twist.twist.linear.x = 0.0;

  steer_msg.steering_tire_angle = 0.0;
  steer_msg.stamp = node->now();

  control_msg.lateral.steering_tire_angle = 0.0;
  control_msg.stamp = node->now();

  error_msg.lateral_deviation_read = 0.0;
  error_msg.steering_read = 0.0;
  error_msg.heading_angle_error_read = 0.0;
  error_msg.curvature_read = 0.0;
  error_msg.stamp = node->now();

  // Broadcast transform
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();

  transform.header.stamp = node->now();
  br->sendTransform(transform);

  vel_pub->publish(odom_msg);
  steer_pub->publish(steer_msg);
  control_pub_->publish(control_msg);
  lat_error_pub_->publish(error_msg);

  test_utils::spinWhile(node);
  test_utils::waitForMessage(node, this, is_comp_msg_received, std::chrono::seconds{2LL}, false);

//  auto time_passed{std::chrono::milliseconds{0LL}};
//  const auto dt{std::chrono::milliseconds{30LL}};
//
//  while (time_passed < std::chrono::seconds{2LL})
//  {
//    rclcpp::spin_some(node);
//    rclcpp::spin_some(this->get_fake_node());
//    std::this_thread::sleep_for(dt);
//    time_passed += dt;
//  }

  ns_utils::print("is compensation_msg received ? ", is_comp_msg_received);
  int a = 1;
  ns_utils::print(a);

}