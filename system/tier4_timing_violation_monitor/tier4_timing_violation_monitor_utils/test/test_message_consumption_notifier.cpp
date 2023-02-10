// Copyright 2022 Tier IV, Inc.
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

#include "tier4_system_msgs/msg/message_tracking_tag.hpp"

#include <gtest/gtest.h>

#include "tier4_timing_violation_monitor_utils/message_consumption_notifier.hpp"

using tier4_timing_violation_monitor_utils::MessageConsumptionNotifier;
using MessageTrackingTag = MessageConsumptionNotifier::MessageTrackingTag;

class NodeWithNotifier : public rclcpp::Node
{
public:
  explicit NodeWithNotifier(const std::string & node_name)
  : rclcpp::Node(node_name)
  {
    notifier_ = std::make_unique<MessageConsumptionNotifier>(this, "/test_notify", 1);
  }

  void notify(const builtin_interfaces::msg::Time & t)
  {
    notifier_->notify(t);
  }

private:
  std::unique_ptr<MessageConsumptionNotifier> notifier_;
};

class TestMessageConsumptionNotifier : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestMessageConsumptionNotifier, simple_case) {
  auto node = std::make_shared<NodeWithNotifier>("node");
  auto checker_node = std::make_shared<rclcpp::Node>("checker_node");

  bool checker_sub_called = false;
  auto checker_sub = checker_node->create_subscription<MessageTrackingTag>(
    "/test_notify", 1,
    [&checker_sub_called](MessageTrackingTag::UniquePtr msg) -> void
    {
      (void) msg;
      checker_sub_called = true;
    });

  EXPECT_FALSE(checker_sub_called);

  builtin_interfaces::msg::Time t;
  t.sec = 123;
  t.nanosec = 456u;
  node->notify(t);

  rclcpp::spin_some(checker_node);
  rclcpp::Rate(50).sleep();

  EXPECT_TRUE(checker_sub_called);
}
