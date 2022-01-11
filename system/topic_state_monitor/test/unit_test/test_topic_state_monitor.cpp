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

#include "topic_state_monitor/topic_state_monitor.hpp"
#include "gtest/gtest.h"


using namespace topic_state_monitor;


class TestTopicStateMonitor : public ::testing::Test
{
public:
  TestTopicStateMonitor()
  {
    rclcpp::init(0, nullptr);
    test_node_ = std::make_shared<rclcpp::Node>("test_node");
  }

  ~TestTopicStateMonitor() override
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr test_node_;
};

TEST_F(TestTopicStateMonitor, getTopicRateShouldReturnZeroWhenNoMessagesReceived)
{
  TopicStateMonitor topic_state_monitor(*test_node_);
  topic_state_monitor.setParam(Param{
    "test_topic",
    "test_type",
    false,
    false,
    "test_diag_name",
    0.1,
    0.2,
    0.3,
    10
  });

  // Test that the topic rate is max(100000.0) when no messages have been received.
  EXPECT_EQ(topic_state_monitor.getTopicStatus(), TopicStatus::NotReceived);
  EXPECT_EQ(topic_state_monitor.getTopicRate(), 100000.0);
}

TEST_F(TestTopicStateMonitor, getTopicRateShouldReturnExpectedRate)
{
  // rclcpp::Node test_node("test_node");
  TopicStateMonitor topic_state_monitor(*test_node_);
  topic_state_monitor.setParam(Param{
    "test_topic",
    "test_type",
    false,
    false,
    "test_diag_name",
    0.1,
    0.2,
    0.3,
    10
  });

  // Set 2 messages with a rate of 10.0.
  topic_state_monitor.update();
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  topic_state_monitor.update();

  // Test that the topic rate is 10.0 when 2 messages have been received.
  EXPECT_NEAR(topic_state_monitor.getTopicRate(), 10.0, 0.05);
  EXPECT_EQ(topic_state_monitor.getTopicStatus(), TopicStatus::Ok);
}
