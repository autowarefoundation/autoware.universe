// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE_PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_HPP_
#define AUTOWARE_PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_HPP_

// since ASSERT_NO_THROW in gtest masks the exception message, redefine it.
#define ASSERT_NO_THROW_WITH_ERROR_MSG(statement)                                                \
  try {                                                                                          \
    statement;                                                                                   \
    SUCCEED();                                                                                   \
  } catch (const std::exception & e) {                                                           \
    FAIL() << "Expected: " << #statement                                                         \
           << " doesn't throw an exception.\nActual: it throws. Error message: " << e.what()     \
           << std::endl;                                                                         \
  } catch (...) {                                                                                \
    FAIL() << "Expected: " << #statement                                                         \
           << " doesn't throw an exception.\nActual: it throws. Error message is not available." \
           << std::endl;                                                                         \
  }

#include <autoware/component_interface_specs_universe/planning.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ctime>
#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_test_manager
{
class PlanningInterfaceTestManager
{
public:
  PlanningInterfaceTestManager();

  template <typename InputT>
  void publishInput(
    const rclcpp::Node::SharedPtr target_node, const std::string & topic_name, const InputT & input,
    const int repeat_count = 3) const
  {
    autoware::test_utils::publishToTargetNode(
      test_node_, target_node, topic_name, {}, input, repeat_count);
  }

  template <typename OutputT>
  void subscribeOutput(const std::string & topic_name)
  {
    const auto qos = []() {
      if constexpr (std::is_same_v<OutputT, autoware_planning_msgs::msg::Trajectory>) {
        return rclcpp::QoS{1};
      }
      return rclcpp::QoS{10};
    }();

    test_output_subs_.push_back(test_node_->create_subscription<OutputT>(
      topic_name, qos, [this](const typename OutputT::ConstSharedPtr) { received_topic_num_++; }));
  }

  void testWithNormalTrajectory(
    rclcpp::Node::SharedPtr target_node, const std::string & topic_name);
  void testWithAbnormalTrajectory(
    rclcpp::Node::SharedPtr target_node, const std::string & topic_name);

  void testWithNormalRoute(rclcpp::Node::SharedPtr target_node, const std::string & topic_name);
  void testWithAbnormalRoute(rclcpp::Node::SharedPtr target_node, const std::string & topic_name);

  void testWithBehaviorNormalRoute(
    rclcpp::Node::SharedPtr target_node, const std::string & topic_name);

  void testWithNormalPathWithLaneId(
    rclcpp::Node::SharedPtr target_node, const std::string & topic_name);
  void testWithAbnormalPathWithLaneId(
    rclcpp::Node::SharedPtr target_node, const std::string & topic_name);

  void testWithNormalPath(rclcpp::Node::SharedPtr target_node, const std::string & topic_name);
  void testWithAbnormalPath(rclcpp::Node::SharedPtr target_node, const std::string & topic_name);

  void testWithOffTrackInitialPoses(
    rclcpp::Node::SharedPtr target_node, const std::string & topic_name);

  void testWithOffTrackOdometry(
    rclcpp::Node::SharedPtr target_node, const std::string & topic_name);

  void resetReceivedTopicNum() { received_topic_num_ = 0; }

  size_t getReceivedTopicNum() const { return received_topic_num_; }

  rclcpp::Node::SharedPtr getTestNode() const { return test_node_; }

private:
  // Subscriber
  std::vector<rclcpp::SubscriptionBase::SharedPtr> test_output_subs_;

  // Node
  rclcpp::Node::SharedPtr test_node_;

  size_t received_topic_num_ = 0;
};  // class PlanningInterfaceTestManager

}  // namespace autoware::planning_test_manager

#endif  // AUTOWARE_PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_HPP_
