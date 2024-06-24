// Copyright 2024 TIER IV, Inc.
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

#include "simple_object_merger/simple_object_merger_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;
using simple_object_merger::SimpleObjectMergerNode;

std::shared_ptr<autoware::test_utils::AutowareTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<autoware::test_utils::AutowareTestManager>();
  return test_manager;
}

std::shared_ptr<SimpleObjectMergerNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto simple_object_merger_dir =
    ament_index_cpp::get_package_share_directory("simple_object_merger");
  node_options.arguments(
    {"--ros-args", "--params-file",
     simple_object_merger_dir + "/config/simple_object_merger.param.yaml"});
  node_options.append_parameter_override("use_sim_time", true);
  return std::make_shared<SimpleObjectMergerNode>(node_options);
}

TEST(SimpleObjectMergerNodeTest, testSimpleObjectMerge)
{
  rclcpp::init(0, nullptr);
  const std::string input_topic_1 = "/input/objects1";
  const std::string input_topic_2 = "/input/objects2";
  const std::string output_topic = "/simple_object_merger/output/objects";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // Create subscriber
  size_t latest_msg_objects_size = 0;
  auto callback = [&latest_msg_objects_size](const DetectedObjects::ConstSharedPtr msg) {
    latest_msg_objects_size = msg->objects.size();
  };
  test_manager->set_subscriber<DetectedObjects>(output_topic, callback);

  // Create a DetectedObjects message with one object
  DetectedObjects msg;
  msg.header.frame_id = "base_link";
  {
    DetectedObject object;
    object.kinematics.pose_with_covariance.pose.position.x = 10.0;
    object.kinematics.pose_with_covariance.pose.position.y = 5.0;
    object.shape.dimensions.x = 1.0;
    object.shape.dimensions.y = 1.0;
    object.shape.dimensions.z = 1.0;
    object.classification.resize(1);
    object.classification[0].label = ObjectClassification::UNKNOWN;
    msg.objects.push_back(object);
  }

  // Set start time
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::Time current_time(0, 0, clock->get_clock_type());

  // Publish the message to topic1
  test_manager->jump_clock(current_time);
  msg.header.stamp = current_time;
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, input_topic_1, msg);

  // Increment time for 0.1 sec
  current_time = current_time + rclcpp::Duration(0, 1e8); // 0.1 sec

  // Publish the message to topic2
  test_manager->jump_clock(current_time);
  msg.header.stamp = current_time;
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, input_topic_2, msg);

  EXPECT_EQ(latest_msg_objects_size, 2u);
  rclcpp::shutdown();
}



TEST(SimpleObjectMergerNodeTest, testSimpleObjectMergeTimeOut)
{
  rclcpp::init(0, nullptr);
  const std::string input_topic_1 = "/input/objects1";
  const std::string input_topic_2 = "/input/objects2";
  const std::string output_topic = "/simple_object_merger/output/objects";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // Create subscriber
  size_t latest_msg_objects_size = 0;
  auto callback = [&latest_msg_objects_size](const DetectedObjects::ConstSharedPtr msg) {
    latest_msg_objects_size = msg->objects.size();
  };
  test_manager->set_subscriber<DetectedObjects>(output_topic, callback);

  // Create a DetectedObjects message with one object
  DetectedObjects msg;
  msg.header.frame_id = "base_link";
  {
    DetectedObject object;
    object.kinematics.pose_with_covariance.pose.position.x = 10.0;
    object.kinematics.pose_with_covariance.pose.position.y = 5.0;
    object.shape.dimensions.x = 1.0;
    object.shape.dimensions.y = 1.0;
    object.shape.dimensions.z = 1.0;
    object.classification.resize(1);
    object.classification[0].label = ObjectClassification::UNKNOWN;
    msg.objects.push_back(object);
  }

  // Set start time
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::Time current_time(0, 0, clock->get_clock_type());

  // Publish the message to topic1
  test_manager->jump_clock(current_time);
  msg.header.stamp = current_time;
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, input_topic_1, msg);

  // Increment time for 2.0 sec
  current_time = current_time + rclcpp::Duration(0, 2e9); // 2.0 sec

  // Publish the message to topic2
  test_manager->jump_clock(current_time);
  msg.header.stamp = current_time;
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, input_topic_2, msg);

  EXPECT_EQ(latest_msg_objects_size, 1u);
  rclcpp::shutdown();
}
