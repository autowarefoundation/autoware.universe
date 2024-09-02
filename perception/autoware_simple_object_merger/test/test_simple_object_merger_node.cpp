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

#include "../src/simple_object_merger_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

using autoware::simple_object_merger::SimpleObjectMergerNode;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;

std::shared_ptr<autoware::test_utils::AutowareTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<autoware::test_utils::AutowareTestManager>();
  return test_manager;
}

std::shared_ptr<SimpleObjectMergerNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto package_dir =
    ament_index_cpp::get_package_share_directory("autoware_simple_object_merger");
  node_options.arguments(
    {"--ros-args", "--params-file", package_dir + "/config/simple_object_merger.param.yaml"});

  std::vector<std::string> input_topics;
  input_topics.push_back("/input/object0");
  input_topics.push_back("/input/object1");
  node_options.append_parameter_override("input_topics", input_topics);

  return std::make_shared<SimpleObjectMergerNode>(node_options);
}

std::shared_ptr<DetectedObjects> generateDetectedObjects(const rclcpp::Time stamp)
{
  auto detected_objects = std::make_shared<DetectedObjects>();
  detected_objects->header.frame_id = "base_link";
  detected_objects->header.stamp = stamp;

  DetectedObject object;

  object.kinematics.pose_with_covariance.pose.position.x = 1.0;
  object.kinematics.pose_with_covariance.pose.position.y = 2.0;
  object.kinematics.pose_with_covariance.pose.position.z = 3.0;
  object.kinematics.pose_with_covariance.pose.orientation.x = 0.0;
  object.kinematics.pose_with_covariance.pose.orientation.y = 0.0;
  object.kinematics.pose_with_covariance.pose.orientation.z = 0.0;
  object.kinematics.pose_with_covariance.pose.orientation.w = 1.0;

  detected_objects->objects.push_back(object);

  return detected_objects;
}

TEST(SimpleObjectMergerTest, testSimpleObjectMergerTwoTopics)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // Check output topic
  int counter = 0;
  auto callback = [&counter](const DetectedObjects::ConstSharedPtr msg) {
    (void)msg;
    ++counter;
  };

  // Create a DetectedObjects message
  rclcpp::Time stamp(0);
  DetectedObjects msg = *generateDetectedObjects(stamp);

  // Publish the message to the input topics
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "/input/object0", msg);
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "/input/object1", msg);

  // Check if the message is published to the output topic
  test_manager->set_subscriber<DetectedObjects>("/output/object", callback);

  // Wait for the message to be published
  // test_manager->spin(test_target_node, 1s);

  // Check if the message is published to the output topic
  EXPECT_GE(counter, 1);

  rclcpp::shutdown();
}
