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

#include "detected_object_validation/detected_object_filter/object_position_filter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

using autoware_perception_msgs::msg::DetectedObjects;
using object_position_filter::ObjectPositionFilterNode;

class AutowareTestManager
{
public:
  AutowareTestManager()
  {
    test_node_ = std::make_shared<rclcpp::Node>("autoware_test_manager_node");
  }

  template <typename MessageType>
  void test_pub_msg(
    rclcpp::Node::SharedPtr target_node, const std::string & topic_name, MessageType & msg)
  {
    if (publishers_.find(topic_name) == publishers_.end()) {
      auto publisher = test_node_->create_publisher<MessageType>(topic_name, 10);
      publishers_[topic_name] = std::static_pointer_cast<void>(publisher);
    }

    auto publisher =
      std::static_pointer_cast<rclcpp::Publisher<MessageType>>(publishers_[topic_name]);

    autoware::test_utils::publishToTargetNode(test_node_, target_node, topic_name, publisher, msg);
    RCLCPP_INFO(test_node_->get_logger(), "Published message on topic '%s'", topic_name.c_str());
  }

  template <typename MessageType>
  void set_subscriber(
    const std::string & topic_name,
    std::function<void(const typename MessageType::ConstSharedPtr)> callback)
  {
    if (subscribers_.find(topic_name) == subscribers_.end()) {
      auto subscriber = test_node_->create_subscription<MessageType>(
        topic_name, rclcpp::QoS{1},
        [callback](const typename MessageType::ConstSharedPtr msg) { callback(msg); });
      subscribers_[topic_name] = std::static_pointer_cast<void>(subscriber);
    }
  }

protected:
  // Publisher
  std::unordered_map<std::string, std::shared_ptr<void>> publishers_;
  std::unordered_map<std::string, std::shared_ptr<void>> subscribers_;

  // Node
  rclcpp::Node::SharedPtr test_node_;
};  // class AutowareTestManager

std::shared_ptr<AutowareTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<AutowareTestManager>();
  return test_manager;
}

std::shared_ptr<ObjectPositionFilterNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto detected_object_validation_dir =
    ament_index_cpp::get_package_share_directory("detected_object_validation");
  node_options.arguments(
    {"--ros-args", "--params-file",
     detected_object_validation_dir + "/config/object_position_filter.param.yaml"});
  return std::make_shared<ObjectPositionFilterNode>(node_options);
}

TEST(DetectedObjectValidationTest, testObjectPositionFilterEmptyObject)
{
  rclcpp::init(0, nullptr);
  const std::string input_topic = "/input/object";
  const std::string output_topic = "/output/object";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  int counter = 0;
  auto callback = [&counter](const DetectedObjects::ConstSharedPtr msg) {
    (void)msg;
    ++counter;
  };
  test_manager->set_subscriber<DetectedObjects>(output_topic, callback);

  DetectedObjects msg;
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, input_topic, msg);
  EXPECT_GE(counter, 1);
  rclcpp::shutdown();
}

TEST(DetectedObjectValidationTest, testObjectPositionFilterSeveralObjects)
{
  rclcpp::init(0, nullptr);
  const std::string input_topic = "/input/object";
  const std::string output_topic = "/output/object";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // Create a DetectedObjects message with several objects
  DetectedObjects msg;
  msg.header.frame_id = "base_link";

  // Object 1: Inside bounds
  {
    autoware_perception_msgs::msg::DetectedObject object;
    object.kinematics.pose_with_covariance.pose.position.x = 10.0;
    object.kinematics.pose_with_covariance.pose.position.y = 5.0;
    object.classification.resize(1);
    object.classification[0].label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    msg.objects.push_back(object);
  }

  // Object 2: Outside bounds (x-axis)
  {
    autoware_perception_msgs::msg::DetectedObject object;
    object.kinematics.pose_with_covariance.pose.position.x = 110.0;
    object.kinematics.pose_with_covariance.pose.position.y = 5.0;
    object.classification.resize(1);
    object.classification[0].label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    msg.objects.push_back(object);
  }

  // Object 3: Outside bounds (y-axis)
  {
    autoware_perception_msgs::msg::DetectedObject object;
    object.kinematics.pose_with_covariance.pose.position.x = 10.0;
    object.kinematics.pose_with_covariance.pose.position.y = 60.0;
    object.classification.resize(1);
    object.classification[0].label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    msg.objects.push_back(object);
  }

  // Object 4: Inside bounds
  {
    autoware_perception_msgs::msg::DetectedObject object;
    object.kinematics.pose_with_covariance.pose.position.x = 20.0;
    object.kinematics.pose_with_covariance.pose.position.y = -5.0;
    object.classification.resize(1);
    object.classification[0].label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    msg.objects.push_back(object);
  }

  DetectedObjects latest_msg;
  auto callback = [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; };
  test_manager->set_subscriber<DetectedObjects>(output_topic, callback);

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, input_topic, msg);
  EXPECT_EQ(latest_msg.objects.size(), 2);
  rclcpp::shutdown();
}
