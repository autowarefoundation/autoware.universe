// Copyright 2024 The Autoware Contributors
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

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/published_time_publisher.hpp>

#include <autoware_internal_msgs/msg/published_time.hpp>
#include <std_msgs/msg/header.hpp>

#include <gtest/gtest.h>

class PublishedTimePublisherTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node_{nullptr};
  std::shared_ptr<tier4_autoware_utils::PublishedTimePublisher> published_time_publisher_{nullptr};

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Header>> first_test_publisher_{nullptr};
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Header>> second_test_publisher_{nullptr};

  std::shared_ptr<rclcpp::Subscription<autoware_internal_msgs::msg::PublishedTime>>
    first_test_subscriber_{nullptr};
  std::shared_ptr<rclcpp::Subscription<autoware_internal_msgs::msg::PublishedTime>>
    second_test_subscriber_{nullptr};

  autoware_internal_msgs::msg::PublishedTime::ConstSharedPtr first_published_time_{nullptr};
  autoware_internal_msgs::msg::PublishedTime::ConstSharedPtr second_published_time_{nullptr};

  void SetUp() override
  {
    ASSERT_TRUE(rclcpp::ok());

    // Simplify node and topic names for brevity and uniqueness
    const std::string test_name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::string base_name =
      "published_time_publisher_" + test_name;           // Base name for node and topics
    const std::string suffix = "/debug/published_time";  // Suffix for published time topics

    // Create a node
    node_ = std::make_shared<rclcpp::Node>(base_name + "_node");

    // Create the first publisher
    first_test_publisher_ =
      node_->create_publisher<std_msgs::msg::Header>(base_name + "_topic1", 1);

    // Create the second publisher
    second_test_publisher_ =
      node_->create_publisher<std_msgs::msg::Header>(base_name + "_topic2", 1);

    // Create a PublishedTimePublisher
    published_time_publisher_ =
      std::make_shared<tier4_autoware_utils::PublishedTimePublisher>(node_.get());

    // Create the first subscriber
    first_test_subscriber_ = node_->create_subscription<autoware_internal_msgs::msg::PublishedTime>(
      base_name + "_topic1" + suffix, 1,
      [this](autoware_internal_msgs::msg::PublishedTime::ConstSharedPtr msg) {
        this->first_published_time_ = std::move(msg);
      });

    // Create the second subscriber
    second_test_subscriber_ =
      node_->create_subscription<autoware_internal_msgs::msg::PublishedTime>(
        base_name + "_topic2" + suffix, 1,
        [this](autoware_internal_msgs::msg::PublishedTime::ConstSharedPtr msg) {
          this->second_published_time_ = std::move(msg);
        });

    rclcpp::spin_some(node_);
  }

  void TearDown() override {}
};

TEST_F(PublishedTimePublisherTest, PublishMsgWithHeader)
{
  // Check if the PublishedTimePublisher is created
  ASSERT_TRUE(published_time_publisher_ != nullptr);

  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(1234);

  // Use Published Time Publisher .publish_if_subscribed() with a header
  published_time_publisher_->publish_if_subscribed(first_test_publisher_, header);
  rclcpp::spin_some(node_);

  // Check if the published_time_ is created
  ASSERT_TRUE(first_published_time_ != nullptr);

  // Check if the published time is the same as the header
  EXPECT_EQ(first_published_time_->header.stamp, header.stamp);
}

TEST_F(PublishedTimePublisherTest, PublishMsgWithTimestamp)
{
  // Check if the PublishedTimePublisher is created
  ASSERT_TRUE(published_time_publisher_ != nullptr);

  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(4321);

  // Use Published Time Publisher .publish_if_subscribed() with a timestamp
  published_time_publisher_->publish_if_subscribed(first_test_publisher_, header.stamp);
  rclcpp::spin_some(node_);

  // Check if the published_time_ is created
  ASSERT_TRUE(first_published_time_ != nullptr);

  // Check if the published time is the same as the header
  EXPECT_EQ(first_published_time_->header.stamp, header.stamp);
}

TEST_F(PublishedTimePublisherTest, MultiplePublishMsgWithHeader)
{
  // Check if the PublishedTimePublisher is created
  ASSERT_TRUE(published_time_publisher_ != nullptr);

  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(12345);

  // Use Published Time Publisher .publish_if_subscribed() with a header for multiple publishers
  published_time_publisher_->publish_if_subscribed(first_test_publisher_, header);
  published_time_publisher_->publish_if_subscribed(second_test_publisher_, header);
  rclcpp::spin_some(node_);

  // Check if the published_time_ is created
  ASSERT_TRUE(first_published_time_ != nullptr);
  ASSERT_TRUE(second_published_time_ != nullptr);

  // Check if the published time is the same as the header
  EXPECT_EQ(first_published_time_->header.stamp, header.stamp);
  EXPECT_EQ(second_published_time_->header.stamp, header.stamp);
}

TEST_F(PublishedTimePublisherTest, MultiplePublishMsgWithTimestamp)
{
  // Check if the PublishedTimePublisher is created
  ASSERT_TRUE(published_time_publisher_ != nullptr);

  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(12345);

  // Use Published Time Publisher .publish_if_subscribed() with a timestamp for multiple publishers
  published_time_publisher_->publish_if_subscribed(first_test_publisher_, header.stamp);
  published_time_publisher_->publish_if_subscribed(second_test_publisher_, header.stamp);
  rclcpp::spin_some(node_);

  // Check if the published_time_ is created
  ASSERT_TRUE(first_published_time_ != nullptr);
  ASSERT_TRUE(second_published_time_ != nullptr);

  // Check if the published time is the same as the header
  EXPECT_EQ(first_published_time_->header.stamp, header.stamp);
  EXPECT_EQ(second_published_time_->header.stamp, header.stamp);
}
