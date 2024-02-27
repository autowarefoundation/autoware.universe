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

#ifndef TIER4_AUTOWARE_UTILS__ROS__PUBLISHED_TIME_PUBLISHER_HPP_
#define TIER4_AUTOWARE_UTILS__ROS__PUBLISHED_TIME_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_msgs/msg/published_time.hpp>
#include <std_msgs/msg/header.hpp>

#include <functional>
#include <map>
#include <string>

namespace tier4_autoware_utils
{
using autoware_internal_msgs::msg::PublishedTime;

struct GidHash
{
  size_t operator()(const rmw_gid_t & gid) const noexcept
  {
    // Hashing function that computes a hash value for the GID
    std::hash<std::string> hasher;
    return hasher(std::string(reinterpret_cast<const char *>(gid.data), RMW_GID_STORAGE_SIZE));
  }
};

struct GidEqual
{
  bool operator()(const rmw_gid_t & lhs, const rmw_gid_t & rhs) const noexcept
  {
    return std::memcmp(lhs.data, rhs.data, RMW_GID_STORAGE_SIZE) == 0;
  }
};

class PublishedTimePublisher
{
public:
  static std::unique_ptr<PublishedTimePublisher> create(
    rclcpp::Node * node, const std::string & end_name = "/debug/published_time",
    const rclcpp::QoS & qos = rclcpp::QoS(1))
  {
    const bool use_published_time = node->declare_parameter<bool>("use_published_time", false);
    if (use_published_time) {
      return std::unique_ptr<PublishedTimePublisher>(
        new PublishedTimePublisher(node, end_name, qos));
    } else {
      return nullptr;
    }
  }

  void publish(const rclcpp::PublisherBase::ConstSharedPtr & publisher, const rclcpp::Time & stamp)
  {
    const auto & gid = publisher->get_gid();
    const auto & topic_name = publisher->get_topic_name();

    // if the publisher is not in the map, create a new publisher for published time
    if (publishers_.find(gid) == publishers_.end()) {
      publishers_[gid] = node_->create_publisher<PublishedTime>(
        static_cast<std::string>(topic_name) + end_name_, qos_);
    }

    const auto & pub_published_time_ = publishers_[gid];

    // Check if there are any subscribers, otherwise don't do anything
    if (pub_published_time_->get_subscription_count() > 0) {
      PublishedTime published_time;

      published_time.header.stamp = stamp;
      published_time.published_stamp = rclcpp::Clock().now();

      pub_published_time_->publish(published_time);
    }
  }

  void publish(
    const rclcpp::PublisherBase::ConstSharedPtr & publisher, const std_msgs::msg::Header & header)
  {
    const auto & gid = publisher->get_gid();
    const auto & topic_name = publisher->get_topic_name();

    // if the publisher is not in the map, create a new publisher for published time
    if (publishers_.find(gid) == publishers_.end()) {
      publishers_[gid] = node_->create_publisher<PublishedTime>(
        static_cast<std::string>(topic_name) + end_name_, qos_);
    }

    const auto & pub_published_time_ = publishers_[gid];

    // Check if there are any subscribers, otherwise don't do anything
    if (pub_published_time_->get_subscription_count() > 0) {
      PublishedTime published_time;

      published_time.header = header;
      published_time.published_stamp = rclcpp::Clock().now();

      pub_published_time_->publish(published_time);
    }
  }

private:
  explicit PublishedTimePublisher(
    rclcpp::Node * node, const std::string & end_name, const rclcpp::QoS & qos)
  : node_(node), end_name_(end_name), qos_(qos)
  {
  }

  rclcpp::Node * node_;
  std::string end_name_;
  rclcpp::QoS qos_;

  // store them for each different publisher of the node
  std::unordered_map<rmw_gid_t, rclcpp::Publisher<PublishedTime>::SharedPtr, GidHash, GidEqual>
    publishers_;
};
}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__ROS__PUBLISHED_TIME_PUBLISHER_HPP_
