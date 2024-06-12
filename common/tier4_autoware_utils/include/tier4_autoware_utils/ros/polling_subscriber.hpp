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

#ifndef TIER4_AUTOWARE_UTILS__ROS__POLLING_SUBSCRIBER_HPP_
#define TIER4_AUTOWARE_UTILS__ROS__POLLING_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace tier4_autoware_utils
{

/**
 * @brief Enum to define different polling policies
 */
enum class PollingPolicy {
  LATEST,  ///< Take the latest data from queue
  NEWEST,  ///< Take the latest data from queue if such data existed, otherwise return nullopt
  ALL,     ///< Take all data from queue
};

template <typename T, PollingPolicy policy = PollingPolicy::LATEST>
class InterProcessPollingSubscriber;

/**
 * @brief Implementation class for InterProcessPollingSubscriber
 */
template <typename T, PollingPolicy policy>
class InterProcessPollingSubscriberImpl
{
public:
  typename rclcpp::Subscription<T>::SharedPtr subscriber_;  ///< Subscription object

public:
  using SharedPtr = std::shared_ptr<InterProcessPollingSubscriber<T, policy>>;

  /**
   * @brief Factory method to create a subscription
   * @param node Node to create the subscription
   * @param topic_name Name of the topic to subscribe to
   * @param qos Quality of Service settings for the subscription
   * @return Shared pointer to the created InterProcessPollingSubscriber
   */
  static SharedPtr create_subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  {
    return std::make_shared<InterProcessPollingSubscriber<T, policy>>(node, topic_name, qos);
  }

  /**
   * @brief Constructor
   * @param node Node to create the subscription
   * @param topic_name Name of the topic to subscribe to
   * @param qos Quality of Service settings for the subscription
   */
  explicit InterProcessPollingSubscriberImpl(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  {
    auto noexec_callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto noexec_subscription_options = rclcpp::SubscriptionOptions();
    noexec_subscription_options.callback_group = noexec_callback_group;

    subscriber_ = node->create_subscription<T>(
      topic_name, qos,
      [node]([[maybe_unused]] const typename T::ConstSharedPtr msg) { assert(false); },
      noexec_subscription_options);
  }
};

/**
 * @brief Specialization of InterProcessPollingSubscriber for the LATEST polling policy
 * @tparam T Message type
 */
template <typename T>
class InterProcessPollingSubscriber<T, PollingPolicy::LATEST>
: public InterProcessPollingSubscriberImpl<T, PollingPolicy::LATEST>
{
private:
  typename T::SharedPtr data_;  ///< Data pointer to store the latest data

public:
  /**
   * @brief Constructor
   * @param node Node to create the subscription
   * @param topic_name Name of the topic to subscribe to
   * @param qos Quality of Service settings for the subscription
   */
  explicit InterProcessPollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  : InterProcessPollingSubscriberImpl<T, PollingPolicy::LATEST>(node, topic_name, qos)
  {
    if (qos.get_rmw_qos_profile().depth > 1) {
      throw std::invalid_argument(
        "InterProcessPollingSubscriber will be used with depth > 1, which may cause inefficient "
        "serialization while updateLatestData()");
    }
  };

  /**
   * @brief Take and return the latest data from DDS queue if such data existed, otherwise return
   * previous taken data ("stale" data)
   * @return Const shared pointer to the taken data
   * @note If you want to ignore "stale" data, you should use takeNewData() instead
   */
  typename T::ConstSharedPtr takeData()
  {
    auto new_data = std::make_shared<T>();
    rclcpp::MessageInfo message_info;
    const bool success = this->subscriber_->take(*new_data, message_info);
    if (success) {
      data_ = new_data;
    }

    return data_;
  };
};

/**
 * @brief Specialization of InterProcessPollingSubscriber for the ALL polling policy
 * @tparam T Message type
 */
template <typename T>
class InterProcessPollingSubscriber<T, PollingPolicy::ALL>
: public InterProcessPollingSubscriberImpl<T, PollingPolicy::ALL>
{
public:
  /**
   * @brief Constructor
   * @param node Node to create the subscription
   * @param topic_name Name of the topic to subscribe to
   * @param qos Quality of Service settings for the subscription
   */
  explicit InterProcessPollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  : InterProcessPollingSubscriberImpl<T, PollingPolicy::ALL>(node, topic_name, qos)
  {
  }

  /**
   * @brief Take and return all the data from DDS queue
   * @return Vector of const shared pointers to the taken data
   */
  std::vector<typename T::ConstSharedPtr> takeData()
  {
    std::vector<typename T::ConstSharedPtr> data;
    rclcpp::MessageInfo message_info;
    while (true) {
      auto datum = std::make_shared<T>();
      if (this->subscriber_->take(*datum, message_info)) {
        data.push_back(datum);
      } else {
        break;
      }
    }
    return data;
  };
};

/**
 * @brief Specialization of InterProcessPollingSubscriber for the NEWEST polling policy
 * @tparam T Message type
 */
template <typename T>
class InterProcessPollingSubscriber<T, PollingPolicy::NEWEST>
: public InterProcessPollingSubscriberImpl<T, PollingPolicy::NEWEST>
{
private:
  typename T::SharedPtr data_;  ///< Data pointer to store the newest data

public:
  /**
   * @brief Constructor
   * @param node Node to create the subscription
   * @param topic_name Name of the topic to subscribe to
   * @param qos Quality of Service settings for the subscription
   */
  explicit InterProcessPollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  : InterProcessPollingSubscriberImpl<T, PollingPolicy::NEWEST>(node, topic_name, qos)
  {
    if (qos.get_rmw_qos_profile().depth > 1) {
      throw std::invalid_argument(
        "InterProcessPollingSubscriber will be used with depth > 1, which may cause inefficient "
        "serialization while updateLatestData()");
    }
  };

  /**
   * @brief Take and return the latest data from DDS queue if such data existed, otherwise return
   * nullptr instead.
   * @return Const shared pointer to the taken data, or nullopt if no data existed
   */
  typename T::ConstSharedPtr takeData()
  {
    auto new_data = std::make_shared<T>();
    rclcpp::MessageInfo message_info;
    const bool success = this->subscriber_->take(*new_data, message_info);
    if (success) {
      data_ = new_data;
      return data_;
    } else {
      return nullptr;
    }
  }
};

}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__ROS__POLLING_SUBSCRIBER_HPP_
