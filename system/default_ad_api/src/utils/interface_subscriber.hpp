// Copyright 2022 TIER IV, Inc.
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

#ifndef UTILS__INTERFACE_SUBSCRIBER_HPP_
#define UTILS__INTERFACE_SUBSCRIBER_HPP_

#include "tier4_autoware_utils/ros/polling_subscriber.hpp"

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace default_ad_api
{
template <typename InterfaceSpec>
rclcpp::QoS get_qos_profile()
{
  rclcpp::QoS qos_profile(InterfaceSpec::depth);
  qos_profile.durability(InterfaceSpec::durability);
  qos_profile.reliability(InterfaceSpec::reliability);
  return qos_profile;
}

template <typename InterFaceSpec>
std::shared_ptr<
  tier4_autoware_utils::InterProcessPollingSubscriber<typename InterFaceSpec::Message>>
create_polling_subscriber(rclcpp::Node * node)
{
  auto qos = get_qos_profile<InterFaceSpec>();
  return std::make_shared<
    tier4_autoware_utils::InterProcessPollingSubscriber<typename InterFaceSpec::Message>>(
    node, InterFaceSpec::name, qos);
}

}  // namespace default_ad_api

#endif  // UTILS__INTERFACE_SUBSCRIBER_HPP_
