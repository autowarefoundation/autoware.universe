// Copyright 2023 The Autoware Foundation
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
#ifndef ADAPTER_TRAFFIC_SIGNALS_HPP_
#define ADAPTER_TRAFFIC_SIGNALS_HPP_

#include "adapter_base.hpp"

#include <rclcpp/rclcpp.hpp>

// #include <autoware_perception_msgs/msg/traffic_signal.hpp>
// #include <autoware_perception_msgs/msg/traffic_signal_element.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <rviz_2d_overlay_msgs/msg/traffic_signal_array_ui.hpp>

#include <string>

namespace autoware_auto_msgs_adapter
{
using autoware_perception_msgs::msg::TrafficSignalArray;
using rviz_2d_overlay_msgs::msg::TrafficSignalArrayUI;

class AdapterTrafficSignals
: public autoware_auto_msgs_adapter::AdapterBase<TrafficSignalArray, TrafficSignalArrayUI>
{
public:
  AdapterTrafficSignals(
    rclcpp::Node & node, const std::string & topic_name_source,
    const std::string & topic_name_target, const rclcpp::QoS & qos = rclcpp::QoS{1})
  : AdapterBase(node, topic_name_source, topic_name_target, qos)
  {
    RCLCPP_DEBUG(
      node.get_logger(), "AdapterTrafficSignals is initialized to convert: %s -> %s",
      topic_name_source.c_str(), topic_name_target.c_str());
  }

protected:
  TrafficSignalArrayUI convert(const TrafficSignalArray & msg_source) override
  {
    TrafficSignalArrayUI msg_target;
    msg_target.stamp = msg_source.stamp;
    msg_target.traffic_signals = msg_source.signals;

    return msg_target;
  }
};
}  // namespace autoware_auto_msgs_adapter

#endif  // ADAPTER_TRAFFIC_SIGNALS_HPP_
