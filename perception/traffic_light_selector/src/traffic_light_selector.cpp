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

#include "traffic_light_selector.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <vector>

namespace lanelet
{

using TrafficLightConstPtr = std::shared_ptr<const TrafficLight>;

std::vector<TrafficLightConstPtr> filter_traffic_signals(const LaneletMapConstPtr map)
{
  std::vector<TrafficLightConstPtr> signals;
  for (const auto & element : map->regulatoryElementLayer) {
    const auto signal = std::dynamic_pointer_cast<const TrafficLight>(element);
    if (signal) {
      signals.push_back(signal);
    }
  }
  return signals;
}

}  // namespace lanelet

namespace utils
{

using Input = autoware_auto_perception_msgs::msg::TrafficLight;
using Output = autoware_perception_msgs::msg::TrafficLightElement;

template <class K, class V>
V at_or(const std::unordered_map<K, V> & map, const K & key, const V & value)
{
  return map.count(key) ? map.at(key) : value;
}

Output convert(const Input & input)
{
  // clang-format off
  static const std::unordered_map<Input::_color_type, Output::_color_type> color_map({
    {Input::RED, Output::RED},
    {Input::AMBER, Output::AMBER},
    {Input::GREEN, Output::GREEN},
    {Input::WHITE, Output::WHITE}
  });
  static const std::unordered_map<Input::_shape_type, Output::_shape_type> shape_map({
    {Input::CIRCLE, Output::CIRCLE},
    {Input::LEFT_ARROW, Output::LEFT_ARROW},
    {Input::RIGHT_ARROW, Output::RIGHT_ARROW},
    {Input::UP_ARROW, Output::UP_ARROW},
    {Input::DOWN_ARROW, Output::DOWN_ARROW},
    {Input::DOWN_LEFT_ARROW, Output::DOWN_LEFT_ARROW},
    {Input::DOWN_RIGHT_ARROW, Output::DOWN_RIGHT_ARROW},
    {Input::CROSS, Output::CROSS}
  });
  static const std::unordered_map<Input::_status_type, Output::_status_type> status_map({
    {Input::SOLID_OFF, Output::SOLID_OFF},
    {Input::SOLID_ON, Output::SOLID_ON},
    {Input::FLASHING, Output::FLASHING}
  });
  // clang-format on

  Output output;
  output.color = at_or(color_map, input.color, Output::UNKNOWN);
  output.shape = at_or(shape_map, input.shape, Output::UNKNOWN);
  output.color = at_or(status_map, input.status, Output::UNKNOWN);
  output.confidence = input.confidence;
  return output;
}

}  // namespace utils

TrafficLightSelector::TrafficLightSelector(const rclcpp::NodeOptions & options)
: Node("traffic_light_selector", options)
{
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/sub/vector_map", rclcpp::QoS(1).transient_local(),
    std::bind(&TrafficLightSelector::on_map, this, std::placeholders::_1));
  sub_lights_ = create_subscription<TrafficLightArray>(
    "~/sub/traffic_lights", rclcpp::QoS(1),
    std::bind(&TrafficLightSelector::on_lights, this, std::placeholders::_1));
  pub_signals_ = create_publisher<TrafficSignalArray>("~/pub/traffic_signals", rclcpp::QoS(1));
}

void TrafficLightSelector::on_map(const LaneletMapBin::ConstSharedPtr msg)
{
  const auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, map);

  const auto signals = lanelet::filter_traffic_signals(map);
  light_to_signal_.clear();
  for (const auto & signal : signals) {
    for (const auto & light : signal->trafficLights()) {
      light_to_signal_[light.id()] = signal->id();
    }
  }
}

void TrafficLightSelector::on_lights(const TrafficLightArray::ConstSharedPtr msg)
{
  using TrafficSignal = autoware_perception_msgs::msg::TrafficSignal;
  using TrafficLightElement = autoware_perception_msgs::msg::TrafficLightElement;

  std::unordered_map<lanelet::Id, std::vector<TrafficLightElement>> elements;
  for (const auto & light : msg->signals) {
    const auto id = light_to_signal_[light.map_primitive_id];
    for (const auto & element : light.lights) {
      elements[id].push_back(utils::convert(element));
    }
  }

  TrafficSignalArray array;
  array.stamp = now();
  for (const auto & [id, elements] : elements) {
    TrafficSignal signal;
    signal.traffic_signal_id = id;
    signal.elements = elements;
    array.signals.push_back(signal);
  }
  pub_signals_->publish(array);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TrafficLightSelector)
