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

using OldData = autoware_auto_perception_msgs::msg::TrafficSignal;
using OldElem = autoware_auto_perception_msgs::msg::TrafficLight;
using NewData = autoware_perception_msgs::msg::TrafficLight;
using NewElem = autoware_perception_msgs::msg::TrafficLightElement;

template <class K, class V>
V at_or(const std::unordered_map<K, V> & map, const K & key, const V & value)
{
  return map.count(key) ? map.at(key) : value;
}

NewElem convert(const OldElem & input)
{
  // clang-format off
  static const std::unordered_map<OldElem::_color_type, NewElem::_color_type> color_map({
    {OldElem::RED, NewElem::RED},
    {OldElem::AMBER, NewElem::AMBER},
    {OldElem::GREEN, NewElem::GREEN},
    {OldElem::WHITE, NewElem::WHITE}
  });
  static const std::unordered_map<OldElem::_shape_type, NewElem::_shape_type> shape_map({
    {OldElem::CIRCLE, NewElem::CIRCLE},
    {OldElem::LEFT_ARROW, NewElem::LEFT_ARROW},
    {OldElem::RIGHT_ARROW, NewElem::RIGHT_ARROW},
    {OldElem::UP_ARROW, NewElem::UP_ARROW},
    {OldElem::DOWN_ARROW, NewElem::DOWN_ARROW},
    {OldElem::DOWN_LEFT_ARROW, NewElem::DOWN_LEFT_ARROW},
    {OldElem::DOWN_RIGHT_ARROW, NewElem::DOWN_RIGHT_ARROW},
    {OldElem::CROSS, NewElem::CROSS}
  });
  static const std::unordered_map<OldElem::_status_type, NewElem::_status_type> status_map({
    {OldElem::SOLID_OFF, NewElem::SOLID_OFF},
    {OldElem::SOLID_ON, NewElem::SOLID_ON},
    {OldElem::FLASHING, NewElem::FLASHING}
  });
  // clang-format on

  NewElem output;
  output.color = at_or(color_map, input.color, NewElem::UNKNOWN);
  output.shape = at_or(shape_map, input.shape, NewElem::UNKNOWN);
  output.color = at_or(status_map, input.status, NewElem::UNKNOWN);
  output.confidence = input.confidence;
  return output;
}

NewData convert(const OldData & input)
{
  NewData output;
  output.traffic_light_id = input.map_primitive_id;
  for (const auto & light : input.lights) {
    output.elements.push_back(convert(light));
  }
  return output;
}

}  // namespace utils

TrafficLightSelector::TrafficLightSelector(const rclcpp::NodeOptions & options)
: Node("traffic_light_selector", options)
{
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/sub/vector_map", rclcpp::QoS(1).transient_local(),
    std::bind(&TrafficLightSelector::on_map, this, std::placeholders::_1));
  sub_v2x_ = create_subscription<TrafficSignalArray>(
    "~/sub/traffic_signals", rclcpp::QoS(1),
    std::bind(&TrafficLightSelector::on_v2x, this, std::placeholders::_1));
  sub_perception_ = create_subscription<TrafficLightArray2>(
    "~/sub/traffic_lights", rclcpp::QoS(1),
    std::bind(&TrafficLightSelector::on_perception, this, std::placeholders::_1));

  pub_ = create_publisher<TrafficSignalArray>("~/pub/traffic_signals", rclcpp::QoS(1));

  const auto rate = rclcpp::Rate(1.0);
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });
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

void TrafficLightSelector::on_v2x(const TrafficSignalArray::ConstSharedPtr msg) { data_v2x_ = msg; }

void TrafficLightSelector::on_perception(const TrafficLightArray2::ConstSharedPtr msg)
{
  const auto data = std::make_shared<TrafficLightArray>();
  data->stamp = msg->header.stamp;
  data->lights.reserve(msg->signals.size());
  for (const auto & light : msg->signals) {
    data->lights.push_back(utils::convert(light));
  }
  data_perception_ = data;
}

void TrafficLightSelector::on_timer()
{
  /*
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
  */
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TrafficLightSelector)
