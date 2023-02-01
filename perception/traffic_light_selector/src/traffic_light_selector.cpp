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
  for (const auto & light : msg->signals) {
    RCLCPP_INFO_STREAM(get_logger(), "ID: " << light.map_primitive_id);
    for (const auto & e : light.lights) {
      RCLCPP_INFO_STREAM(
        get_logger(), "  - " << (int)e.color << " " << (int)e.shape << " " << e.confidence);
    }
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TrafficLightSelector)
