// Copyright 2020 Tier IV, Inc.
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

#ifndef TRAFFIC_LIGHT_MAP_VISUALIZER__NODE_HPP_
#define TRAFFIC_LIGHT_MAP_VISUALIZER__NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "autoware_lanelet2_msgs/msg/map_bin.hpp"
#include "autoware_perception_msgs/msg/traffic_light_state_array.hpp"
#include "lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"
#include "lanelet2_extension/utility/query.hpp"

namespace traffic_light
{
class TrafficLightMapVisualizerNode : public rclcpp::Node
{
public:
  TrafficLightMapVisualizerNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options);
  ~TrafficLightMapVisualizerNode() = default;
  void trafficLightStatesCallback(
    const autoware_perception_msgs::msg::TrafficLightStateArray::ConstSharedPtr
    input_tl_states_msg);
  void binMapCallback(
    const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr input_map_msg);

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr light_marker_pub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrafficLightStateArray>::SharedPtr
    tl_state_sub_;
  rclcpp::Subscription<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr vector_map_sub_;

  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems_;
};

}  // namespace traffic_light

#endif  // TRAFFIC_LIGHT_MAP_VISUALIZER__NODE_HPP_
