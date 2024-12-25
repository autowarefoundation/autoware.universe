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

#include "traffic_light_signals_merger_node.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::traffic_light
{

TrafficLightSignalsMergerNode::TrafficLightSignalsMergerNode(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node("traffic_light_signals_merger_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  car_signal_sub_(this, "input/car_signals", rclcpp::QoS{1}.get_rmw_qos_profile()),
  pedestrian_signal_sub_(this, "input/pedestrian_signals", rclcpp::QoS{1}.get_rmw_qos_profile()),
  expected_rois_sub_(this, "input/expect_rois", rclcpp::QoS{1}.get_rmw_qos_profile()),
  sync_(SyncPolicy(10), car_signal_sub_, pedestrian_signal_sub_, expected_rois_sub_)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  sync_.registerCallback(
    std::bind(&TrafficLightSignalsMergerNode::signalsCallback, this, _1, _2, _3));
  pub_traffic_light_signals_ =
    create_publisher<TrafficLightArray>("output/traffic_light_signals", rclcpp::QoS{1});
}

void TrafficLightSignalsMergerNode::signalsCallback(
  const TrafficLightArray::ConstSharedPtr & car_signals_msg,
  const TrafficLightArray::ConstSharedPtr & pedestrian_signals_msg,
  const TrafficLightRoiArray::ConstSharedPtr & expected_rois_msg)
{
  std::map<int, TrafficLightRoi> expected_rois_map;
  for (const auto & roi : expected_rois_msg->rois) {
    expected_rois_map[roi.traffic_light_id].traffic_light_id = roi.traffic_light_id;
    expected_rois_map[roi.traffic_light_id].roi = roi.roi;
    expected_rois_map[roi.traffic_light_id].traffic_light_type = roi.traffic_light_type;
  }

  TrafficLightArray output;
  output.header = car_signals_msg->header;
  output.signals.insert(
    output.signals.end(), car_signals_msg->signals.begin(), car_signals_msg->signals.end());
  output.signals.insert(
    output.signals.end(), pedestrian_signals_msg->signals.begin(),
    pedestrian_signals_msg->signals.end());
  for (auto & signal : output.signals) {
    // remove expected_rois which are already in signals
    if (expected_rois_map.find(signal.traffic_light_id) != expected_rois_map.end()) {
      expected_rois_map.erase(signal.traffic_light_id);
    }
  }
  for (const auto & roi : expected_rois_map) {
    TrafficLight signal;
    signal.traffic_light_id = roi.first;
    signal.traffic_light_type = roi.second.traffic_light_type;
    signal.elements.resize(1);
    signal.elements[0].shape = TrafficLightElement::UNKNOWN;
    signal.elements[0].color = TrafficLightElement::UNKNOWN;
    signal.elements[0].confidence = 1.0;
    output.signals.push_back(signal);
  }
  pub_traffic_light_signals_->publish(output);
}

}  // namespace autoware::traffic_light

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightSignalsMergerNode)
