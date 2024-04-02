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

#ifndef TENSORRT_MTR__NODE_HPP_
#define TENSORRT_MTR__NODE_HPP_

#include "tensorrt_mtr/polyline.hpp"
#include "tensorrt_mtr/trt_mtr.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <fstream>
#include <map>
#include <memory>
#include <string>

namespace trt_mtr
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_perception_msgs::msg::TrackedObjects;
using autoware_perception_msgs::msg::PredictedObjects;

class PolylineTypeMap
{
public:
  explicit PolylineTypeMap(rclcpp::Node * node)
  {
    const auto filepath = node->declare_parameter<std::string>("polyline.label_file");
    std::ifstream file(filepath);
    if (!file.is_open()) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not open polyline label file: " << filepath);
      rclcpp::shutdown();
    }

    int label_index = 0;
    std::string label;
    while (getline(file, label)) {
      std::transform(
        label.begin(), label.end(), label.begin(), [](auto c) { return std::toupper(c); });
      label_map_.insert({label_index, label});
      ++label_index;
    }
  }

  const size_t & getTypeID(const & std::string & type) const { return label_map_.at(type); }

private:
  std::map<size_t, std::string> label_map_;
};  // class PolylineTypeMap

class MTRNode : public rclcpp::Node
{
public:
  explicit MTRNode(const rclcpp::NodeOptions & node_options);

private:
  /**
   * @brief Main callback being invoked when the tracked objects topic is subscribed.
   *
   * @param object_msg
   */
  void callback(const TrackedObjects::ConstSharedPtr object_msg);

  /**
   * @brief Callback being invoked when the HD map topic is subscribed.
   *
   * @param map_msg
   */
  void onMap(const HADMapBin::ConstSharedPtr map_msg);

  /**
   * @brief Converts lanelet2 to polylines.
   *
   * @return true
   */
  bool convertLaneletToPolyline();

  /**
   * @brief Appends new states to history and remove old data.
   *
   * @param current_time
   */
  void updateAgentHistory(const float current_time);

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;

  std::unique_ptr<MtrConfig> config_ptr_;
  PolylineTypeMap polyline_type_map_;
  PolylineData polylines_;
};  // class MTRNode
}  // namespace trt_mtr
#endif  // TENSORRT_MTR__NODE_HPP_
