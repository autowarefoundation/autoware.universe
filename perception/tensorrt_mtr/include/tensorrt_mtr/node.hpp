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

#include "tensorrt_mtr/agent.hpp"
#include "tensorrt_mtr/polyline.hpp"
#include "tensorrt_mtr/trajectory.hpp"
#include "tensorrt_mtr/trt_mtr.hpp"

#include <object_recognition_utils/object_classification.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/transform_listener.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object_kinematics.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_path.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace trt_mtr
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjectKinematics;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;
using nav_msgs::msg::Odometry;

// TODO(ktro2828): use received ego size topic
// wheel_base: between front wheel center and rear wheel center [m]
// wheel_tread: between left wheel center and right wheel center [m]
// front_overhang: between front wheel center and vehicle front [m]
// rear_overhang: between rear wheel center and vehicle rear [m]
// left_overhang: between left wheel center and vehicle left [m]
// right_overhang: between right wheel center and vehicle right [m]
constexpr float EGO_LENGTH = 4.0f;
constexpr float EGO_WIDTH = 2.0f;
constexpr float EGO_HEIGHT = 1.0f;

class PolylineTypeMap
{
public:
  explicit PolylineTypeMap(rclcpp::Node * node)
  {
    const auto filepath = node->declare_parameter<std::string>("polyline_label_path");
    std::ifstream file(filepath);
    if (!file.is_open()) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not open polyline label file: " << filepath);
      rclcpp::shutdown();
    }

    int label_index = 0;
    std::string label;
    while (getline(file, label)) {
      label_map_.insert({label, label_index});
      ++label_index;
    }
  }

  // Return the ID of the corresponding label type. If specified type is not contained in map,
  // return `-1`.
  int getTypeID(const std::string & type) const
  {
    return label_map_.count(type) == 0 ? -1 : label_map_.at(type);
  }

private:
  std::map<std::string, size_t> label_map_;
};  // class PolylineTypeMap

class MTRNode : public rclcpp::Node
{
public:
  explicit MTRNode(const rclcpp::NodeOptions & node_options);

  // Object ID of the ego vehicle
  const std::string EGO_ID{"EGO"};

private:
  // Main callback being invoked when the tracked objects topic is subscribed.
  void callback(const TrackedObjects::ConstSharedPtr object_msg);

  // Callback being invoked when the HD map topic is subscribed.
  void onMap(const HADMapBin::ConstSharedPtr map_msg);

  // Callback being invoked when the Ego's odometry topic is subscribed.
  void onEgo(const Odometry::ConstSharedPtr ego_msg);

  // Convert Lanelet to `PolylineData`.
  bool convertLaneletToPolyline();

  // Remove ancient agent histories.
  void removeAncientAgentHistory(
    const float current_time, const TrackedObjects::ConstSharedPtr objects_msg);

  // Appends new states to history.
  void updateAgentHistory(
    const float current_time, const TrackedObjects::ConstSharedPtr objects_msg);

  // Extract ego state stored in the buffer which has the nearest timestamp from current timestamp.
  AgentState extractNearestEgo(const float current_time) const;

  // Extract target agents and return corresponding indices.
  // NOTE: Extract targets in order of proximity, closest first.
  std::vector<size_t> extractTargetAgent(const std::vector<AgentHistory> & histories);

  // Return the timestamps relative from the first element.Return the timestamps relative from the
  // first element.
  std::vector<float> getRelativeTimestamps() const;

  // Generate `PredictedObject` from `PredictedTrajectory`.
  PredictedObject generatePredictedObject(
    const TrackedObject & object, const PredictedTrajectory & trajectory);

  // ROS Publisher and Subscriber
  // TODO(ktro2828): add debug publisher
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects_;
  rclcpp::Subscription<TrackedObjects>::SharedPtr sub_objects_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_ego_;

  // Lanelet map pointers
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;

  // Agent history
  std::map<std::string, AgentHistory> agent_history_map_;
  std::map<std::string, TrackedObject> object_msg_map_;

  // Pose transform listener
  tier4_autoware_utils::TransformListener transform_listener_;

  // MTR parameters
  std::unique_ptr<MTRConfig> config_ptr_;
  std::unique_ptr<TrtMTR> model_ptr_;
  PolylineTypeMap polyline_type_map_;
  std::shared_ptr<PolylineData> polyline_ptr_;
  std::vector<std::pair<float, AgentState>> ego_states_;
  std::vector<float> timestamps_;
};  // class MTRNode
}  // namespace trt_mtr
#endif  // TENSORRT_MTR__NODE_HPP_
