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

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>

namespace trt_mtr
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_perception_msgs::msg::TrackedObjects;
using autoware_perception_msgs::msg::PredictedObjects;

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
   */
  void convertLaneletToPolyline();

  /**
   * @brief Appends new states to history and remove old data.
   *
   * @param current_time
   */
  void updateAgentHistory(const float current_time);
};  // class MTRNode
}  // namespace trt_mtr
#endif  // TENSORRT_MTR__NODE_HPP_
