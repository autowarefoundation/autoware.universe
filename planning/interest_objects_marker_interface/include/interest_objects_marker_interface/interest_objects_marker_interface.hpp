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

#ifndef INTEREST_OBJECTS_MARKER_INTERFACE__INTEREST_OBJECTS_MARKER_INTERFACE_HPP_
#define INTEREST_OBJECTS_MARKER_INTERFACE__INTEREST_OBJECTS_MARKER_INTERFACE_HPP_
#include "rclcpp/rclcpp.hpp"

#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace interest_objects_marker_interface
{
using geometry_msgs::msg::Pose;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

struct ObjectStatus
{
  Pose pose{};
  double height{0.0};
  bool safe{false};
};

class InterestObjectsMarkerInterface
{
public:
  InterestObjectsMarkerInterface(rclcpp::Node * node, const std::string & name);
  void insertObjectStatus(const Pose & pose, const double obj_height, const bool safe);
  void publishMarkerArray();

private:
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;

  std::vector<ObjectStatus> obj_status_array_;

  std::string name_;
  std::string topic_namespace_ = "/planning/interest_objects_marker";

  mutable std::mutex mutex_;
};

}  // namespace interest_objects_marker_interface

#endif  // INTEREST_OBJECTS_MARKER_INTERFACE__INTEREST_OBJECTS_MARKER_INTERFACE_HPP_
