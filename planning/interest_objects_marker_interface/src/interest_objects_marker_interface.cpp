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

#include "interest_objects_marker_interface/interest_objects_marker_interface.hpp"
namespace
{
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using interest_objects_marker_interface::ObjectStatus;
using visualization_msgs::msg::Marker;

using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;

Marker createArrowMarker(const size_t & id, const ObjectStatus & status, const std::string & name)
{
  const float r = status.safe ? 0.0f : 0.75f;
  const float g = status.safe ? 0.75f : 0.0f;
  const float b = 0.0f;

  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), name, id, Marker::ARROW,
    createMarkerScale(0.25, 0.5, 0.5), createMarkerColor(r, g, b, 0.999));

  Point src, dst;
  src = status.pose.position;
  src.z += status.height + 1.0;
  dst = status.pose.position;
  dst.z += status.height;

  marker.points.push_back(src);
  marker.points.push_back(dst);

  return marker;
}
}  // namespace

namespace interest_objects_marker_interface
{

InterestObjectsMarkerInterface::InterestObjectsMarkerInterface(
  rclcpp::Node * node, const std::string & name)
: name_{name}
{
  // Publisher
  pub_marker_ = node->create_publisher<MarkerArray>(topic_namespace_ + "/" + name, 1);
}

void InterestObjectsMarkerInterface::insertObjectStatus(
  const Pose & pose, const double obj_height, const bool safe)
{
  ObjectStatus status;
  status.pose = pose;
  status.height = obj_height;
  status.safe = safe;

  obj_status_array_.push_back(status);
}

void InterestObjectsMarkerInterface::publishMarkerArray()
{
  MarkerArray marker_array;
  for (size_t i = 0; i < obj_status_array_.size(); ++i) {
    const auto obj = obj_status_array_.at(i);
    const Marker marker = createArrowMarker(i, obj, name_);
    marker_array.markers.push_back(marker);
  }

  pub_marker_->publish(marker_array);
  obj_status_array_.clear();
}

}  // namespace interest_objects_marker_interface
