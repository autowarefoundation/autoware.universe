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

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/constants.hpp>
#include <tier4_autoware_utils/math/trigonometry.hpp>

#include <std_msgs/msg/color_rgba.hpp>

namespace
{
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Transform;
using geometry_msgs::msg::Vector3;
using interest_objects_marker_interface::ObjectStatus;
using std_msgs::msg::ColorRGBA;
using tier4_autoware_utils::Polygon2d;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

using tier4_autoware_utils::calcAzimuthAngle;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_autoware_utils::createTranslation;

using tier4_autoware_utils::cos;
using tier4_autoware_utils::getRPY;
using tier4_autoware_utils::pi;
using tier4_autoware_utils::sin;

ColorRGBA getColor(const bool safe, const float alpha)
{
  const float r = safe ? 0.0f : 214.0 / 255.0;
  const float g = safe ? 211.0 / 255.0 : 0.0f;
  const float b = safe ? 141.0 / 255.0 : 77.0 / 255.0;
  return createMarkerColor(r, g, b, alpha);
}

Marker createArrowMarker(const size_t & id, const ObjectStatus & status, const std::string & name)
{
  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), name, id, Marker::ARROW,
    createMarkerScale(0.25, 0.5, 0.5), getColor(status.safe, 0.95f));

  Point src, dst;
  src = status.pose.position;
  src.z += status.height + 1.0;
  dst = status.pose.position;
  dst.z += status.height;

  marker.points.push_back(src);
  marker.points.push_back(dst);

  return marker;
}

Marker createCircleMarker(
  const size_t & id, const ObjectStatus & status, const std::string & name, const double radius)
{
  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), name, id, Marker::LINE_STRIP,
    createMarkerScale(0.1, 0.0, 0.0), getColor(status.safe, 0.951f));

  constexpr size_t num_points = 20;
  std::vector<Point> points;
  points.resize(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(num_points);
    const double theta = 2 * pi * ratio;
    points.at(i).x = status.pose.position.x + radius * cos(theta);
    points.at(i).y = status.pose.position.y + radius * sin(theta);
    points.at(i).z = status.pose.position.z + status.height + 0.75;
    marker.points.push_back(points.at(i));
  }
  marker.points.push_back(points.front());

  return marker;
}

MarkerArray createTargetMarker(
  const size_t & id, const ObjectStatus & status, const std::string & name)
{
  MarkerArray marker_array;
  marker_array.markers.push_back(createArrowMarker(id, status, name + "_arrow"));
  marker_array.markers.push_back(createCircleMarker(id, status, name + "_circle1", 0.5));
  marker_array.markers.push_back(createCircleMarker(id, status, name + "_circle2", 0.75));

  return marker_array;
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
  const Pose & pose, const Polygon2d & polygon, double obj_height, const bool safe)
{
  ObjectStatus status;
  status.pose = pose;
  status.polygon = polygon;
  status.height = obj_height;
  status.safe = safe;

  obj_status_array_.push_back(status);
}

void InterestObjectsMarkerInterface::publishMarkerArray()
{
  MarkerArray marker_array;
  for (size_t i = 0; i < obj_status_array_.size(); ++i) {
    const auto obj = obj_status_array_.at(i);
    const MarkerArray target_marker = createTargetMarker(i, obj, name_);
    marker_array.markers.insert(
      marker_array.markers.end(), target_marker.markers.begin(), target_marker.markers.end());
  }

  pub_marker_->publish(marker_array);
  obj_status_array_.clear();
}

}  // namespace interest_objects_marker_interface
