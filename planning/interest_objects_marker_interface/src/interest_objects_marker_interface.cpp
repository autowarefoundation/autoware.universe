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
using tier4_autoware_utils::createQuaternion;
using tier4_autoware_utils::createTranslation;

using tier4_autoware_utils::cos;
using tier4_autoware_utils::getRPY;
using tier4_autoware_utils::pi;
using tier4_autoware_utils::sin;

ColorRGBA getColor(const bool safe)
{
  const float r = safe ? 0.0f : 214.0 / 255.0;
  const float g = safe ? 211.0 / 255.0 : 0.0f;
  const float b = safe ? 141.0 / 255.0 : 77.0 / 255.0;
  return createMarkerColor(r, g, b, 0.95);
}

double calcAzimuthAngleFromPolygon(const Polygon2d & polygon)
{
  const size_t polygon_size = polygon.outer().size();

  const auto polygon_from = polygon.outer().at(polygon_size - 2);
  Point p_from;
  p_from.x = polygon_from.x();
  p_from.y = polygon_from.y();
  p_from.z = 0.0;

  const auto polygon_to = polygon.outer().back();
  Point p_to;
  p_to.x = polygon_to.x();
  p_to.y = polygon_to.y();
  p_to.z = 0.0;

  return calcAzimuthAngle(p_from, p_to);
}

Marker createCircleMarker(
  const size_t & id, const ObjectStatus & status, const std::string & name, const double radius)
{
  if (status.polygon.outer().size() < 2) {
    return Marker{};
  }

  const auto azimuth = calcAzimuthAngleFromPolygon(status.polygon);

  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), name + "_circle", id, Marker::LINE_STRIP,
    createMarkerScale(0.25, 0.0, 0.0), getColor(status.safe));

  constexpr size_t num_points = 20;
  std::vector<Point> points;
  points.resize(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(num_points);
    const double theta = 2 * pi * ratio + azimuth;
    points.at(i).x = status.pose.position.x + radius * cos(theta);
    points.at(i).y = status.pose.position.y + radius * sin(theta);
    points.at(i).z = status.pose.position.z;
    marker.points.push_back(points.at(i));
  }
  marker.points.push_back(points.front());

  return marker;
}

MarkerArray createCrosshairMarker(
  const size_t & id, const ObjectStatus & status, const std::string & name, const double radius)
{
  MarkerArray marker_array;

  if (status.polygon.outer().size() < 2) {
    return marker_array;
  }

  const auto azimuth = calcAzimuthAngleFromPolygon(status.polygon);

  marker_array.markers.push_back(createCircleMarker(id, status, name, radius));

  for (size_t i = 0; i < 8; ++i) {
    const double scale = (i % 2 == 0) ? 0.25 : 0.1;
    const double radius_in = (i % 2 == 0) ? radius * 0.5 : radius * 0.75;
    const double theta = 2 * pi * static_cast<double>(i) / 8.0 + azimuth;

    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), name + "_cross_" + std::to_string(i), id,
      Marker::LINE_STRIP, createMarkerScale(scale, 0.0, 0.0), getColor(status.safe));

    Point src;
    src.x = status.pose.position.x + radius * cos(theta);
    src.y = status.pose.position.y + radius * sin(theta);
    src.z = status.pose.position.z;
    marker.points.push_back(src);

    Point dst;
    dst.x = status.pose.position.x + radius_in * cos(theta);
    dst.y = status.pose.position.y + radius_in * sin(theta);
    dst.z = status.pose.position.z;
    marker.points.push_back(dst);

    marker_array.markers.push_back(marker);
  }

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
    const double radius = 2.5;
    const MarkerArray crosshair_markers = createCrosshairMarker(i, obj, name_, radius);
    marker_array.markers.insert(
      marker_array.markers.end(), crosshair_markers.markers.begin(),
      crosshair_markers.markers.end());
  }

  pub_marker_->publish(marker_array);
  obj_status_array_.clear();
}

}  // namespace interest_objects_marker_interface
