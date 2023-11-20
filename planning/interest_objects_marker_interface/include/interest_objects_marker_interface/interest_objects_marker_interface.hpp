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
#include "interest_objects_marker_interface/coloring.hpp"
#include "interest_objects_marker_interface/marker_data.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace interest_objects_marker_interface
{
using autoware_auto_perception_msgs::msg::Shape;
using geometry_msgs::msg::Pose;
using std_msgs::msg::ColorRGBA;
using tier4_autoware_utils::Polygon2d;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class InterestObjectsMarkerInterface
{
public:
  InterestObjectsMarkerInterface(rclcpp::Node * node, const std::string & name);
  void insertObjectData(const Pose & pose, const Shape & shape, const ColorName & color_name);
  void insertObjectDataWithColor(const Pose & pose, const Shape & shape, const ColorRGBA & color);
  void publishMarkerArray();
  void setHeightOffset(const double offset);
  static ColorRGBA getColor(const ColorName & color_name, const float alpha = 0.99f);

private:
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;

  double height_offset_{0.0};
  std::vector<ObjectMarkerData> obj_marker_data_array_;

  std::string name_;
  std::string topic_namespace_ = "/planning/debug/interest_objects_marker";
};

}  // namespace interest_objects_marker_interface

#endif  // INTEREST_OBJECTS_MARKER_INTERFACE__INTEREST_OBJECTS_MARKER_INTERFACE_HPP_
