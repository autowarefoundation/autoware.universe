// Copyright 2021 Tier IV, Inc.
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

#ifndef TRAFFIC_LIGHT_RECOGNITION_MARKER_PUBLISHER_HPP_
#define TRAFFIC_LIGHT_RECOGNITION_MARKER_PUBLISHER_HPP_

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <string>

namespace autoware::traffic_light_recognition_marker_publisher
{
class TrafficLightRecognitionMarkerPublisher : public rclcpp::Node
{
public:
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using TrafficSignalArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
  using TrafficLight = autoware_perception_msgs::msg::TrafficLightElement;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Pose = geometry_msgs::msg::Pose;

  explicit TrafficLightRecognitionMarkerPublisher(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_ptr_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr
    sub_tlr_ptr_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_ptr_;

  void onMap(const LaneletMapBin::ConstSharedPtr msg_ptr);
  void onTrafficSignalArray(const TrafficSignalArray::ConstSharedPtr msg_ptr);
  visualization_msgs::msg::Marker getTrafficLightMarker(
    const Pose & tl_pose, const uint8_t tl_color, const uint8_t tl_shape);

  std::string getTrafficLightString(const uint8_t tl_color, const uint8_t tl_shape);
  std_msgs::msg::ColorRGBA getTrafficLightColor(const uint8_t tl_color, const uint8_t tl_shape);

  std::map<int32_t, Pose> tl_position_map_;
  bool is_map_ready_ = false;
  int32_t marker_id = 0;
};
}  // namespace autoware::traffic_light_recognition_marker_publisher

#endif  // TRAFFIC_LIGHT_RECOGNITION_MARKER_PUBLISHER_HPP_
