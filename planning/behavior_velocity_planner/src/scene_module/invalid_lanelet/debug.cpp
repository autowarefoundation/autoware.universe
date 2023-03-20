// Copyright 2020 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include "scene_module/invalid_lanelet/scene.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <utilization/util.hpp>

#include <vector>

namespace behavior_velocity_planner
{
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;
using visualization_msgs::msg::Marker;

namespace
{
visualization_msgs::msg::MarkerArray createInvalidLaneletMarkers(
  const InvalidLaneletModule::DebugData & debug_data, const rclcpp::Time & now,
  const int64_t module_id)
{
  visualization_msgs::msg::MarkerArray msg;
  const int32_t uid = planning_utils::bitShift(module_id);

  // Invalid Lanelet polygon
  if (!debug_data.invalid_lanelet_polygon.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "invalid_lanelet polygon", uid, Marker::LINE_STRIP,
      createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));
    for (const auto & p : debug_data.invalid_lanelet_polygon) {
      marker.points.push_back(createPoint(p.x, p.y, p.z));
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Path - polygon intersection points
  {
    auto marker = createDefaultMarker(
      "map", now, "path_polygon intersection points", uid, Marker::POINTS,
      createMarkerScale(0.25, 0.25, 0.0), createMarkerColor(1.0, 0.0, 0.0, 0.999));
    const auto & p_first = debug_data.path_polygon_intersection.first_intersection_point;
    if (p_first) {
      marker.points.push_back(createPoint(p_first->x, p_first->y, p_first->z));
    }
    const auto & p_second = debug_data.path_polygon_intersection.second_intersection_point;
    if (p_second) {
      marker.points.push_back(createPoint(p_second->x, p_second->y, p_second->z));
    }
    if (!marker.points.empty()) msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace

// visualization_msgs::msg::MarkerArray InvalidLaneletModule::createVirtualWallMarkerArray()
// {
//   visualization_msgs::msg::MarkerArray wall_marker;

//   const auto now = this->clock_->now();

//   appendMarkerArray(
//     virtual_wall_marker_creator_->createStopVirtualWallMarker(
//       {debug_data_.stop_wall_pose}, "invalid_lanelet", now, module_id_),
//     &wall_marker, now);
//   return wall_marker;
// }

// visualization_msgs::msg::MarkerArray InvalidLaneletModule::createVirtualWallMarkerArray()
// {
//   const auto now = this->clock_->now();
//   visualization_msgs::msg::MarkerArray wall_marker;

//   const auto p_front = tier4_autoware_utils::calcOffsetPose(
//     debug_data_.stop_pose, debug_data_.base_link2front, 0.0, 0.0);
//     appendMarkerArray(
//       virtual_wall_marker_creator_->createStopVirtualWallMarker(
//         {p_front}, "invalid_lanelet", now, module_id_),
//       &wall_marker, now);

//   return wall_marker;
// }

// visualization_msgs::msg::MarkerArray InvalidLaneletModule::createVirtualWallMarkerArray()
// {
//   const auto now = this->clock_->now();
//   visualization_msgs::msg::MarkerArray wall_marker;

//   const auto p_front = tier4_autoware_utils::calcOffsetPose(
//     debug_data_.stop_pose, debug_data_.base_link2front, 0.0, 0.0);
//     appendMarkerArray(
//       virtual_wall_marker_creator_->createStopVirtualWallMarker(
//         {p_front}, "invalid_lanelet", now, module_id_),
//       &wall_marker, now);

//   return wall_marker;
// }

visualization_msgs::msg::MarkerArray InvalidLaneletModule::createVirtualWallMarkerArray()
{
  visualization_msgs::msg::MarkerArray wall_marker;

  const auto now = this->clock_->now();

  RCLCPP_INFO(logger_, "\n\n\n\n *******DEBUG ---- InvalidLaneletModule ************** :");

  if (
    (state_ == State::APPROACH) || (state_ == State::INSIDE_INVALID_LANELET) ||
    (state_ == State::STOPPED)) {
    // wall_marker = virtual_wall_marker_creator_->createStopVirtualWallMarker(
    //   {debug_data_.stop_pose}, "invalid_lanelet", now, module_id_);
    appendMarkerArray(
      virtual_wall_marker_creator_->createStopVirtualWallMarker(
        {debug_data_.stop_pose}, "invalid_lanelet", now, module_id_),
      &wall_marker, now);
  }
  // std::vector<visualization_msgs::msg::Marker>::iterator position =
  // std::find(wall_marker.markers.begin(), wall_marker.markers.end(),
  // [&](const visualization_msgs::msg::Marker & mrkr){return (mrkr.action ==
  // visualization_msgs::msg::Marker::DELETE);}); if (position != wall_marker.markers.end()) // ==
  // wall_marker.markers.end() means the element was not found
  //   wall_marker.markers.erase(position);

  RCLCPP_INFO_STREAM(logger_, "markerarraysize = " << wall_marker.markers.size());
  return wall_marker;
}

visualization_msgs::msg::MarkerArray InvalidLaneletModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;
  const auto now = this->clock_->now();

  appendMarkerArray(
    createInvalidLaneletMarkers(debug_data_, this->clock_->now(), module_id_), &debug_marker_array);

  return debug_marker_array;
}

}  // namespace behavior_velocity_planner
