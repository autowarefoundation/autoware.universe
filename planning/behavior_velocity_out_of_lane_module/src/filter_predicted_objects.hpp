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

#ifndef FILTER_PREDICTED_OBJECTS_HPP_
#define FILTER_PREDICTED_OBJECTS_HPP_

#include "types.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <traffic_light_utils/traffic_light_utils.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

// for writing the svg file
#include <fstream>
#include <iostream>
// for the geometry types
#include <tier4_autoware_utils/geometry/geometry.hpp>
// for the svg mapper
#include <boost/geometry/io/svg/svg_mapper.hpp>
#include <boost/geometry/io/svg/write.hpp>

namespace behavior_velocity_planner::out_of_lane
{
void cut_predicted_path_beyond_line(
  autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const lanelet::BasicLineString2d & stop_line)
{
  lanelet::BasicSegment2d path_segment;
  path_segment.first.x() = predicted_path.path.front().position.x;
  path_segment.first.y() = predicted_path.path.front().position.y;
  for (auto i = 1; i < predicted_path.path.size(); ++i) {
    path_segment.second.x() = predicted_path.path[i].position.x;
    path_segment.second.y() = predicted_path.path[i].position.y;
    if (boost::geometry::intersects(stop_line, path_segment)) {
      predicted_path.path.resize(i - std::min(i, 10));
      return;
    }
    path_segment.first = path_segment.second;
  }
}

std::optional<const lanelet::BasicLineString2d> find_next_stop_line(
  const autoware_auto_perception_msgs::msg::PredictedPath & path, const PlannerData & planner_data,
  const double max_length)
{
  lanelet::ConstLanelets lanelets;
  lanelet::BasicPoint2d query_point;
  for (const auto & p : path.path) {
    query_point.x() = p.position.x;
    query_point.y() = p.position.y;
    const auto results = lanelet::geometry::findWithin2d(
      planner_data.route_handler_->getLaneletMapPtr()->laneletLayer, query_point);
    for (const auto & r : results) lanelets.push_back(r.second);
  }
  for (const auto & ll : lanelets) {
    for (const auto & element : ll.regulatoryElementsAs<lanelet::TrafficLight>()) {
      const auto traffic_signal_stamped = planner_data.getTrafficSignal(element->id());
      if (
        traffic_signal_stamped.has_value() && element->stopLine().has_value() &&
        traffic_light_utils::isTrafficSignalStop(ll, traffic_signal_stamped.value().signal)) {
        lanelet::BasicLineString2d stop_line;
        for (const auto & p : *(element->stopLine())) stop_line.emplace_back(p.x(), p.y());
        return stop_line;
      }
    }
  }
  return std::nullopt;
}

/// @brief cut predicted path beyond stop lines of red lights
/// @param [inout] predicted_path predicted path to cut
/// @param [in] planner_data planner data to get the map and traffic light information
void cut_predicted_path_beyond_red_lights(
  autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const PlannerData & planner_data)
{
  // Declare a stream and an SVG mapper
  std::ofstream svg("/home/mclement/Pictures/image.svg");  // /!\ CHANGE PATH
  boost::geometry::svg_mapper<tier4_autoware_utils::Point2d> mapper(svg, 400, 400);
  const lanelet::BasicPoint2d first_point = {
    predicted_path.path.front().position.x, predicted_path.path.front().position.y};
  const auto stop_line = find_next_stop_line(
    predicted_path, planner_data,
    motion_utils::calcSignedArcLength(predicted_path.path, 0, predicted_path.path.size() - 1));
  lanelet::BasicLineString2d path_ls;
  for (const auto & p : predicted_path.path) path_ls.emplace_back(p.position.x, p.position.y);
  if (stop_line) cut_predicted_path_beyond_line(predicted_path, *stop_line);
  lanelet::BasicLineString2d cut_path_ls;
  for (const auto & p : predicted_path.path) cut_path_ls.emplace_back(p.position.x, p.position.y);
  mapper.add(cut_path_ls);
  mapper.map(first_point, "opacity:1.0;fill:red;stroke:green;stroke-width:2", 2);
  mapper.map(path_ls, "opacity:0.3;fill:black;stroke:black;stroke-width:2");
  mapper.map(cut_path_ls, "opacity:0.3;fill:red;stroke:red;stroke-width:2");
}

/// @brief filter predicted objects and their predicted paths
/// @param [in] planner_data planner data
/// @param [in] ego_data ego data
/// @param [in] params parameters
/// @return filtered predicted objects
autoware_auto_perception_msgs::msg::PredictedObjects filter_predicted_objects(
  const PlannerData & planner_data, const EgoData & ego_data, const PlannerParam & params)
{
  autoware_auto_perception_msgs::msg::PredictedObjects filtered_objects;
  filtered_objects.header = planner_data.predicted_objects->header;
  for (const auto & object : planner_data.predicted_objects->objects) {
    const auto is_pedestrian =
      std::find_if(object.classification.begin(), object.classification.end(), [](const auto & c) {
        return c.label == autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
      }) != object.classification.end();
    if (is_pedestrian) continue;

    auto filtered_object = object;
    const auto is_invalid_predicted_path = [&](const auto & predicted_path) {
      const auto is_low_confidence = predicted_path.confidence < params.objects_min_confidence;
      const auto no_overlap_path = motion_utils::removeOverlapPoints(predicted_path.path);
      if (no_overlap_path.size() <= 1) return true;
      const auto lat_offset_to_current_ego =
        std::abs(motion_utils::calcLateralOffset(no_overlap_path, ego_data.pose.position));
      const auto is_crossing_ego =
        lat_offset_to_current_ego <=
        object.shape.dimensions.y / 2.0 + std::max(
                                            params.left_offset + params.extra_left_offset,
                                            params.right_offset + params.extra_right_offset);
      return is_low_confidence || is_crossing_ego;
    };
    if (params.objects_use_predicted_paths) {
      auto & predicted_paths = filtered_object.kinematics.predicted_paths;
      const auto new_end =
        std::remove_if(predicted_paths.begin(), predicted_paths.end(), is_invalid_predicted_path);
      predicted_paths.erase(new_end, predicted_paths.end());
      if (true || params.objects_cut_predicted_paths_beyond_red_lights)
        for (auto & predicted_path : predicted_paths)
          cut_predicted_path_beyond_red_lights(predicted_path, planner_data);
      predicted_paths.erase(
        std::remove_if(predicted_paths.begin(), predicted_paths.end(), is_invalid_predicted_path),
        predicted_paths.end());
    }

    if (!params.objects_use_predicted_paths || !filtered_object.kinematics.predicted_paths.empty())
      filtered_objects.objects.push_back(filtered_object);
  }
  return filtered_objects;
}

}  // namespace behavior_velocity_planner::out_of_lane

#endif  // FILTER_PREDICTED_OBJECTS_HPP_
