/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 */

#ifndef LANELET2_EXTENSION_VISUALIZATION_VISUALIZATION_H
#define LANELET2_EXTENSION_VISUALIZATION_VISUALIZATION_H

#include "geometry_msgs/msg/polygon.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp/rclcpp.hpp"

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_core/primitives/Lanelet.h"

#include "lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"
#include "lanelet2_extension/utility/query.hpp"

#include <string>
#include <vector>

namespace lanelet
{
namespace visualization
{
/**
 * [lanelet2Triangle converts lanelet into vector of triangles. Used for
 * triangulation]
 * @param ll        [input lanelet]
 * @param triangles [array of polygon message, each containing 3 vertices]
 */
void lanelet2Triangle(
  const lanelet::ConstLanelet & ll, std::vector<geometry_msgs::msg::Polygon> * triangles);

/**
 * [polygon2Triangle converts polygon into vector of triangles]
 * @param polygon   [input polygon]
 * @param triangles [array of polygon message, each containing 3 vertices]
 */
void polygon2Triangle(
  const geometry_msgs::msg::Polygon & polygon, std::vector<geometry_msgs::msg::Polygon> * triangles);

/**
 * [lanelet2Polygon converts lanelet into a polygon]
 * @param ll      [input lanelet]
 * @param polygon [polygon message containing shape of the input lanelet.]
 */
void lanelet2Polygon(const lanelet::ConstLanelet & ll, geometry_msgs::msg::Polygon * polygon);

/**
 * [lineString2Marker creates marker to visualize shape of linestring]
 * @param ls         [input linestring]
 * @param line_strip [output marker message]
 * @param frame_id   [frame id of the marker]
 * @param ns         [namespace of the marker]
 * @param c          [color of the marker]
 * @param lss        [thickness of the marker]
 */
void lineString2Marker(
  const lanelet::ConstLineString3d ls, visualization_msgs::msg::Marker * line_strip,
  const std::string frame_id, const std::string ns, const std_msgs::msg::ColorRGBA c,
  const float lss = 0.1);
/**
 * [trafficLight2TriangleMarker creates marker to visualize shape of traffic
 * lights]
 * @param ls       [linestring that represents traffic light shape]
 * @param marker   [created marker]
 * @param ns       [namespace of the marker]
 * @param cl       [color of the marker]
 * @param duration [lifetime of the marker]
 */
void trafficLight2TriangleMarker(
  const lanelet::ConstLineString3d ls, visualization_msgs::msg::Marker * marker, const std::string ns,
  const std_msgs::msg::ColorRGBA cl, const rclcpp::Duration duration = rclcpp::Duration(0, 0),
  const double scale = 1.0);

/**
 * [laneletsBoundaryAsMarkerArray create marker array to visualize shape of
 * boundaries of lanelets]
 * @param  lanelets       [input lanelets]
 * @param  c              [color of the boundary]
 * @param  viz_centerline [flag to visuazlize centerline or not]
 * @return                [created marker array]
 */
visualization_msgs::msg::MarkerArray laneletsBoundaryAsMarkerArray(
  const lanelet::ConstLanelets & lanelets, const std_msgs::msg::ColorRGBA c, const bool viz_centerline);
/**
 * [laneletsAsTriangleMarkerArray create marker array to visualize shape of the
 * lanelet]
 * @param  ns       [namespace of the marker]
 * @param  lanelets [input lanelets]
 * @param  c        [color of the marker]
 * @return          [created marker]
 */
visualization_msgs::msg::MarkerArray laneletsAsTriangleMarkerArray(
  const std::string ns, const lanelet::ConstLanelets & lanelets, const std_msgs::msg::ColorRGBA c);

/**
 * [laneletDirectionAsMarkerArray create marker array to visualize direction of
 * the lanelet]
 * @param  lanelets [input lanelets]
 * @return          [created marker array]
 */
visualization_msgs::msg::MarkerArray laneletDirectionAsMarkerArray(
  const lanelet::ConstLanelets lanelets);

/**
 * [lineStringsAsMarkerArray creates marker array to visualize shape of
 * linestrings]
 * @param  line_strings [input linestrings]
 * @param  name_space   [namespace of the marker]
 * @param  c            [color of the marker]
 * @param  lss          [thickness of the marker]
 * @return              [created marker array]
 */
visualization_msgs::msg::MarkerArray lineStringsAsMarkerArray(
  const std::vector<lanelet::ConstLineString3d> line_strings, const std::string name_space,
  const std_msgs::msg::ColorRGBA c, const double lss);

/**
 * [autowareTrafficLightsAsMarkerArray creates marker array to visualize traffic
 * lights]
 * @param  tl_reg_elems [traffic light regulatory elements]
 * @param  c            [color of the marker]
 * @param  duration     [lifetime of the marker]
 * @return              [created marker array]
 */
visualization_msgs::msg::MarkerArray autowareTrafficLightsAsMarkerArray(
  const std::vector<lanelet::AutowareTrafficLightConstPtr> tl_reg_elems,
  const std_msgs::msg::ColorRGBA c, const rclcpp::Duration duration = rclcpp::Duration(0, 0),
  const double scale = 1.0);

/**
 * [trafficLightsAsTriangleMarkerArray creates marker array to visualize shape
 * of traffic lights]
 * @param  tl_reg_elems [traffic light regulatory elements]
 * @param  c            [color of the marker]
 * @param  duration     [lifetime of the marker]
 * @return              [created marker array]
 */
visualization_msgs::msg::MarkerArray trafficLightsAsTriangleMarkerArray(
  const std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems, const std_msgs::msg::ColorRGBA c,
  const rclcpp::Duration duration = rclcpp::Duration(0, 0), const double scale = 1.0);

/**
 * [detectionAreasAsMarkerArray creates marker array to visualize detection areas]
 * @param  da_reg_elems [detection area regulatory elements]
 * @param  c            [color of the marker]
 * @param  duration     [lifetime of the marker]
 */
visualization_msgs::msg::MarkerArray detectionAreasAsMarkerArray(
  const std::vector<lanelet::DetectionAreaConstPtr> & da_reg_elems, const std_msgs::msg::ColorRGBA c,
  const rclcpp::Duration duration = rclcpp::Duration(0, 0));

visualization_msgs::msg::MarkerArray parkingLotsAsMarkerArray(
  const lanelet::ConstPolygons3d & parking_lots, const std_msgs::msg::ColorRGBA & c);
visualization_msgs::msg::MarkerArray parkingSpacesAsMarkerArray(
  const lanelet::ConstLineStrings3d & parking_spaces, const std_msgs::msg::ColorRGBA & c);

}  // namespace visualization
}  // namespace lanelet

#endif  // LANELET2_EXTENSION_VISUALIZATION_VISUALIZATION_H
