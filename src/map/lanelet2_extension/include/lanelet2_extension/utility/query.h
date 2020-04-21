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

#ifndef LANELET2_EXTENSION_UTILITY_QUERY_H
#define LANELET2_EXTENSION_UTILITY_QUERY_H

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>
#include <lanelet2_extension/regulatory_elements/detection_area.h>

#include <string>
#include <vector>

namespace lanelet
{
using TrafficSignConstPtr = std::shared_ptr<const lanelet::TrafficSign>;
using TrafficLightConstPtr = std::shared_ptr<const lanelet::TrafficLight>;
using AutowareTrafficLightConstPtr = std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>;
using DetectionAreaConstPtr = std::shared_ptr<const lanelet::autoware::DetectionArea>;
}  // namespace lanelet

namespace lanelet
{
namespace utils
{
namespace query
{
/**
 * [laneletLayer converts laneletLayer into lanelet vector]
 * @param  ll_Map [input lanelet map]
 * @return        [all lanelets in the map]
 */
lanelet::ConstLanelets laneletLayer(const lanelet::LaneletMapConstPtr & ll_Map);

/**
 * [subtypeLanelets extracts Lanelet that has given subtype attribute]
 * @param  lls     [input lanelets with various subtypes]
 * @param  subtype [subtype of lanelets to be retrieved (e.g.
 * lanelet::AttributeValueString::Road)]
 * @return         [lanelets with given subtype]
 */
lanelet::ConstLanelets subtypeLanelets(const lanelet::ConstLanelets lls, const char subtype[]);

/**
 * [crosswalkLanelets extracts crosswalk lanelets]
 * @param  lls [input lanelets with various subtypes]
 * @return     [crosswalk lanelets]
 */
lanelet::ConstLanelets crosswalkLanelets(const lanelet::ConstLanelets lls);

/**
 * [roadLanelets extracts road lanelets]
 * @param  lls [input lanelets with subtype road]
 * @return     [road lanelets]
 */
lanelet::ConstLanelets roadLanelets(const lanelet::ConstLanelets lls);

/**
 * [trafficLights extracts Traffic Light regulatory element from lanelets]
 * @param lanelets [input lanelets]
 * @return         [traffic light that are associated with input lanenets]
 */
std::vector<lanelet::TrafficLightConstPtr> trafficLights(const lanelet::ConstLanelets lanelets);

/**
 * [autowareTrafficLights extracts Autoware Traffic Light regulatory element
 * from lanelets]
 * @param lanelets [input lanelets]
 * @return         [autoware traffic light that are associated with input
 * lanenets]
 */
std::vector<lanelet::AutowareTrafficLightConstPtr> autowareTrafficLights(
  const lanelet::ConstLanelets lanelets);

/**
 * [detectionAreas extracts Detection Area regulatory elements from lanelets]
 * @param lanelets [input lanelets]
 * @return         [detection areas that are associated with input lanelets]
 */
std::vector<lanelet::DetectionAreaConstPtr> detectionAreas(const lanelet::ConstLanelets & lanelets);

// query all parking lots in lanelet2 map
lanelet::ConstPolygons3d getAllParkingLots(const lanelet::LaneletMapConstPtr & lanelet_map_ptr);

// query all parking spaces in lanelet2 map
lanelet::ConstLineStrings3d getAllParkingSpaces(
  const lanelet::LaneletMapConstPtr & lanelet_map_ptr);

// query linked parking spaces from lanelet
lanelet::ConstLineStrings3d getLinkedParkingSpaces(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr & lanelet_map_ptr);
lanelet::ConstLineStrings3d getLinkedParkingSpaces(
  const lanelet::ConstLanelet & lanelet, const lanelet::ConstLineStrings3d & all_parking_spaces,
  const lanelet::ConstPolygons3d & all_parking_lots);
// query linked lanelets from parking space
bool getLinkedLanelet(
  const lanelet::ConstLineString3d & parking_space,
  const lanelet::ConstLanelets & all_road_lanelets,
  const lanelet::ConstPolygons3d & all_parking_lots, lanelet::ConstLanelet * linked_lanelet);
bool getLinkedLanelet(
  const lanelet::ConstLineString3d & parking_space,
  const lanelet::LaneletMapConstPtr & lanelet_map_ptr, lanelet::ConstLanelet * linked_lanelet);
lanelet::ConstLanelets getLinkedLanelets(
  const lanelet::ConstLineString3d & parking_space,
  const lanelet::ConstLanelets & all_road_lanelets,
  const lanelet::ConstPolygons3d & all_parking_lots);
lanelet::ConstLanelets getLinkedLanelets(
  const lanelet::ConstLineString3d & parking_space,
  const lanelet::LaneletMapConstPtr & lanelet_map_ptr);

// get linked parking lot from lanelet
bool getLinkedParkingLot(
  const lanelet::ConstLanelet & lanelet, const lanelet::ConstPolygons3d & all_parking_lots,
  lanelet::ConstPolygon3d * linked_parking_lot);
bool getLinkedParkingLot(
  const lanelet::ConstLineString3d & parking_space,
  const lanelet::ConstPolygons3d & all_parking_lots, lanelet::ConstPolygon3d * linked_parking_lot);

// query linked parking space from parking lot
lanelet::ConstLineStrings3d getLinkedParkingSpaces(
  const lanelet::ConstPolygon3d & parking_lot,
  const lanelet::ConstLineStrings3d & all_parking_spaces);
// query linked lanelets from parking lot
lanelet::ConstLanelets getLinkedLanelets(
  const lanelet::ConstPolygon3d & parking_lot, const lanelet::ConstLanelets & all_road_lanelets);

/**
 * [stopLinesLanelets extracts stoplines that are associated to lanelets]
 * @param lanelets [input lanelets]
 * @return         [stop lines that are associated with input lanelets]
 */
std::vector<lanelet::ConstLineString3d> stopLinesLanelets(const lanelet::ConstLanelets lanelets);

/**
 * [stopLinesLanelet extracts stop lines that are associated with a given
 * lanelet]
 * @param ll [input lanelet]
 * @return   [stop lines that are associated with input lanelet]
 */
std::vector<lanelet::ConstLineString3d> stopLinesLanelet(const lanelet::ConstLanelet ll);

/**
 * [stopSignes extracts stoplines that are associated with stopsignes]
 * @param lanelets     [input lanelets]
 * @param stop_sign_id [sign id of stop sign]
 * @return             [array of stoplines]
 */
std::vector<lanelet::ConstLineString3d> stopSignStopLines(
  const lanelet::ConstLanelets lanelets, const std::string & stop_sign_id = "stop_sign");

ConstLanelets getLaneletsWithinRange(
  const lanelet::ConstLanelets & lanelets, const lanelet::BasicPoint2d & search_point,
  const double range);
ConstLanelets getLaneletsWithinRange(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::Point & search_point,
  const double range);

ConstLanelets getLaneChangeableNeighbors(
  const routing::RoutingGraphPtr & graph, const ConstLanelet & lanelet);
ConstLanelets getLaneChangeableNeighbors(
  const routing::RoutingGraphPtr & graph, const ConstLanelets & road_lanelets,
  const geometry_msgs::Point & search_point);

ConstLanelets getAllNeighbors(const routing::RoutingGraphPtr & graph, const ConstLanelet & lanelet);
ConstLanelets getAllNeighborsLeft(
  const routing::RoutingGraphPtr & graph, const ConstLanelet & lanelet);
ConstLanelets getAllNeighborsRight(
  const routing::RoutingGraphPtr & graph, const ConstLanelet & lanelet);
ConstLanelets getAllNeighbors(
  const routing::RoutingGraphPtr & graph, const ConstLanelets & road_lanelets,
  const geometry_msgs::Point & search_point);

bool getClosestLanelet(
  const ConstLanelets & lanelets, const geometry_msgs::Pose & search_pose,
  ConstLanelet * closest_lanelet_ptr);

}  // namespace query
}  // namespace utils
}  // namespace lanelet

#endif  // LANELET2_EXTENSION_UTILITY_QUERY_H
