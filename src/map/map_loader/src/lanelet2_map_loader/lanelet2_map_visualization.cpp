/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 */

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_projection/UTM.h>

#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>

#include <vector>

static bool g_viz_lanelets_centerline = true;
static ros::Publisher g_map_pub;

void insertMarkerArray(
  visualization_msgs::MarkerArray * a1, const visualization_msgs::MarkerArray & a2)
{
  a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
}

void setColor(std_msgs::ColorRGBA * cl, double r, double g, double b, double a)
{
  cl->r = r;
  cl->g = g;
  cl->b = b;
  cl->a = a;
}

void binMapCallback(autoware_lanelet2_msgs::MapBin msg)
{
  lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(msg, viz_lanelet_map);
  ROS_INFO("Map is loaded\n");

  // get lanelets etc to visualize
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  lanelet::ConstLanelets crosswalk_lanelets =
    lanelet::utils::query::crosswalkLanelets(all_lanelets);
  std::vector<lanelet::ConstLineString3d> stop_lines =
    lanelet::utils::query::stopLinesLanelets(road_lanelets);
  std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems =
    lanelet::utils::query::trafficLights(all_lanelets);
  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  std::vector<lanelet::DetectionAreaConstPtr> da_reg_elems =
    lanelet::utils::query::detectionAreas(all_lanelets);
  lanelet::ConstLineStrings3d parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(viz_lanelet_map);
  lanelet::ConstPolygons3d parking_lots = lanelet::utils::query::getAllParkingLots(viz_lanelet_map);

  std_msgs::ColorRGBA cl_road, cl_cross, cl_ll_borders, cl_stoplines, cl_trafficlights,
    cl_detection_areas, cl_parking_lots, cl_parking_spaces;
  setColor(&cl_road, 0.2, 0.7, 0.7, 0.3);
  setColor(&cl_cross, 0.2, 0.7, 0.2, 0.3);
  setColor(&cl_ll_borders, 1.0, 1.0, 1.0, 0.999);
  setColor(&cl_stoplines, 1.0, 0.0, 0.0, 0.5);
  setColor(&cl_trafficlights, 0.7, 0.7, 0.7, 0.8);
  setColor(&cl_detection_areas, 0.7, 0.7, 0.7, 0.3);
  setColor(&cl_parking_lots, 0.7, 0.7, 0.0, 0.3);
  setColor(&cl_parking_spaces, 1.0, 0.647, 0.0, 0.6);

  visualization_msgs::MarkerArray map_marker_array;

  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletsBoundaryAsMarkerArray(
                         road_lanelets, cl_ll_borders, g_viz_lanelets_centerline));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("road_lanelets", road_lanelets, cl_road));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                         "crosswalk_lanelets", crosswalk_lanelets, cl_cross));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletDirectionAsMarkerArray(road_lanelets));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::lineStringsAsMarkerArray(stop_lines, "stop_lines", cl_stoplines, 0.5));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::autowareTrafficLightsAsMarkerArray(aw_tl_reg_elems, cl_trafficlights));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::detectionAreasAsMarkerArray(da_reg_elems, cl_detection_areas));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::parkingLotsAsMarkerArray(parking_lots, cl_parking_lots));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::parkingSpacesAsMarkerArray(parking_spaces, cl_parking_spaces));

  g_map_pub.publish(map_marker_array);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "lanelet2_map_visualizer");
  ros::NodeHandle pnh("~");
  ros::Subscriber bin_map_sub;

  bin_map_sub = pnh.subscribe("input/lanelet2_map", 1, binMapCallback);
  g_map_pub = pnh.advertise<visualization_msgs::MarkerArray>("output/lanelet2_map_marker", 1, true);

  ros::spin();

  return 0;
}
