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
 * Authors: Yukihiro Saito
 *
 */

#pragma once

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/TrafficLightRoiArray.h>
#include <autoware_planning_msgs/Route.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>

namespace traffic_light
{
class MapBasedDetector
{
public:
  MapBasedDetector();
  ~MapBasedDetector() {}

private:
  struct Config
  {
    double max_vibration_pitch;
    double max_vibration_yaw;
    double max_vibration_height;
    double max_vibration_width;
    double max_vibration_depth;
  };

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber map_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Subscriber route_sub_;
  ros::Publisher roi_pub_;
  ros::Publisher viz_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  sensor_msgs::CameraInfo::ConstPtr camera_info_ptr_;
  std::shared_ptr<std::vector<lanelet::ConstLineString3d>> all_traffic_lights_ptr_;
  std::shared_ptr<std::vector<lanelet::ConstLineString3d>> route_traffic_lights_ptr_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  Config config_;

  void mapCallback(const autoware_lanelet2_msgs::MapBin & input_msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & input_msg);
  void routeCallback(const autoware_planning_msgs::Route::ConstPtr & input_msg);
  void isInVisibility(
    const std::vector<lanelet::ConstLineString3d> & all_traffic_lights,
    const geometry_msgs::Pose & camera_pose, const sensor_msgs::CameraInfo & camera_info,
    std::vector<lanelet::ConstLineString3d> & visible_traffic_lights);
  bool isInDistanceRange(
    const geometry_msgs::Point & tl_point, const geometry_msgs::Point & camera_point,
    const double max_distance_range);
  bool isInAngleRange(
    const double & tl_yaw, const double & camera_yaw, const double max_angele_range);
  bool isInImageFrame(
    const sensor_msgs::CameraInfo & camera_info, const geometry_msgs::Point & point);
  double normalizeAngle(const double & angle);
  bool getTrafficLightRoi(
    const geometry_msgs::Pose & camera_pose, const sensor_msgs::CameraInfo & camera_info,
    const lanelet::ConstLineString3d traffic_light, const Config & config,
    autoware_perception_msgs::TrafficLightRoi & tl_roi);
  void publishVisibleTrafficLights(
    const geometry_msgs::PoseStamped camera_pose_stamped,
    const std::vector<lanelet::ConstLineString3d> & visible_traffic_lights,
    const ros::Publisher & pub);
};
}  // namespace traffic_light
