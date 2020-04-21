/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 */

#ifndef MAP_BASED_PREDICTION_ROS_H
#define MAP_BASED_PREDICTION_ROS_H

#include <unordered_map>

namespace tf2_ros
{
class Buffer;
class TransformListener;
}  // namespace tf2_ros

namespace lanelet
{
class Lanelet;
class LaneletMap;
using LaneletMapPtr = std::shared_ptr<LaneletMap>;
namespace routing
{
class RoutingGraph;
}
namespace traffic_rules
{
class TrafficRules;
}
}  // namespace lanelet

namespace geometry_msgs
{
ROS_DECLARE_MESSAGE(Pose);
}

namespace autoware_lanelet2_msgs
{
ROS_DECLARE_MESSAGE(MapBin);
}

namespace autoware_perception_msgs
{
ROS_DECLARE_MESSAGE(DynamicObjectArray);
ROS_DECLARE_MESSAGE(DynamicObject);
}  // namespace autoware_perception_msgs

namespace uuid_msgs
{
ROS_DECLARE_MESSAGE(UniqueID);
}

class MapBasedPrediction;

class MapBasedPredictionROS
{
private:
  bool has_subscribed_map_;
  double prediction_time_horizon_;
  double prediction_sampling_delta_time_;
  double interpolating_resolution_;
  double debug_accumulated_time_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_objects_;
  ros::Subscriber sub_map_;
  ros::Publisher pub_objects_;
  ros::Publisher pub_markers_;

  std::unordered_map<std::string, std::vector<int>> uuid2laneids_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
  std::shared_ptr<MapBasedPrediction> map_based_prediction_;

  bool getSelfPose(geometry_msgs::Pose & self_pose, const std_msgs::Header & header);
  bool getSelfPoseInMap(geometry_msgs::Pose & self_pose);

  void objectsCallback(const autoware_perception_msgs::DynamicObjectArrayConstPtr & in_objects);
  void mapCallback(const autoware_lanelet2_msgs::MapBin & msg);

  bool getClosestLanelets(
    const autoware_perception_msgs::DynamicObject & object,
    const lanelet::LaneletMapPtr & lanelet_map_ptr,
    std::vector<lanelet::Lanelet> & closest_lanelets, std::string uuid_string);

public:
  MapBasedPredictionROS();

  void createROSPubSub();
};

#endif  // MAP_BASED_PREDICTION_H
