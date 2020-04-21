/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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
#include <turn_signal_decider/turn_signal_decider.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>

using autoware_planning_msgs::PathWithLaneId;

namespace
{
bool exists(const lanelet::ConstLanelets & lanes, const lanelet::Id & id)
{
  for (const auto & lane : lanes) {
    if (lane.id() == id) {
      return true;
    }
  }
  return false;
}
lanelet::ConstLanelets pathToLanes(
  const PathWithLaneId & path, const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  lanelet::ConstLanelets lanes;
  for (const auto & path_point : path.points) {
    for (const auto & id : path_point.lane_ids) {
      if (!exists(lanes, id)) {
        lanes.push_back(lanelet_map_ptr->laneletLayer.get(id));
      }
    }
  }
  return lanes;
}
}  // namespace

namespace turn_signal_decider
{
DataManager::DataManager()
: is_map_ready_(false), is_path_ready_(false), is_pose_ready_(false), tf_listener_(tf_buffer_)
{
}

void DataManager::onPathWithLaneId(const PathWithLaneId & msg)
{
  path_ = msg;
  is_path_ready_ = true;
  if (is_map_ready_) {
    path_lanes_ = pathToLanes(path_, lanelet_map_ptr_);
  }
}

void DataManager::onLaneletMap(const autoware_lanelet2_msgs::MapBin & map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  is_map_ready_ = true;

  if (is_path_ready_) {
    path_lanes_ = pathToLanes(path_, lanelet_map_ptr_);
  }
}

void DataManager::onVehiclePoseUpdate(const ros::TimerEvent & event)
{
  try {
    const auto current_time = ros::Time::now();
    const auto transform =
      tf_buffer_.lookupTransform("map", "base_link", current_time, ros::Duration(0.1));
    vehicle_pose_.pose.position.x = transform.transform.translation.x;
    vehicle_pose_.pose.position.y = transform.transform.translation.y;
    vehicle_pose_.pose.position.z = transform.transform.translation.z;
    vehicle_pose_.pose.orientation.x = transform.transform.rotation.x;
    vehicle_pose_.pose.orientation.y = transform.transform.rotation.y;
    vehicle_pose_.pose.orientation.z = transform.transform.rotation.z;
    vehicle_pose_.pose.orientation.w = transform.transform.rotation.w;
    vehicle_pose_.header.frame_id = "map";
    vehicle_pose_.header.stamp = current_time;
    is_pose_ready_ = true;
  } catch (tf2::TransformException & ex) {
    // if pose has never arrived before, then wait for localizatioon
    if (!is_pose_ready_) {
      ROS_WARN_STREAM_THROTTLE(5, ex.what());
    } else  // if tf suddenly stops comming, then there must be something wrong.
    {
      ROS_ERROR_STREAM_THROTTLE(5, ex.what());
    }
  }
}

bool DataManager::isPoseValid() const
{
  if (!is_pose_ready_) {
    return false;
  }

  // check time stamp
  constexpr double timeout = 1.0;
  if (ros::Time::now() - vehicle_pose_.header.stamp > ros::Duration(timeout)) {
    return false;
  }

  return true;
}

bool DataManager::isPathValid() const
{
  if (!is_path_ready_) {
    return false;
  }

  // check time stamp
  constexpr double timeout = 1.0;
  if (ros::Time::now() - path_.header.stamp > ros::Duration(timeout)) {
    return false;
  }

  // check lane ids
  if (path_.points.empty()) {
    return false;
  }
  for (const auto & path_point : path_.points) {
    if (path_point.lane_ids.empty()) {
      return false;
    }
  }

  return true;
}

bool DataManager::isDataReady() const
{
  // check map
  if (!is_map_ready_) {
    ROS_WARN_THROTTLE(5, "waiting for vector_map");
    return false;
  }

  // check path
  if (!is_path_ready_) {
    ROS_WARN_THROTTLE(5, "waiting for path_with_lane_id");
    return false;
  }
  if (!isPathValid()) {
    ROS_WARN_THROTTLE(5, "path is invalid!");
    return false;
  }

  // check vehicle pose
  if (!is_pose_ready_) {
    ROS_WARN_THROTTLE(5, "waiting for vehicle pose");
    return false;
  }
  if (!isPoseValid()) {
    ROS_WARN_THROTTLE(5, "vehicle pose is invalid!");
    return false;
  }

  return true;
}

autoware_planning_msgs::PathWithLaneId DataManager::getPath() const { return path_; }

lanelet::LaneletMapPtr DataManager::getMapPtr() const { return lanelet_map_ptr_; }

lanelet::ConstLanelet DataManager::getLaneFromId(const lanelet::Id & id) const
{
  for (const auto & lane : path_lanes_) {
    if (lane.id() == id) {
      return lane;
    }
  }
  return lanelet::Lanelet();
}

lanelet::routing::RoutingGraphPtr DataManager::getRoutingGraphPtr() const
{
  return routing_graph_ptr_;
}
geometry_msgs::PoseStamped DataManager::getVehiclePoseStamped() const { return vehicle_pose_; }

}  // namespace turn_signal_decider
