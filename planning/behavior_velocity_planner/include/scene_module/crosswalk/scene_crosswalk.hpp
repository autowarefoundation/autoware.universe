// Copyright 2020 Tier IV, Inc.
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
#pragma once

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/dynamic_object_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <scene_module/crosswalk/util.hpp>
#include <scene_module/scene_module_interface.hpp>

class CrosswalkModule : public SceneModuleInterface
{
public:
  struct PlannerParam
  {
    double stop_margin;
    double slow_margin;
    double slow_velocity;
    double stop_dynamic_object_prediction_time_margin;
  };

  CrosswalkModule(
    const int64_t module_id, const lanelet::ConstLanelet & crosswalk,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(
    autoware_planning_msgs::msg::PathWithLaneId * path,
    autoware_planning_msgs::msg::StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;

private:
  int64_t module_id_;

  bool checkSlowArea(
    const autoware_planning_msgs::msg::PathWithLaneId & input,
    const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false> &
      polygon,
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr & objects_ptr,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & no_ground_pointcloud_ptr,
    autoware_planning_msgs::msg::PathWithLaneId & output);

  bool checkStopArea(
    const autoware_planning_msgs::msg::PathWithLaneId & input,
    const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false> &
      polygon,
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr & objects_ptr,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & no_ground_pointcloud_ptr,
    autoware_planning_msgs::msg::PathWithLaneId & output, bool * insert_stop);

  bool isTargetType(const autoware_perception_msgs::msg::DynamicObject & obj);

  enum class State { APPROACH, INSIDE, GO_OUT };

  lanelet::ConstLanelet crosswalk_;
  State state_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;
};
