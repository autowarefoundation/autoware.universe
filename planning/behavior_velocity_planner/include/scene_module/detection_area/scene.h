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
#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <boost/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/regulatory_elements/detection_area.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.h>

class DetectionAreaModule : public SceneModuleInterface
{
public:
  enum class State { APPROACH, STOP, PASS };

  struct DebugData
  {
    double base_link2front;
    std::vector<geometry_msgs::Pose> stop_poses;
    std::vector<geometry_msgs::Pose> dead_line_poses;
    geometry_msgs::Pose first_stop_pose;
    std::vector<geometry_msgs::Point> detection_points;
  };

  struct PlannerParam
  {
    double stop_margin;
  };

public:
  DetectionAreaModule(
    const int64_t module_id, const lanelet::autoware::DetectionArea & detection_area_reg_elem,
    const PlannerParam & planner_param);

  bool modifyPathVelocity(
    autoware_planning_msgs::PathWithLaneId * path,
    autoware_planning_msgs::StopReason * stop_reason) override;

  visualization_msgs::MarkerArray createDebugMarkerArray() override;

private:
  int64_t module_id_;

  using Point = boost::geometry::model::d2::point_xy<double>;
  using Line = boost::geometry::model::linestring<Point>;
  using Polygon = boost::geometry::model::polygon<Point, false>;

  bool isPointsWithinDetectionArea(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & no_ground_pointcloud_ptr,
    const lanelet::ConstPolygons3d & detection_areas);

  bool getBackwordPointFromBasePoint(
    const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
    const Eigen::Vector2d & base_point, const double backward_length,
    Eigen::Vector2d & output_point);

  bool insertTargetVelocityPoint(
    const autoware_planning_msgs::PathWithLaneId & input,
    const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &
      stop_line,
    const double & margin, const double & velocity,
    autoware_planning_msgs::PathWithLaneId & output);

  bool createTargetPoint(
    const autoware_planning_msgs::PathWithLaneId & input,
    const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &
      stop_line,
    const double & margin, size_t & target_point_idx, Eigen::Vector2d & target_point);

  bool isOverDeadLine(
    const geometry_msgs::Pose & self_pose, const autoware_planning_msgs::PathWithLaneId & input_path,
    const size_t & dead_line_point_idx, const Eigen::Vector2d & dead_line_point,
    const double dead_line_range);

  // Key Feature
  const lanelet::autoware::DetectionArea & detection_area_reg_elem_;

  // State
  State state_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;
};
