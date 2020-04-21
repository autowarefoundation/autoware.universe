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

#include <string>
#include <unordered_map>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.h>

class StopLineModule : public SceneModuleInterface
{
public:
  enum class State { APPROARCH, STOP, START };

  struct DebugData
  {
    double base_link2front;
    std::vector<geometry_msgs::Pose> stop_poses;
  };

public:
  StopLineModule(const int64_t module_id, const lanelet::ConstLineString3d & stop_line);

  bool modifyPathVelocity(autoware_planning_msgs::PathWithLaneId * path) override;

  visualization_msgs::MarkerArray createDebugMarkerArray() override;

private:
  bool getBackwordPointFromBasePoint(
    const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
    const Eigen::Vector2d & base_point, const double backward_length,
    Eigen::Vector2d & output_point);

  lanelet::ConstLineString3d stop_line_;
  State state_;

  // Paramter
  const double stop_margin_ = 0.0;

  // Debug
  DebugData debug_data_;
};
