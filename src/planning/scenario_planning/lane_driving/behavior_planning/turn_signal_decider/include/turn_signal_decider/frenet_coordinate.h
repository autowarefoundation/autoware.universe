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

#include <autoware_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Point.h>
#include <vector>

namespace turn_signal_decider
{
struct FrenetCoordinate3d
{
  double length;
  double distance;
  FrenetCoordinate3d() : length(0), distance(0) {}
};

bool convertToFrenetCoordinate3d(
  const autoware_planning_msgs::PathWithLaneId & path,
  const geometry_msgs::Point & search_point_geom, FrenetCoordinate3d * frenet_coordinate);
bool convertToFrenetCoordinate3d(
  const std::vector<geometry_msgs::Point> & linestring,
  const geometry_msgs::Point & search_point_geom, FrenetCoordinate3d * frenet_coordinate);
}  // namespace turn_signal_decider