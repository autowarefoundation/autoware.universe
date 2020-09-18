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
#include "shape_estimation/vector_map.hpp"

#include <unordered_map>
#include "ros/ros.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/PointArray.h"

bool VectorMap::load()
{
  auto points = ros::topic::waitForMessage<vector_map_msgs::PointArray>(
    "/vector_map_info/point", ros::Duration());
  if (!points) {
    return false;
  }

  auto dtlanes = ros::topic::waitForMessage<vector_map_msgs::DTLaneArray>(
    "/vector_map_info/dtlane", ros::Duration());
  if (!dtlanes) {
    return false;
  }

  std::unordered_map<int, vector_map_msgs::Point> points_map;
  for (const auto & point : points->data) {
    points_map[point.pid] = point;
  }

  lane_points_.reserve(dtlanes->data.size());
  for (const auto & dtlane : dtlanes->data) {
    const auto & point = points_map[dtlane.pid];
    LanePoint lane_point;
    lane_point.tx = point.ly;
    lane_point.ty = point.bx;
    lane_point.rz = dtlane.dir;
    lane_point.sr = 1.0 + std::pow(std::max(dtlane.lw, dtlane.rw), 2);
    lane_points_.emplace_back(lane_point);
  }

  return true;
}

bool VectorMap::getyaw(double x, double y, double & yaw) const
{
  double dist = 1e+10;
  bool flag = false;
  for (const auto & lane_point : lane_points_) {
    double dx = x - lane_point.tx;
    double dy = y - lane_point.ty;
    double dr = (dx * dx) + (dy * dy);
    if ((dr < dist) && (dr < lane_point.sr)) {
      yaw = lane_point.rz;
      dist = dr;
      flag = true;
    }
  }
  return flag;
}