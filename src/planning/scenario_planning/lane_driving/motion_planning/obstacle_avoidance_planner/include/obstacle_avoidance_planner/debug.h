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
#ifndef DEBUG_OBSTACLEA_AVOIDANCE_PLANNER_H
#define DEBUG_OBSTACLEA_AVOIDANCE_PLANNER_H

class ConstrainRectangle;

visualization_msgs::MarkerArray getDebugVisualizationMarker(
  const std::vector<geometry_msgs::Point> & interpolated_points,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & optimized_points,
  const std::vector<geometry_msgs::Point> & straight_points,
  const std::vector<geometry_msgs::Pose> & fixed_points,
  const std::vector<geometry_msgs::Pose> & non_fixed_points,
  const std::vector<ConstrainRectangle> & constrain_ranges);

visualization_msgs::MarkerArray getDebugPointsMarkers(
  const std::vector<geometry_msgs::Point> & interpolated_points,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & optimized_points,
  const std::vector<geometry_msgs::Point> & straight_points,
  const std::vector<geometry_msgs::Pose> & fixed_points,
  const std::vector<geometry_msgs::Pose> & non_fixed_points);

visualization_msgs::MarkerArray getDebugConstrainMarkers(
  const std::vector<ConstrainRectangle> & constrain_ranges);

nav_msgs::OccupancyGrid getDebugCostmap(
  const cv::Mat & clearance_map, const nav_msgs::OccupancyGrid & occupancy_grid);
#endif
