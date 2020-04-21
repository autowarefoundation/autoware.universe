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

#include <opencv2/core.hpp>

#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "obstacle_avoidance_planner/debug.h"
#include "obstacle_avoidance_planner/eb_path_optimizer.h"
#include "obstacle_avoidance_planner/process_cv.h"

visualization_msgs::MarkerArray getDebugVisualizationMarker(
  const std::vector<geometry_msgs::Point> & interpolated_points,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & optimized_points,
  const std::vector<geometry_msgs::Point> & straight_points,
  const std::vector<geometry_msgs::Pose> & fixed_points,
  const std::vector<geometry_msgs::Pose> & non_fixed_points,
  const std::vector<ConstrainRectangle> & constrain_ranges)
{
  const auto points_marker_array = getDebugPointsMarkers(
    interpolated_points, optimized_points, straight_points, fixed_points, non_fixed_points);
  const auto constrain_marker_array = getDebugConstrainMarkers(constrain_ranges);
  visualization_msgs::MarkerArray vis_marker_array;
  vis_marker_array.markers.insert(
    vis_marker_array.markers.end(), points_marker_array.markers.begin(),
    points_marker_array.markers.end());
  vis_marker_array.markers.insert(
    vis_marker_array.markers.end(), constrain_marker_array.markers.begin(),
    constrain_marker_array.markers.end());
  return vis_marker_array;
}

visualization_msgs::MarkerArray getDebugPointsMarkers(
  const std::vector<geometry_msgs::Point> & interpolated_points,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & optimized_points,
  const std::vector<geometry_msgs::Point> & straight_points,
  const std::vector<geometry_msgs::Pose> & fixed_points,
  const std::vector<geometry_msgs::Pose> & non_fixed_points)
{
  visualization_msgs::MarkerArray marker_array;
  int unique_id = 0;

  unique_id = 0;
  visualization_msgs::Marker interpolated_points_marker;
  interpolated_points_marker.lifetime = ros::Duration(-1);
  interpolated_points_marker.header.frame_id = "map";
  interpolated_points_marker.header.stamp = ros::Time(0);
  interpolated_points_marker.ns = std::string("interpolated_points_marker");
  interpolated_points_marker.action = visualization_msgs::Marker::ADD;
  interpolated_points_marker.pose.orientation.w = 1.0;
  interpolated_points_marker.id = unique_id;
  interpolated_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  interpolated_points_marker.scale.x = .3;
  interpolated_points_marker.scale.y = .3;
  interpolated_points_marker.scale.z = .3;
  interpolated_points_marker.color.r = 1.0f;
  interpolated_points_marker.color.a = 0.99;
  unique_id++;
  for (const auto & point : interpolated_points) {
    interpolated_points_marker.points.push_back(point);
  }
  if (!interpolated_points_marker.points.empty()) {
    marker_array.markers.push_back(interpolated_points_marker);
  }

  unique_id = 0;
  visualization_msgs::Marker optimized_points_marker;
  optimized_points_marker.lifetime = ros::Duration(-1);
  optimized_points_marker.header.frame_id = "map";
  optimized_points_marker.header.stamp = ros::Time(0);
  optimized_points_marker.ns = std::string("optimized_points_marker");
  optimized_points_marker.action = visualization_msgs::Marker::ADD;
  optimized_points_marker.pose.orientation.w = 1.0;
  optimized_points_marker.id = unique_id;
  optimized_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  optimized_points_marker.scale.x = 0.35;
  optimized_points_marker.scale.y = 0.35;
  optimized_points_marker.scale.z = 0.35;
  optimized_points_marker.color.g = 1.0f;
  optimized_points_marker.color.a = 0.99;
  unique_id++;
  for (const auto & point : optimized_points) {
    optimized_points_marker.points.push_back(point.pose.position);
  }
  if (!optimized_points_marker.points.empty()) {
    marker_array.markers.push_back(optimized_points_marker);
  }

  unique_id = 0;
  for (int i = 0; i < optimized_points.size(); i++) {
    visualization_msgs::Marker optimized_points_text_marker;
    optimized_points_text_marker.lifetime = ros::Duration(-1);
    optimized_points_text_marker.header.frame_id = "map";
    optimized_points_text_marker.header.stamp = ros::Time(0);
    optimized_points_text_marker.ns = std::string("optimized_points_text_marker");
    optimized_points_text_marker.action = visualization_msgs::Marker::ADD;
    optimized_points_text_marker.pose.orientation.w = 1.0;
    optimized_points_text_marker.id = unique_id;
    optimized_points_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    optimized_points_text_marker.pose.position = optimized_points[i].pose.position;
    optimized_points_text_marker.scale.z = 0.5;
    optimized_points_text_marker.color.g = 1.0f;
    optimized_points_text_marker.color.a = 0.99;
    optimized_points_text_marker.text = std::to_string(i);
    unique_id++;
    marker_array.markers.push_back(optimized_points_text_marker);
  }

  unique_id = 0;
  for (int i = 0; i < interpolated_points.size(); i++) {
    visualization_msgs::Marker interpolated_points_text_marker;
    interpolated_points_text_marker.lifetime = ros::Duration(-1);
    interpolated_points_text_marker.header.frame_id = "map";
    interpolated_points_text_marker.header.stamp = ros::Time(0);
    interpolated_points_text_marker.ns = std::string("interpolated_points_text_marker");
    interpolated_points_text_marker.action = visualization_msgs::Marker::ADD;
    interpolated_points_text_marker.pose.orientation.w = 1.0;
    interpolated_points_text_marker.id = unique_id;
    interpolated_points_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    interpolated_points_text_marker.pose.position = interpolated_points[i];
    interpolated_points_text_marker.scale.z = 0.5;
    interpolated_points_text_marker.color.g = 1.0f;
    interpolated_points_text_marker.color.a = 0.99;
    interpolated_points_text_marker.text = std::to_string(i);
    unique_id++;
    marker_array.markers.push_back(interpolated_points_text_marker);
  }

  unique_id = 0;
  visualization_msgs::Marker straight_points_marker;
  straight_points_marker.lifetime = ros::Duration(40);
  straight_points_marker.header.frame_id = "map";
  straight_points_marker.header.stamp = ros::Time(0);
  straight_points_marker.ns = std::string("straight_points_marker");
  straight_points_marker.action = visualization_msgs::Marker::ADD;
  straight_points_marker.pose.orientation.w = 1.0;
  straight_points_marker.id = unique_id;
  straight_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  straight_points_marker.scale.x = 1.0;
  straight_points_marker.scale.y = 1.0;
  straight_points_marker.scale.z = 1.0;
  straight_points_marker.color.r = 1.0f;
  straight_points_marker.color.g = 1.0f;
  straight_points_marker.color.a = 0.99;
  unique_id++;
  for (const auto & point : straight_points) {
    straight_points_marker.points.push_back(point);
  }
  if (!straight_points_marker.points.empty()) {
    marker_array.markers.push_back(straight_points_marker);
  }

  unique_id = 0;
  visualization_msgs::Marker fixed_marker;
  fixed_marker.lifetime = ros::Duration(-1);
  fixed_marker.header.frame_id = "map";
  fixed_marker.header.stamp = ros::Time(0);
  fixed_marker.ns = std::string("fixed_points_marker");
  fixed_marker.action = visualization_msgs::Marker::ADD;
  fixed_marker.pose.orientation.w = 1.0;
  fixed_marker.id = unique_id;
  fixed_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  fixed_marker.scale.x = 0.3;
  fixed_marker.scale.y = 0.3;
  fixed_marker.scale.z = 0.3;
  fixed_marker.color.r = 1.0f;
  fixed_marker.color.a = 0.999;
  unique_id++;
  for (const auto & point : fixed_points) {
    fixed_marker.points.push_back(point.position);
  }
  marker_array.markers.push_back(fixed_marker);

  visualization_msgs::Marker non_fixed_marker;
  non_fixed_marker.lifetime = ros::Duration(20);
  non_fixed_marker.header.frame_id = "map";
  non_fixed_marker.header.stamp = ros::Time(0);
  non_fixed_marker.ns = std::string("non_fixed_points_marker");
  non_fixed_marker.action = visualization_msgs::Marker::ADD;
  non_fixed_marker.pose.orientation.w = 1.0;
  non_fixed_marker.id = unique_id;
  non_fixed_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  non_fixed_marker.scale.x = 1.0;
  non_fixed_marker.scale.y = 1.0;
  non_fixed_marker.scale.z = 1.0;
  non_fixed_marker.color.g = 1.0f;
  non_fixed_marker.color.a = 0.99;
  unique_id++;
  for (const auto & point : non_fixed_points) {
    non_fixed_marker.points.push_back(point.position);
  }
  if (!non_fixed_marker.points.empty()) {
    marker_array.markers.push_back(non_fixed_marker);
  }
  return marker_array;
}

visualization_msgs::MarkerArray getDebugConstrainMarkers(
  const std::vector<ConstrainRectangle> & constrain_ranges)
{
  visualization_msgs::MarkerArray marker_array;
  int unique_id = 0;
  for (int i = 0; i < constrain_ranges.size(); i++) {
    visualization_msgs::Marker constrain_rect_marker;
    constrain_rect_marker.lifetime = ros::Duration(-1);
    constrain_rect_marker.header.frame_id = "map";
    constrain_rect_marker.header.stamp = ros::Time(0);
    constrain_rect_marker.ns = std::string("constrain_rect_marker");
    constrain_rect_marker.action = visualization_msgs::Marker::ADD;
    constrain_rect_marker.pose.orientation.w = 1.0;
    constrain_rect_marker.id = unique_id;
    constrain_rect_marker.type = visualization_msgs::Marker::LINE_STRIP;
    constrain_rect_marker.scale.x = 0.01;
    constrain_rect_marker.color.r = 1.0f;
    constrain_rect_marker.color.a = 0.99;
    unique_id++;
    geometry_msgs::Point top_left_point = constrain_ranges[i].top_left;
    geometry_msgs::Point top_right_point = constrain_ranges[i].top_right;
    geometry_msgs::Point bottom_right_point = constrain_ranges[i].bottom_right;
    geometry_msgs::Point bottom_left_point = constrain_ranges[i].bottom_left;
    constrain_rect_marker.points.push_back(top_left_point);
    constrain_rect_marker.points.push_back(top_right_point);
    constrain_rect_marker.points.push_back(bottom_right_point);
    constrain_rect_marker.points.push_back(bottom_left_point);
    constrain_rect_marker.points.push_back(top_left_point);
    marker_array.markers.push_back(constrain_rect_marker);
  }

  unique_id = 0;
  for (int i = 0; i < constrain_ranges.size(); i++) {
    visualization_msgs::Marker constrain_range_text_marker;
    constrain_range_text_marker.lifetime = ros::Duration(-1);
    constrain_range_text_marker.header.frame_id = "map";
    constrain_range_text_marker.header.stamp = ros::Time(0);
    constrain_range_text_marker.ns = std::string("constrain_range_location_marker");
    constrain_range_text_marker.action = visualization_msgs::Marker::ADD;
    constrain_range_text_marker.pose.orientation.w = 1.0;
    constrain_range_text_marker.id = unique_id;
    constrain_range_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    constrain_range_text_marker.pose.position = constrain_ranges[i].top_left;
    constrain_range_text_marker.scale.z = 0.1;
    constrain_range_text_marker.color.r = 1.0f;
    constrain_range_text_marker.color.a = 0.99;
    constrain_range_text_marker.text = std::to_string(i) + std::string(" x ") +
                                       std::to_string(constrain_range_text_marker.pose.position.x) +
                                       std::string("y ") +
                                       std::to_string(constrain_range_text_marker.pose.position.y);
    unique_id++;
    marker_array.markers.push_back(constrain_range_text_marker);

    constrain_range_text_marker.id = unique_id;
    constrain_range_text_marker.pose.position = constrain_ranges[i].top_right;
    constrain_range_text_marker.text = std::to_string(i) + std::string(" x ") +
                                       std::to_string(constrain_range_text_marker.pose.position.x) +
                                       std::string("y ") +
                                       std::to_string(constrain_range_text_marker.pose.position.y);
    unique_id++;
    marker_array.markers.push_back(constrain_range_text_marker);

    constrain_range_text_marker.id = unique_id;
    constrain_range_text_marker.pose.position = constrain_ranges[i].bottom_left;
    constrain_range_text_marker.text = std::to_string(i) + std::string(" x ") +
                                       std::to_string(constrain_range_text_marker.pose.position.x) +
                                       std::string("y ") +
                                       std::to_string(constrain_range_text_marker.pose.position.y);
    unique_id++;
    marker_array.markers.push_back(constrain_range_text_marker);

    constrain_range_text_marker.id = unique_id;
    constrain_range_text_marker.pose.position = constrain_ranges[i].bottom_right;
    constrain_range_text_marker.text = std::to_string(i) + std::string(" x ") +
                                       std::to_string(constrain_range_text_marker.pose.position.x) +
                                       std::string("y ") +
                                       std::to_string(constrain_range_text_marker.pose.position.y);
    unique_id++;
    marker_array.markers.push_back(constrain_range_text_marker);
  }
  return marker_array;
}

nav_msgs::OccupancyGrid getDebugCostmap(
  const cv::Mat & clearance_map, const nav_msgs::OccupancyGrid & occupancy_grid)
{
  cv::Mat tmp;
  clearance_map.copyTo(tmp);
  cv::normalize(tmp, tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  nav_msgs::OccupancyGrid clearance_map_in_og = occupancy_grid;
  tmp.forEach<unsigned char>([&](const unsigned char & value, const int * position) -> void {
    process_cv::putOccupancyGridValue(clearance_map_in_og, position[0], position[1], value);
  });
  return clearance_map_in_og;
}
