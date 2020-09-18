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
#ifndef PROCESS_CV_H
#define PROCESS_CV_H

#include <autoware_planning_msgs/Path.h>
#include <geometry_msgs/Point32.h>
#include <boost/optional/optional_fwd.hpp>

struct CVMaps
{
  cv::Mat drivable_area;
  cv::Mat clearance_map;
  cv::Mat only_objects_clearance_map;
  cv::Mat area_with_objects_map;
};

struct Edges
{
  int front_idx;
  int back_idx;
  geometry_msgs::Point extended_front;
  geometry_msgs::Point extended_back;
  geometry_msgs::Point origin;
};

struct PolygonPoints
{
  std::vector<geometry_msgs::Point> points_in_image;
  std::vector<geometry_msgs::Point> points_in_map;
};

namespace process_cv
{
void getOccupancyGridValue(
  const nav_msgs::OccupancyGrid & og, const int i, const int j, unsigned char & value);

void putOccupancyGridValue(
  nav_msgs::OccupancyGrid & og, const int i, const int j, const unsigned char value);

cv::Mat drawObstaclesOnImage(
  const std::vector<autoware_perception_msgs::DynamicObject> & objects,
  const nav_msgs::MapMetaData & map_info, const cv::Mat & clearance_map,
  const double max_avoiding_objects_velocity_ms, const double center_line_width,
  std::vector<autoware_perception_msgs::DynamicObject> * debug_avoiding_objects);

PolygonPoints getPolygonPoints(
  const autoware_perception_msgs::DynamicObject & object, const nav_msgs::MapMetaData & map_info);

PolygonPoints getPolygonPointsFromBB(
  const autoware_perception_msgs::DynamicObject & object, const nav_msgs::MapMetaData & map_info);
PolygonPoints getPolygonPointsFromCircle(
  const autoware_perception_msgs::DynamicObject & object, const nav_msgs::MapMetaData & map_info);
PolygonPoints getPolygonPointsFromPolygon(
  const autoware_perception_msgs::DynamicObject & object, const nav_msgs::MapMetaData & map_info);

bool isAvoidingObject(
  const PolygonPoints & polygon_points, const autoware_perception_msgs::DynamicObject & object,
  const cv::Mat & clearance_map, const nav_msgs::MapMetaData & map_info,
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const double max_avoiding_objects_velocity_ms, const double center_line_width);

std::vector<cv::Point> getCVPolygon(
  const autoware_perception_msgs::DynamicObject & object, const PolygonPoints & polygon_points,
  const std::vector<autoware_planning_msgs::PathPoint> & path_points, const cv::Mat & clearance_map,
  const nav_msgs::MapMetaData & map_info);

std::vector<cv::Point> getDefaultCVPolygon(
  const std::vector<geometry_msgs::Point> & points_in_image);

std::vector<cv::Point> getExtendedCVPolygon(
  const std::vector<geometry_msgs::Point> & points_in_image,
  const std::vector<geometry_msgs::Point> & points_in_map,
  const geometry_msgs::Pose & nearest_path_point_pose,
  const autoware_perception_msgs::DynamicObject & object, const cv::Mat & clearance_map,
  const nav_msgs::MapMetaData & map_info);

cv::Mat getDrivableAreaInCV(const nav_msgs::OccupancyGrid & occupancy_grid);

cv::Mat getClearanceMap(const cv::Mat & drivable_area);

cv::Mat getAreaWithObjects(const cv::Mat & drivable_area, const cv::Mat & objects_image);

boost::optional<Edges> getEdges(
  const std::vector<geometry_msgs::Point> & points_in_image,
  const std::vector<geometry_msgs::Point> & points_in_map,
  const geometry_msgs::Pose & nearest_path_point_pose,
  const autoware_perception_msgs::DynamicObject & object, const cv::Mat & clearance_map,
  const nav_msgs::MapMetaData & map_info);

bool arePointsInsideDriveableArea(
  const std::vector<geometry_msgs::Point> & image_points, const cv::Mat & clearance_map);

CVMaps getMaps(
  const autoware_planning_msgs::Path & path,
  const std::vector<autoware_perception_msgs::DynamicObject> & objects,
  const double max_avoiding_objects_velocity_ms, const double center_line_width,
  DebugData * debug_data);
}  // namespace process_cv
#endif
