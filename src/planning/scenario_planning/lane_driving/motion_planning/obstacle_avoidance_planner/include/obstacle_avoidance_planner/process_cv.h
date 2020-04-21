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

namespace process_cv
{
void getOccupancyGridValue(
  const nav_msgs::OccupancyGrid & og, const int i, const int j, unsigned char & value);

void putOccupancyGridValue(
  nav_msgs::OccupancyGrid & og, const int i, const int j, const unsigned char value);

cv::Mat drawObstalcesOnImage(
  const std::vector<autoware_perception_msgs::DynamicObject> & objects,
  const nav_msgs::MapMetaData & map_info, const cv::Mat & clearance_map,
  const double max_avoiding_objects_velocity_ms);

std::vector<cv::Point> getCVPolygonFromBoundingBox(
  const autoware_perception_msgs::DynamicObject & object, const nav_msgs::MapMetaData & map_info);

std::vector<cv::Point> getCVPolygonFromPolygon(
  const autoware_perception_msgs::DynamicObject & object, const nav_msgs::MapMetaData & map_info);

cv::Mat getOnlyObjectsClearanceMap(
  const cv::Mat & clearance_map,
  const std::vector<autoware_perception_msgs::DynamicObject> & objects,
  const nav_msgs::OccupancyGrid & occupancy_grid, const double max_avoiding_objects_velocity_ms);

cv::Mat getDrivableAreaInCV(const nav_msgs::OccupancyGrid & occupancy_grid);

cv::Mat getClearanceMap(
  const nav_msgs::OccupancyGrid & occupancy_grid, const cv::Mat & drivable_area,
  const std::vector<autoware_perception_msgs::DynamicObject> & objects);
}  // namespace process_cv
#endif
