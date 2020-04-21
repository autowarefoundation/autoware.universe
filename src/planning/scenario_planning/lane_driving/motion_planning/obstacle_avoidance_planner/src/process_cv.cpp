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

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>

#include <autoware_perception_msgs/DynamicObject.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/utils.h>

#include "obstacle_avoidance_planner/process_cv.h"
#include "obstacle_avoidance_planner/util.h"

namespace process_cv
{
void getOccupancyGridValue(
  const nav_msgs::OccupancyGrid & og, const int i, const int j, unsigned char & value)
{
  int i_flip = og.info.width - i - 1;
  int j_flip = og.info.height - j - 1;
  if (og.data[i_flip + j_flip * og.info.width] > 0) {
    value = 0;
  } else {
    value = 255;
  }
}

void putOccupancyGridValue(
  nav_msgs::OccupancyGrid & og, const int i, const int j, const unsigned char value)
{
  int i_flip = og.info.width - i - 1;
  int j_flip = og.info.height - j - 1;
  og.data[i_flip + j_flip * og.info.width] = value;
}

cv::Mat drawObstalcesOnImage(
  const std::vector<autoware_perception_msgs::DynamicObject> & objects,
  const nav_msgs::MapMetaData & map_info, const cv::Mat & clearance_map,
  const double max_avoiding_objects_velocity_ms)
{
  cv::Mat objects_image = cv::Mat::ones(clearance_map.rows, clearance_map.cols, CV_8UC1) * 255;
  std::vector<std::vector<cv::Point>> cv_polygons;
  for (const auto & object : objects) {
    float clearance = std::numeric_limits<float>::lowest();
    std::shared_ptr<geometry_msgs::Point> image_point_ptr =
      util::transformMapToImagePtr(object.state.pose_covariance.pose.position, map_info);
    if (image_point_ptr) {
      clearance = clearance_map.ptr<float>((int)image_point_ptr->y)[(int)image_point_ptr->x];
    } else {
      continue;
    }
    const double vel = std::sqrt(
      object.state.twist_covariance.twist.linear.x * object.state.twist_covariance.twist.linear.x +
      object.state.twist_covariance.twist.linear.y * object.state.twist_covariance.twist.linear.y +
      object.state.twist_covariance.twist.linear.z * object.state.twist_covariance.twist.linear.z);
    if (vel < max_avoiding_objects_velocity_ms && clearance > 0) {
      if (object.shape.type == object.shape.BOUNDING_BOX) {
        cv_polygons.push_back(getCVPolygonFromBoundingBox(object, map_info));
      } else if (object.shape.type == object.shape.POLYGON) {
        cv_polygons.push_back(getCVPolygonFromPolygon(object, map_info));
      } else if (object.shape.type == object.shape.CYLINDER) {
        cv::circle(
          objects_image, cv::Point(image_point_ptr->x, image_point_ptr->y),
          (int)(object.shape.dimensions.x / map_info.resolution), cv::Scalar(0), -1);
      }
    }
  }
  cv::fillPoly(objects_image, cv_polygons, cv::Scalar(0));
  return objects_image;
}

std::vector<cv::Point> getCVPolygonFromBoundingBox(
  const autoware_perception_msgs::DynamicObject & object, const nav_msgs::MapMetaData & map_info)
{
  geometry_msgs::Pose object_pose = object.state.pose_covariance.pose;
  geometry_msgs::Point top_left;
  top_left.x = object.shape.dimensions.x * 0.5;
  top_left.y = object.shape.dimensions.y * 0.5;
  geometry_msgs::Point top_left_map = util::transformToAbsoluteCoordinate2D(top_left, object_pose);

  geometry_msgs::Point top_right;
  top_right.x = object.shape.dimensions.x * 0.5;
  top_right.y = -1 * object.shape.dimensions.y * 0.5;
  geometry_msgs::Point top_right_map =
    util::transformToAbsoluteCoordinate2D(top_right, object_pose);

  geometry_msgs::Point bottom_left;
  bottom_left.x = -1 * object.shape.dimensions.x * 0.5;
  bottom_left.y = object.shape.dimensions.y * 0.5;
  geometry_msgs::Point bottom_left_map =
    util::transformToAbsoluteCoordinate2D(bottom_left, object_pose);

  geometry_msgs::Point bottom_right;
  bottom_right.x = -1 * object.shape.dimensions.x * 0.5;
  bottom_right.y = -1 * object.shape.dimensions.y * 0.5;
  geometry_msgs::Point bottom_right_map =
    util::transformToAbsoluteCoordinate2D(bottom_right, object_pose);

  geometry_msgs::Point top_left_map_in_image = util::transformMapToImage(top_left_map, map_info);
  geometry_msgs::Point top_right_map_in_image = util::transformMapToImage(top_right_map, map_info);
  geometry_msgs::Point bottom_left_map_in_image =
    util::transformMapToImage(bottom_left_map, map_info);
  geometry_msgs::Point bottom_right_map_in_image =
    util::transformMapToImage(bottom_right_map, map_info);
  cv::Point top_left_image_point = cv::Point(top_left_map_in_image.x, top_left_map_in_image.y);
  cv::Point top_right_image_point = cv::Point(top_right_map_in_image.x, top_right_map_in_image.y);
  cv::Point bottom_right_image_point =
    cv::Point(bottom_right_map_in_image.x, bottom_right_map_in_image.y);
  cv::Point bottom_left_image_point =
    cv::Point(bottom_left_map_in_image.x, bottom_left_map_in_image.y);
  std::vector<cv::Point> cv_polygon;
  cv_polygon.push_back(top_left_image_point);
  cv_polygon.push_back(top_right_image_point);
  cv_polygon.push_back(bottom_right_image_point);
  cv_polygon.push_back(bottom_left_image_point);
  return cv_polygon;
}

std::vector<cv::Point> getCVPolygonFromPolygon(
  const autoware_perception_msgs::DynamicObject & object, const nav_msgs::MapMetaData & map_info)
{
  std::vector<cv::Point> cv_polygon;
  for (const auto & point : object.shape.footprint.points) {
    geometry_msgs::Point point_in_image = util::transformMapToImage(point, map_info);
    cv::Point image_point = cv::Point(point_in_image.x, point_in_image.y);
    cv_polygon.push_back(image_point);
  }
  return cv_polygon;
}

cv::Mat getOnlyObjectsClearanceMap(
  const cv::Mat & clearance_map,
  const std::vector<autoware_perception_msgs::DynamicObject> & objects,
  const nav_msgs::OccupancyGrid & occupancy_grid, const double max_avoiding_objects_velocity_ms)
{
  cv::Mat objects_image = drawObstalcesOnImage(
    objects, occupancy_grid.info, clearance_map, max_avoiding_objects_velocity_ms);

  cv::Mat only_objects_clearance_map;
  cv::distanceTransform(objects_image, only_objects_clearance_map, cv::DIST_L2, 5);
  return only_objects_clearance_map;
}

cv::Mat getDrivableAreaInCV(const nav_msgs::OccupancyGrid & occupancy_grid)
{
  cv::Mat drivable_area = cv::Mat(occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1);

  drivable_area.forEach<unsigned char>([&](unsigned char & value, const int * position) -> void {
    getOccupancyGridValue(occupancy_grid, position[0], position[1], value);
  });
  return drivable_area;
}

cv::Mat getClearanceMap(
  const nav_msgs::OccupancyGrid & occupancy_grid, const cv::Mat & drivable_area,
  const std::vector<autoware_perception_msgs::DynamicObject> & objects)
{
  cv::Mat clearance_map;
  cv::distanceTransform(drivable_area, clearance_map, cv::DIST_L2, 5);
  return clearance_map;
}
}  // namespace process_cv
