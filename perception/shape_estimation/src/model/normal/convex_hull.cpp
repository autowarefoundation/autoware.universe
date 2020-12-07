/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 *
 *
 * v1.0 Yukihiro Saito
 */

#include "convex_hull.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "geometry_msgs/msg/point32.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "autoware_perception_msgs/msg/shape.hpp"

namespace normal
{
bool ConvexHullModel::estimate(
  const pcl::PointCloud<pcl::PointXYZ> & cluster, autoware_perception_msgs::msg::Shape & shape_output,
  geometry_msgs::msg::Pose & pose_output)
{
  // calc centroid point for convex hull height(z)
  pcl::PointXYZ centroid;
  centroid.x = 0;
  centroid.y = 0;
  centroid.z = 0;
  for (const auto & pcl_point : cluster) {
    centroid.x += pcl_point.x;
    centroid.y += pcl_point.y;
    centroid.z += pcl_point.z;
  }
  centroid.x = centroid.x / (double)cluster.size();
  centroid.y = centroid.y / (double)cluster.size();
  centroid.z = centroid.z / (double)cluster.size();

  // calc min and max z for convex hull height(z)
  double min_z = 0;
  double max_z = 0;
  for (size_t i = 0; i < cluster.size(); ++i) {
    if (cluster.at(i).z < min_z || i == 0) min_z = cluster.at(i).z;
    if (max_z < cluster.at(i).z || i == 0) max_z = cluster.at(i).z;
  }

  std::vector<cv::Point> v_pointcloud;
  std::vector<cv::Point> v_polygon_points;
  for (size_t i = 0; i < cluster.size(); ++i) {
    v_pointcloud.push_back(
      cv::Point((cluster.at(i).x - centroid.x) * 1000.0, (cluster.at(i).y - centroid.y) * 1000.0));
  }
  cv::convexHull(v_pointcloud, v_polygon_points);

  pcl::PointXYZ polygon_centroid;
  polygon_centroid.x = 0;
  polygon_centroid.y = 0;
  for (size_t i = 0; i < v_polygon_points.size(); ++i) {
    polygon_centroid.x += (double)v_polygon_points.at(i).x / 1000.0;
    polygon_centroid.y += (double)v_polygon_points.at(i).y / 1000.0;
  }
  polygon_centroid.x = polygon_centroid.x / (double)v_polygon_points.size();
  polygon_centroid.y = polygon_centroid.y / (double)v_polygon_points.size();

  for (size_t i = 0; i < v_polygon_points.size(); ++i) {
    geometry_msgs::msg::Point32 point;
    point.x = (double)v_polygon_points.at(i).x / 1000.0 - polygon_centroid.x;
    point.y = (double)v_polygon_points.at(i).y / 1000.0 - polygon_centroid.y;
    point.z = 0.0;
    shape_output.footprint.points.push_back(point);
  }

  constexpr double ep = 0.001;
  shape_output.type = autoware_perception_msgs::msg::Shape::POLYGON;
  pose_output.position.x = centroid.x + polygon_centroid.x;
  pose_output.position.y = centroid.y + polygon_centroid.y;
  pose_output.position.z = centroid.z;
  pose_output.orientation.x = 0;
  pose_output.orientation.y = 0;
  pose_output.orientation.z = 0;
  pose_output.orientation.w = 1;
  shape_output.dimensions.x = 0.0;
  shape_output.dimensions.y = 0.0;
  shape_output.dimensions.z = std::max((max_z - min_z), ep);
  return true;
}
}  // namespace normal
