// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef SAMPLER_NODE__UTILS__OCCUPANCY_GRID_TO_POLYGONS_HPP_
#define SAMPLER_NODE__UTILS__OCCUPANCY_GRID_TO_POLYGONS_HPP_

#include "sampler_common/structures.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <boost/assign/list_of.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

#include <opencv2/imgproc/imgproc_c.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

// TODO(Maxime CLEMENT): move code to .cpp file

namespace sampler_node::utils
{
inline std::vector<sampler_common::Polygon> occupancyGridToPolygons(
  const nav_msgs::msg::OccupancyGrid & og)
{
  // Convert to CV format
  cv::Mat drivable_area =
    cv::Mat(static_cast<int>(og.info.width), static_cast<int>(og.info.height), CV_8UC1);
  drivable_area.forEach<unsigned char>([&](unsigned char & value, const int * position) -> void {
    const auto i_flip = og.info.width - position[0] - 1;
    const auto j_flip = og.info.height - position[1] - 1;
    if (og.data[i_flip + j_flip * og.info.width] > 0) {
      value = 255;
    } else {
      value = 0;
    }
  });
  // Get contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(
    drivable_area, contours, cv::RETR_LIST, cv::ContourApproximationModes::CHAIN_APPROX_TC89_KCOS);

  // Get polygons
  const sampler_common::Point origin = {og.info.origin.position.x, og.info.origin.position.y};
  double yaw{};
  double pitch{};
  double roll{};
  tf2::getEulerYPR(og.info.origin.orientation, yaw, pitch, roll);
  std::vector<sampler_common::Polygon> polygons;
  polygons.reserve(contours.size());
  for (const auto & cv_poly : contours) {
    auto & polygon = polygons.emplace_back();
    polygon.outer().reserve(cv_poly.size());
    for (const auto & cv_point : cv_poly) {
      // Shift and scale (CV convention to ROS convention)
      const auto x = static_cast<float>(og.info.width - cv_point.y) * og.info.resolution;
      const auto y = static_cast<float>(og.info.height - cv_point.x) * og.info.resolution;
      // Rotate
      const auto map_x = x * std::cos(yaw) - y * std::sin(yaw);
      const auto map_y = x * std::sin(yaw) + y * std::cos(yaw);
      polygon.outer().emplace_back(origin.x() + map_x, origin.y() + map_y);
    }
  }
  return polygons;
}

inline sampler_common::Polygon createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & size)
{
  // rename
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double h = size.x;
  const double w = size.y;
  const double yaw = tf2::getYaw(pose.orientation);

  // create base polygon
  sampler_common::Polygon obj_poly;
  boost::geometry::exterior_ring(obj_poly) = boost::assign::list_of<sampler_common::Point>(
    h / 2.0, w / 2.0)(-h / 2.0, w / 2.0)(-h / 2.0, -w / 2.0)(h / 2.0, -w / 2.0)(h / 2.0, w / 2.0);

  // rotate polygon(yaw)
  boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>
    rotate(-yaw);  // anti-clockwise -> :clockwise rotation
  sampler_common::Polygon rotate_obj_poly;
  boost::geometry::transform(obj_poly, rotate_obj_poly, rotate);

  // translate polygon(x, y)
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(x, y);
  sampler_common::Polygon translate_obj_poly;
  boost::geometry::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}

inline std::vector<sampler_common::Polygon> predictedObjectsToPolygons(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects)
{
  std::vector<sampler_common::Polygon> polygons;
  polygons.reserve(objects.objects.size());
  for (const auto & object : objects.objects) {
    auto & polygon = polygons.emplace_back();
    if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
      for (const auto & polygon_point : object.shape.footprint.points) {
        polygon.outer().push_back({polygon_point.x, polygon_point.y});
      }
    } else {
      polygon = createObjPolygon(
        object.kinematics.initial_pose_with_covariance.pose, object.shape.dimensions);
    }
    // TODO(Maxime CLEMENT): extend the polygon with its predicted path
  }
  return polygons;
}
}  // namespace sampler_node::utils

#endif  // SAMPLER_NODE__UTILS__OCCUPANCY_GRID_TO_POLYGONS_HPP_
