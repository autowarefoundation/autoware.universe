// Copyright 2022 TIER IV, Inc.
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

#ifndef IMAGE_PROJECTION_BASED_FUSION__UTILS__GEOMETRY_HPP_
#define IMAGE_PROJECTION_BASED_FUSION__UTILS__GEOMETRY_HPP_

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <perception_utils/matching.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include <vector>

namespace image_projection_based_fusion
{

using autoware_auto_perception_msgs::msg::Shape;
using geometry_msgs::msg::Pose;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

struct PolygonRoi
{
  double min_x{0.0}, min_y{0.0}, max_x{0.0}, max_y{0.0};
  Polygon2d roi;

  explicit PolygonRoi(const Polygon2d & _roi) : roi(_roi)
  {
    for (const auto & point : roi.outer()) {
      if (point.x() < min_x) {
        min_x = point.x();
      }
      if (point.y() < min_y) {
        min_y = point.y();
      }
      if (max_x < point.x()) {
        max_x = point.x();
      }
      if (max_y < point.y()) {
        max_y = point.y();
      }
    }
  }
};

double calcIoU(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2);

double calcIoU(const PolygonRoi & roi_1, const PolygonRoi & roi_2);

double calcIoUX(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2);

double calcIoUX(const PolygonRoi & roi_1, const PolygonRoi & roi_2);

double calcIoUY(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2);

double calcIoUY(const PolygonRoi & roi_1, const PolygonRoi & roi_2);

void objectToVertices(
  const Pose & pose, const Shape & shape, std::vector<Eigen::Vector3d> & vertices);

void boundingBoxToVertices(
  const Pose & pose, const Shape & shape, std::vector<Eigen::Vector3d> & vertices);

void cylinderToVertices(
  const Pose & pose, const Shape & shape, std::vector<Eigen::Vector3d> & vertices);

void polygonToVertices(
  const Pose & pose, const Shape & shape, std::vector<Eigen::Vector3d> & vertices);

void transformPoints(
  const std::vector<Eigen::Vector3d> & input_points, const Eigen::Affine3d & affine_transform,
  std::vector<Eigen::Vector3d> & output_points);

Polygon2d roi2Polygon(const sensor_msgs::msg::RegionOfInterest & roi);

Polygon2d point2ConvexHull(const std::vector<Eigen::Vector2d> & points);
}  // namespace image_projection_based_fusion

#endif  // IMAGE_PROJECTION_BASED_FUSION__UTILS__GEOMETRY_HPP_
