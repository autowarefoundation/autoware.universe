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

#ifndef TIER4_AUTOWARE_UTILS__GEOMETRY__BOOST_POLYGON_UTILS_HPP_
#define TIER4_AUTOWARE_UTILS__GEOMETRY__BOOST_POLYGON_UTILS_HPP_

#include "tier4_autoware_utils/geometry/boost_geometry.hpp"

#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <vector>

// cppcheck-suppress unknownMacro
BOOST_GEOMETRY_REGISTER_POINT_3D(geometry_msgs::msg::Point, double, cs::cartesian, x, y, z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  geometry_msgs::msg::Pose, double, cs::cartesian, position.x, position.y, position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  geometry_msgs::msg::PoseWithCovarianceStamped, double, cs::cartesian, pose.pose.position.x,
  pose.pose.position.y, pose.pose.position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  autoware_auto_planning_msgs::msg::PathPoint, double, cs::cartesian, pose.position.x,
  pose.position.y, pose.position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  autoware_auto_planning_msgs::msg::PathPointWithLaneId, double, cs::cartesian,
  point.pose.position.x, point.pose.position.y, point.pose.position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  autoware_auto_planning_msgs::msg::TrajectoryPoint, double, cs::cartesian, pose.position.x,
  pose.position.y, pose.position.z)

namespace tier4_autoware_utils
{
bool isClockwise(const Polygon2d & polygon);
Polygon2d inverseClockwise(const Polygon2d & polygon);
geometry_msgs::msg::Polygon rotatePolygon(
  const geometry_msgs::msg::Polygon & polygon, const double & angle);
/// @brief rotate a polygon by some angle around the origin
/// @param[in] polygon input polygon
/// @param[in] angle angle of rotation [rad]
/// @return rotated polygon
Polygon2d rotatePolygon(const Polygon2d & polygon, const double angle);
Polygon2d toPolygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_auto_perception_msgs::msg::Shape & shape);
Polygon2d toPolygon2d(const lanelet::ConstLanelet & lanelet);
Polygon2d toPolygon2d(const lanelet::BasicPolygon2d & polygon);
Polygon2d toPolygon2d(const autoware_auto_perception_msgs::msg::DetectedObject & object);
Polygon2d toPolygon2d(const autoware_auto_perception_msgs::msg::TrackedObject & object);
Polygon2d toPolygon2d(const autoware_auto_perception_msgs::msg::PredictedObject & object);
Polygon2d toFootprint(
  const geometry_msgs::msg::Pose & base_link_pose, const double base_to_front,
  const double base_to_rear, const double width);
double getArea(const autoware_auto_perception_msgs::msg::Shape & shape);
Polygon2d expandPolygon(const Polygon2d & input_polygon, const double offset);

namespace bg = boost::geometry;

using Point2d = tier4_autoware_utils::Point2d;
using LineString2d = tier4_autoware_utils::LineString2d;
using Polygon2d = tier4_autoware_utils::Polygon2d;

template <class T>
Point2d to_bg2d(const T & p)
{
  return Point2d(bg::get<0>(p), bg::get<1>(p));
}

template <class T>
LineString2d to_bg2d(const std::vector<T> & vec)
{
  LineString2d ps;
  for (const auto & p : vec) {
    ps.push_back(to_bg2d(p));
  }
  return ps;
}

Polygon2d lines2polygon(const LineString2d & left_line, const LineString2d & right_line);

Polygon2d upScalePolygon(
  const geometry_msgs::msg::Point & position, const Polygon2d & polygon, const double scale);

geometry_msgs::msg::Polygon toGeomPoly(const Polygon2d & polygon);
}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__GEOMETRY__BOOST_POLYGON_UTILS_HPP_
