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
#pragma once

#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <autoware_planning_msgs/PathPoint.h>
#include <autoware_planning_msgs/PathPointWithLaneId.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>

using Point2d = boost::geometry::model::d2::point_xy<double>;
using Segment2d = boost::geometry::model::segment<Point2d>;
using LineString2d = boost::geometry::model::linestring<Point2d>;
using Polygon2d =
  boost::geometry::model::polygon<Point2d, false, false>;  // counter-clockwise, open

BOOST_GEOMETRY_REGISTER_POINT_3D(geometry_msgs::Point, double, cs::cartesian, x, y, z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  geometry_msgs::PoseWithCovarianceStamped, double, cs::cartesian, pose.pose.position.x,
  pose.pose.position.y, pose.pose.position.z)

BOOST_GEOMETRY_REGISTER_POINT_3D(
  autoware_planning_msgs::PathPoint, double, cs::cartesian, pose.position.x, pose.position.y,
  pose.position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  autoware_planning_msgs::PathPointWithLaneId, double, cs::cartesian, point.pose.position.x,
  point.pose.position.y, point.pose.position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  autoware_planning_msgs::TrajectoryPoint, double, cs::cartesian, pose.position.x, pose.position.y,
  pose.position.z)

template <class T>
Point2d to_bg2d(const T & p)
{
  return Point2d(boost::geometry::get<0>(p), boost::geometry::get<1>(p));
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

inline Polygon2d linestring2polygon(const LineString2d & line_string)
{
  Polygon2d polygon;

  for (const auto & p : line_string) {
    polygon.outer().push_back(p);
  }

  return polygon;
}

inline Polygon2d lines2polygon(const LineString2d & left_line, const LineString2d & right_line)
{
  Polygon2d polygon;

  polygon.outer().push_back(left_line.front());

  for (auto itr = right_line.begin(); itr != right_line.end(); ++itr) {
    polygon.outer().push_back(*itr);
  }

  for (auto itr = left_line.rbegin(); itr != left_line.rend(); ++itr) {
    polygon.outer().push_back(*itr);
  }

  return polygon;
}

inline std::vector<Segment2d> makeSegments(const LineString2d & ls)
{
  std::vector<Segment2d> segments;
  for (size_t i = 0; i < ls.size(); ++i) {
    segments.emplace_back(ls.at(i), ls.at(i + 1));
  }
  return segments;
}
