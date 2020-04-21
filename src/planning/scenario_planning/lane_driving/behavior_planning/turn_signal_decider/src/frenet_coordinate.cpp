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
#include <turn_signal_decider/frenet_coordinate.h>
#include <Eigen/Dense>

namespace
{
Eigen::Vector3d convertToEigenPt(const geometry_msgs::Point geom_pt)
{
  return Eigen::Vector3d(geom_pt.x, geom_pt.y, geom_pt.z);
}

std::vector<geometry_msgs::Point> convertToPointArray(
  const autoware_planning_msgs::PathWithLaneId & path)
{
  std::vector<geometry_msgs::Point> point_array;
  for (const auto & pt : path.points) {
    point_array.push_back(pt.point.pose.position);
  }
  return point_array;
}
}  // namespace

namespace turn_signal_decider
{
bool convertToFrenetCoordinate3d(
  const autoware_planning_msgs::PathWithLaneId & path,
  const geometry_msgs::Point & search_point_geom, FrenetCoordinate3d * frenet_coordinate)
{
  const auto linestring = convertToPointArray(path);
  return convertToFrenetCoordinate3d(linestring, search_point_geom, frenet_coordinate);
}

// returns false when search point is off the linestring
bool convertToFrenetCoordinate3d(
  const std::vector<geometry_msgs::Point> & linestring,
  const geometry_msgs::Point & search_point_geom, FrenetCoordinate3d * frenet_coordinate)
{
  if (linestring.empty()) {
    return false;
  }

  const auto search_pt = convertToEigenPt(search_point_geom);
  bool found = false;
  double min_distance = std::numeric_limits<double>::max();

  // get frenet coordinate based on points
  // this is done because linestring is not differentiable at vertices
  {
    double accumulated_length = 0;

    for (std::size_t i = 0; i < linestring.size(); i++) {
      const auto geom_pt = linestring.at(i);
      const auto current_pt = convertToEigenPt(geom_pt);
      const auto current2search_pt = (search_pt - current_pt);
      // update accumulated length
      if (i != 0) {
        const auto p1 = convertToEigenPt(linestring.at(i - 1));
        const auto p2 = current_pt;
        accumulated_length += (p2 - p1).norm();
      }
      // update frenet coordinate

      const double tmp_distance = current2search_pt.norm();
      if (tmp_distance < min_distance) {
        found = true;
        min_distance = tmp_distance;
        frenet_coordinate->distance = tmp_distance;
        frenet_coordinate->length = accumulated_length;
      }
    }
  }

  // get frenet coordinate based on lines
  {
    auto prev_geom_pt = linestring.front();
    double accumulated_length = 0;
    for (const auto & geom_pt : linestring) {
      const auto start_pt = convertToEigenPt(prev_geom_pt);
      const auto end_pt = convertToEigenPt(geom_pt);

      const auto line_segment = end_pt - start_pt;
      const double line_segment_length = line_segment.norm();
      const auto direction = line_segment / line_segment_length;
      const auto start2search_pt = (search_pt - start_pt);

      double tmp_length = direction.dot(start2search_pt);
      if (tmp_length >= 0 && tmp_length <= line_segment_length) {
        double tmp_distance = direction.cross(start2search_pt).norm();
        if (tmp_distance < min_distance) {
          found = true;
          min_distance = tmp_distance;
          frenet_coordinate->distance = tmp_distance;
          frenet_coordinate->length = accumulated_length + tmp_length;
        }
      }
      accumulated_length += line_segment_length;
      prev_geom_pt = geom_pt;
    }
  }
  return found;
}
}  // namespace turn_signal_decider