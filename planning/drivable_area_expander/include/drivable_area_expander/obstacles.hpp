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

#ifndef DRIVABLE_AREA_EXPANDER__OBSTACLES_HPP_
#define DRIVABLE_AREA_EXPANDER__OBSTACLES_HPP_

#include "drivable_area_expander/parameters.hpp"
#include "drivable_area_expander/types.hpp"

#include <tier4_autoware_utils/ros/transform_listener.hpp>

#include "autoware_auto_planning_msgs/msg/detail/path_point__struct.hpp"
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/overlaps.hpp>
#include <boost/geometry/index/predicates.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace drivable_area_expander
{
/// @brief rotate a polygon by some angle around (0,0)
/// @param[in] polygon input polygon
/// @param[in] angle angle of rotation
/// @return rotated polygon
polygon_t rotatePolygon(const polygon_t & polygon, const double angle);

/// @brief translate a polygon by some (x,y) vector
/// @param[in] polygon input polygon
/// @param[in] x translation distance on the x axis
/// @param[in] y translation distance on the y axis
/// @return translated polygon
polygon_t translatePolygon(const polygon_t & polygon, const double x, const double y);

/// @brief wrapper to extract a pose
inline const geometry_msgs::msg::Pose & getPose(const geometry_msgs::msg::Pose & pose)
{
  return pose;
};
/// @brief wrapper to extract a pose from a PathPoint
inline const geometry_msgs::msg::Pose & getPose(
  const autoware_auto_planning_msgs::msg::PathPoint & path_point)
{
  return path_point.pose;
}

/// @brief create the footprint polygon from a path
/// @param[in] path the path for which to create a footprint
/// @param[in] params expansion parameters defining how to create the footprint
/// @return polygon footprint of the path
/// @tparam path type (sequence of PathPoint or sequence of Pose)
template <class T>
multipolygon_t createFootprintPolygons(
  const T & path, const double front, const double rear, const double left, const double right);

/// @brief create the footprint polygon from a path
/// @param[in] path the path for which to create a footprint
/// @param[in] params expansion parameters defining how to create the footprint
/// @return polygon footprint of the path
multipolygon_t createPathFootprint(const Path & path, const ExpansionParameters & params);

/// @brief create polygons from the predicted paths of an object
/// @param [in] objects objects from which to create polygons
/// @param [in] buffer buffer to add to the objects dimensions
/// @param [in] min_velocity objects with velocity lower will be ignored
/// @return polygons of the object's predicted paths
multipolygon_t createObjectFootprints(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects, const double buffer,
  double min_velocity);
}  // namespace drivable_area_expander
#endif  // DRIVABLE_AREA_EXPANDER__OBSTACLES_HPP_
