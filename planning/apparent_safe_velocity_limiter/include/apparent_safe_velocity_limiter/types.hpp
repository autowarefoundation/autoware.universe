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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__TYPES_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__TYPES_HPP_

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_linestring.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <functional>
#include <vector>

namespace apparent_safe_velocity_limiter
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::OccupancyGrid;
using PointCloud = sensor_msgs::msg::PointCloud2;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using Float = decltype(TrajectoryPoint::longitudinal_velocity_mps);

enum ObstacleType { POINTCLOUD, OCCUPANCYGRID };

using point_t = boost::geometry::model::d2::point_xy<double>;
using polygon_t = boost::geometry::model::polygon<point_t>;
using multipolygon_t = boost::geometry::model::multi_polygon<polygon_t>;
using segment_t = boost::geometry::model::segment<point_t>;
using linestring_t = boost::geometry::model::linestring<point_t>;
using multilinestring_t = boost::geometry::model::multi_linestring<linestring_t>;

using ForwardProjectionFunction = std::function<segment_t(const TrajectoryPoint &)>;
}  // namespace apparent_safe_velocity_limiter
#endif  // APPARENT_SAFE_VELOCITY_LIMITER__TYPES_HPP_
