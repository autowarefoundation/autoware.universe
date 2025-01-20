// Copyright 2025 TIER IV, Inc.
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

#include "autoware/control_evaluator/metrics/metrics_utils.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/geometry.hpp>
// #include <boost/geometry/algorithms/detail/distance/interface.hpp>

#include <lanelet2_core/Forward.h>
namespace control_diagnostics
{
namespace metrics
{
namespace utils
{
using autoware::route_handler::RouteHandler;
using geometry_msgs::msg::Pose;

lanelet::ConstLanelets get_current_lanes(const RouteHandler & route_handler, const Pose & ego_pose)
{
  lanelet::ConstLanelet closest_route_lanelet;
  route_handler.getClosestLaneletWithinRoute(ego_pose, &closest_route_lanelet);
  const auto shoulder_lanelets = route_handler.getShoulderLaneletsAtPose(ego_pose);
  lanelet::ConstLanelets closest_lanelets{closest_route_lanelet};
  closest_lanelets.insert(
    closest_lanelets.end(), shoulder_lanelets.begin(), shoulder_lanelets.end());
  return closest_lanelets;
}

double calc_distance_to_line(
  const autoware::universe_utils::LinearRing2d & vehicle_footprint,
  const autoware::universe_utils::LineString2d & line)
{
  return boost::geometry::distance(vehicle_footprint, line);
}

double calc_yaw_to_line(const Pose & ego_pose, const autoware::universe_utils::LineString2d & line)
{
  const double ego_yaw = tf2::getYaw(ego_pose.orientation);

  // find nearest point on the line.
  double nearest_pt_x = ego_pose.position.x;
  double nearest_pt_y = ego_pose.position.y;
  double min_dist = std::numeric_limits<double>::max();
  for (const auto & pt : line) {
    const double dist = std::hypot(pt.x() - ego_pose.position.x, pt.y() - ego_pose.position.y);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_pt_x = pt.x();
      nearest_pt_y = pt.y();
    }
  }

  const double ego2line_yaw =
    std::atan2(nearest_pt_y - ego_pose.position.y, nearest_pt_x - ego_pose.position.x);

  double yaw_diff = ego2line_yaw - ego_yaw;
  while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
  while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

  return yaw_diff;
}

}  // namespace utils
}  // namespace metrics
}  // namespace control_diagnostics
