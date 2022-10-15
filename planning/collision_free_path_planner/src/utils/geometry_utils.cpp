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

#include "collision_free_path_planner/utils/geometry_utils.hpp"

#include "collision_free_path_planner/eb_path_optimizer.hpp"
#include "collision_free_path_planner/mpt_optimizer.hpp"
#include "motion_utils/motion_utils.hpp"
#include "tf2/utils.h"

#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

#include "boost/optional.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <stack>
#include <vector>

namespace collision_free_path_planner
{
namespace geometry_utils
{
// TODO(murooka) check if this can be replaced with calcOffsetPose considering calculation time
geometry_msgs::msg::Point transformToAbsoluteCoordinate2D(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin)
{
  // NOTE: implement transformation without defining yaw variable
  //       but directly sin/cos of yaw for fast calculation
  const auto & q = origin.orientation;
  const double cos_yaw = 1 - 2 * q.z * q.z;
  const double sin_yaw = 2 * q.w * q.z;

  geometry_msgs::msg::Point absolute_p;
  absolute_p.x = point.x * cos_yaw - point.y * sin_yaw + origin.position.x;
  absolute_p.y = point.x * sin_yaw + point.y * cos_yaw + origin.position.y;
  absolute_p.z = point.z;

  return absolute_p;
}

geometry_msgs::msg::Quaternion getQuaternionFromPoints(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & a_root)
{
  const double yaw = tier4_autoware_utils::calcAzimuthAngle(a_root, a);
  return tier4_autoware_utils::createQuaternionFromYaw(yaw);
}

geometry_msgs::msg::Quaternion getQuaternionFromPoints(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3, const geometry_msgs::msg::Point & p4)
{
  const double dx = (8.0 * (p3.x - p2.x) - (p4.x - p1.x)) / 12.0;
  const double dy = (8.0 * (p3.y - p2.y) - (p4.y - p1.y)) / 12.0;
  const double yaw = std::atan2(dy, dx);

  return tier4_autoware_utils::createQuaternionFromYaw(yaw);
}

boost::optional<geometry_msgs::msg::Point> transformMapToOptionalImage(
  const geometry_msgs::msg::Point & map_point,
  const nav_msgs::msg::MapMetaData & occupancy_grid_info)
{
  const geometry_msgs::msg::Point relative_p =
    transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
  const double resolution = occupancy_grid_info.resolution;
  const double map_y_height = occupancy_grid_info.height;
  const double map_x_width = occupancy_grid_info.width;
  const double map_x_in_image_resolution = relative_p.x / resolution;
  const double map_y_in_image_resolution = relative_p.y / resolution;
  const double image_x = map_y_height - map_y_in_image_resolution;
  const double image_y = map_x_width - map_x_in_image_resolution;
  if (
    image_x >= 0 && image_x < static_cast<int>(map_y_height) && image_y >= 0 &&
    image_y < static_cast<int>(map_x_width)) {
    geometry_msgs::msg::Point image_point;
    image_point.x = image_x;
    image_point.y = image_y;
    return image_point;
  } else {
    return boost::none;
  }
}

bool transformMapToImage(
  const geometry_msgs::msg::Point & map_point,
  const nav_msgs::msg::MapMetaData & occupancy_grid_info, geometry_msgs::msg::Point & image_point)
{
  geometry_msgs::msg::Point relative_p =
    transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
  const double map_y_height = occupancy_grid_info.height;
  const double map_x_width = occupancy_grid_info.width;
  const double scale = 1 / occupancy_grid_info.resolution;
  const double map_x_in_image_resolution = relative_p.x * scale;
  const double map_y_in_image_resolution = relative_p.y * scale;
  const double image_x = map_y_height - map_y_in_image_resolution;
  const double image_y = map_x_width - map_x_in_image_resolution;
  if (
    image_x >= 0 && image_x < static_cast<int>(map_y_height) && image_y >= 0 &&
    image_y < static_cast<int>(map_x_width)) {
    image_point.x = image_x;
    image_point.y = image_y;
    return true;
  } else {
    return false;
  }
}
}  // namespace geometry_utils
}  // namespace collision_free_path_planner
