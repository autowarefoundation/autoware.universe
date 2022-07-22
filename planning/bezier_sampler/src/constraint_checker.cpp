/*
 * Copyright 2022 Tier IV, Inc. All rights reserved.
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

#include <bezier_sampler/constraint_checker.hpp>

namespace bezier_sampler
{
ConstraintChecker::ConstraintChecker(
  nav_msgs::msg::OccupancyGrid drivable_area, ConstraintParameters parameters)
: drivable_area_(std::move(drivable_area)), params_(parameters)
{
  // Prepare rotation map -> OccupancyGrid cell
  tf2::Quaternion drivable_area_quaternion;
  tf2::convert(drivable_area_.info.origin.orientation, drivable_area_quaternion);
  tf2::Matrix3x3 m(drivable_area_quaternion);
  double roll{};
  double pitch{};
  double yaw{};
  m.getRPY(roll, pitch, yaw);
  drivable_area_rotation_ << std::cos(-yaw), -std::sin(-yaw), std::sin(-yaw), std::cos(-yaw);
  // TODO(Maxime CLEMENT):
  //  build polygon from the OccupancyGrid
  //  Convert to opencv image
  //  Extract contour
  //  Rotate and translate

  // Prepare vectors to the footprint corners;
  left_front_ = {parameters.ego_front_length, parameters.ego_width / 2.0};
  right_front_ = {parameters.ego_front_length, -parameters.ego_width / 2.0};
  left_rear_ = {-parameters.ego_rear_length, parameters.ego_width / 2.0};
  right_rear_ = {-parameters.ego_rear_length, -parameters.ego_width / 2.0};
}
polygon_t ConstraintChecker::buildFootprintPolygon(const Bezier & path) const
{
  // Using the method from Section IV.A of A. Artuñedoet al.: Real-Time Motion Planning Approach for
  // Automated Driving in Urban Environments
  polygon_t footprint;
  // sample points
  // we store the right bound as it needs to be added to the polygon after the left bound
  std::vector<Eigen::Vector2d> right_bound;
  const std::vector<Eigen::Vector3d> points = path.cartesianWithHeading(50);
  // first point: use the left and right point on the rear
  {
    const Eigen::Vector2d first_point = points.front().head<2>();
    double heading = points.front().z();
    Eigen::Matrix2d rotation;
    rotation << std::cos(heading), -std::sin(heading), std::sin(heading), std::cos(heading);
    const Eigen::Vector2d left_rear_point = first_point + rotation * left_rear_;
    bg::append(footprint, left_rear_point);
    right_bound.emplace_back(first_point + rotation * right_rear_);
  }
  // For each points (except 1st and last)
  for (auto it = std::next(points.begin()); it != std::prev(points.end()); ++it) {
    const Eigen::Vector2d point = it->head<2>();
    const double prev_heading = std::prev(it)->z();
    const double heading = it->z();
    Eigen::Matrix2d rotation;
    rotation << std::cos(heading), -std::sin(heading), std::sin(heading), std::cos(heading);
    // We use the change in the heading (restricted in [-pi;pi]) to determine if the path is turning
    // left or right
    const bool turning_right = (std::fmod(heading - prev_heading + M_PI, 2 * M_PI) - M_PI) < 0;
    if (turning_right) {
      const Eigen::Vector2d left_front_point = point + rotation * left_front_;
      bg::append(footprint, left_front_point);
      right_bound.emplace_back(point + rotation * right_rear_);
    } else {
      const Eigen::Vector2d left_rear_point = point + rotation * left_rear_;
      bg::append(footprint, left_rear_point);
      right_bound.emplace_back(point + rotation * right_front_);
    }
  }
  // last point: use the left and right point on the front
  {
    Eigen::Vector2d last_point = points.back().head<2>();
    double heading = points.back().z();
    Eigen::Matrix2d rotation;
    rotation << std::cos(heading), -std::sin(heading), std::sin(heading), std::cos(heading);
    Eigen::Vector2d left_front_point = last_point + rotation * left_front_;
    bg::append(footprint, left_front_point);
    right_bound.emplace_back(last_point + rotation * right_front_);
  }
  for (auto it = right_bound.rbegin(); it != right_bound.rend(); ++it) bg::append(footprint, *it);
  bg::correct(footprint);
  return footprint;
}
bool ConstraintChecker::isDrivable(const Bezier & path) const
{
  const double step = 1.0 / (params_.nb_points - 1);
  for (double t = 0.0; t <= 1.0; t += step) {
    const double curvature = std::abs(path.curvature(t));
    if (curvature >= params_.maximum_curvature) return false;
  }
  return true;
}
bool ConstraintChecker::isCollisionFree(const Bezier & path) const
{
  for (const Eigen::Vector2d & position : buildFootprintPolygon(path).outer()) {
    if (!isCollisionFree(position)) {
      // std::printf("Collision @ (%2.2f, %2.2f)\n", position.x(), position.y());
      return false;
    }
  }
  return true;
}
bool ConstraintChecker::isCollisionFree(const Eigen::Vector2d & position) const
{
  const Eigen::Vector2d local_pose(
    position.x() - drivable_area_.info.origin.position.x,
    position.y() - drivable_area_.info.origin.position.y);
  // const Eigen::Vector2d local_pose_rot = drivable_area_rotation_ * local_pose;
  const Eigen::Vector2i index =
    (drivable_area_rotation_ * local_pose / drivable_area_.info.resolution).cast<int>();
  const size_t flat_index = index.x() + index.y() * drivable_area_.info.width;
  // std::printf("\tPosition (%2.2f,%2.2f)\n", position.x(), position.y());
  // std::printf("\tIndex (%d,%d)\n", index.x(), index.y());
  // std::printf("\tFlat index (%d)\n", flat_index);
  // std::printf("\tArea info (%d x %d)\n", drivable_area_.info.width, drivable_area_.info.height);
  // std::cout << std::endl;
  // Return true if the position is outside of the grid OR if the corresponding cell value is 0
  return index.x() < 0 || index.y() < 0 ||
         index.x() >= static_cast<int>(drivable_area_.info.width) ||
         index.y() >= static_cast<int>(drivable_area_.info.height) ||
         static_cast<int>(drivable_area_.data[flat_index]) == 0;
}
}  // namespace bezier_sampler
