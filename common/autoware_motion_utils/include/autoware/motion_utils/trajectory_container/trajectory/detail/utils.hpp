// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__DETAIL__UTILS_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__DETAIL__UTILS_HPP_

#include "lanelet2_core/primitives/Point.h"

#include <Eigen/Core>

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <set>
#include <vector>

namespace autoware::motion_utils::trajectory_container::trajectory::detail
{
/**
 * @brief Convert various point types to geometry_msgs::msg::Point.
 * @param p The input point to be converted.
 * @return geometry_msgs::msg::Point The converted point.
 */
geometry_msgs::msg::Point to_point(const geometry_msgs::msg::Point & p);
geometry_msgs::msg::Point to_point(const geometry_msgs::msg::Pose & p);
geometry_msgs::msg::Point to_point(const Eigen::Ref<const Eigen::Vector2d> & p);
geometry_msgs::msg::Point to_point(const autoware_planning_msgs::msg::PathPoint & p);
geometry_msgs::msg::Point to_point(const tier4_planning_msgs::msg::PathPointWithLaneId & p);
geometry_msgs::msg::Point to_point(const lanelet::BasicPoint2d & p);
geometry_msgs::msg::Point to_point(const lanelet::ConstPoint3d & p);

/**
 * @brief Merge multiple vectors into one, keeping only unique elements.
 * @tparam Vectors Variadic template parameter for vector types.
 * @param vectors Vectors to be merged.
 * @return Eigen::VectorXd Merged vector with unique elements.
 */
template <typename... Vectors>
Eigen::VectorXd merge_vectors(const Vectors &... vectors)
{
  std::set<double> unique_elements;

  // Helper function to insert elements into the set
  auto insert_elements = [&unique_elements](const auto & vec) {
    unique_elements.insert(vec.begin(), vec.end());
  };

  // Expand the parameter pack and insert elements from each vector
  (insert_elements(vectors), ...);

  // Convert the set to Eigen::VectorXd
  return Eigen::Map<Eigen::VectorXd>(
    std::vector<double>(unique_elements.begin(), unique_elements.end()).data(),
    static_cast<Eigen::Index>(unique_elements.size()));
}

/**
 * @brief Fill the given axis with additional points to meet the minimum point requirement.
 * @param x The input vector to be filled.
 * @param min_points The minimum number of points required.
 * @return Eigen::VectorXd The filled vector.
 */
Eigen::VectorXd fill_axis(const Eigen::Ref<const Eigen::VectorXd> & x, Eigen::Index min_points);

/**
 * @brief Crop the given axis between the specified start and end values.
 * @param x The input vector to be cropped.
 * @param start The start value for cropping.
 * @param end The end value for cropping.
 * @return Eigen::VectorXd The cropped vector.
 */
Eigen::VectorXd crop_axis(
  const Eigen::Ref<const Eigen::VectorXd> & x, const double & start, const double & end);

}  // namespace autoware::motion_utils::trajectory_container::trajectory::detail

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__DETAIL__UTILS_HPP_
