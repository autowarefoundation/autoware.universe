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

#include "autoware/motion_utils/trajectory_container/trajectory/detail/utils.hpp"

namespace autoware::motion_utils::trajectory_container::trajectory::detail
{

geometry_msgs::msg::Point to_point(const geometry_msgs::msg::Point & p)
{
  return p;
}

geometry_msgs::msg::Point to_point(const geometry_msgs::msg::Pose & p)
{
  return p.position;
}

geometry_msgs::msg::Point to_point(const Eigen::Ref<const Eigen::Vector2d> & p)
{
  geometry_msgs::msg::Point point;
  point.x = p(0);
  point.y = p(1);
  return point;
}

geometry_msgs::msg::Point to_point(const autoware_planning_msgs::msg::PathPoint & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.pose.position.x;
  point.y = p.pose.position.y;
  return point;
}

geometry_msgs::msg::Point to_point(const tier4_planning_msgs::msg::PathPointWithLaneId & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.point.pose.position.x;
  point.y = p.point.pose.position.y;
  return point;
}

geometry_msgs::msg::Point to_point(const lanelet::BasicPoint2d & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.x();
  point.y = p.y();
  return point;
}

geometry_msgs::msg::Point to_point(const lanelet::ConstPoint3d & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.x();
  point.y = p.y();
  return point;
}

Eigen::VectorXd fill_axis(const Eigen::Ref<const Eigen::VectorXd> & x, Eigen::Index min_points)
{
  const auto original_size = x.size();

  if (original_size >= min_points) {
    return x;
  }

  const auto points_to_add = min_points - original_size;
  const auto num_gaps = original_size - 1;

  std::vector<Eigen::Index> points_per_gap(num_gaps, points_to_add / num_gaps);
  std::fill_n(points_per_gap.begin(), points_to_add % num_gaps, points_per_gap[0] + 1);

  Eigen::VectorXd result(min_points);
  Eigen::Index index = 0;

  for (Eigen::Index i = 0; i < original_size - 1; ++i) {
    result[index++] = x[i];

    const double start = x[i];
    const double end = x[i + 1];
    const double step = (end - start) / static_cast<int16_t>(points_per_gap[i] + 1);

    for (Eigen::Index j = 0; j < points_per_gap[i]; ++j) {
      result[index++] = start + static_cast<int16_t>(j + 1) * step;
    }
  }

  result[index] = x[original_size - 1];

  return result;
}

Eigen::VectorXd crop_axis(
  const Eigen::Ref<const Eigen::VectorXd> & x, const double & start, const double & end)
{
  std::vector<double> result;

  // Add start point if it's not already in x
  if (std::find(x.begin(), x.end(), start) == x.end()) {
    result.push_back(start);
  }

  // Copy all points within the range [start, end]
  for (double i : x) {
    if (i >= start && i <= end) {
      result.push_back(i);
    }
  }

  // Add end point if it's not already in x
  if (std::find(x.begin(), x.end(), end) == x.end()) {
    result.push_back(end);
  }

  // Convert std::vector to Eigen::VectorXd
  return Eigen::Map<Eigen::VectorXd>(result.data(), static_cast<Eigen::Index>(result.size()));
}

}  // namespace autoware::motion_utils::trajectory_container::trajectory::detail
