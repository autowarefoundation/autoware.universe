// Copyright 2024 Tier IV, Inc.
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

#include "motion_utils/trajectory_v2/trajectory/trajectory_pose.hpp"

#include "motion_utils/trajectory_v2/interpolator/linear.hpp"

namespace motion_utils::trajectory_v2::trajectory
{

TrajectoryV2<geometry_msgs::msg::Pose>::TrajectoryV2() : BaseClass()
{
  set_orientation_interpolator(interpolator::Linear());
}

TrajectoryV2<geometry_msgs::msg::Pose> &
TrajectoryV2<geometry_msgs::msg::Pose>::set_orientation_interpolator(
  const interpolator::Interpolator<double> & interpolator)
{
  orientation_x_interpolator_ =
    std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
  orientation_y_interpolator_ =
    std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
  orientation_z_interpolator_ =
    std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
  orientation_w_interpolator_ =
    std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
  return *this;
}

TrajectoryV2<geometry_msgs::msg::Pose> & TrajectoryV2<geometry_msgs::msg::Pose>::build(
  const std::vector<geometry_msgs::msg::Pose> & points)
{
  std::vector<geometry_msgs::msg::Point> path_points;
  for (const auto & point : points) {
    path_points.emplace_back(point.position);
  }
  BaseClass::build(path_points);

  std::vector<double> xs, ys, zs, ws;

  for (const auto & point : points) {
    xs.emplace_back(point.orientation.x);
    ys.emplace_back(point.orientation.y);
    zs.emplace_back(point.orientation.z);
    ws.emplace_back(point.orientation.w);
  }

  orientation_x_interpolator_->build(axis_, xs);
  orientation_y_interpolator_->build(axis_, ys);
  orientation_z_interpolator_->build(axis_, zs);
  orientation_w_interpolator_->build(axis_, ws);

  return *this;
}

geometry_msgs::msg::Pose TrajectoryV2<geometry_msgs::msg::Pose>::compute(const double & s) const
{
  geometry_msgs::msg::Pose result;
  result.position = BaseClass::compute(s);
  result.orientation.x = orientation_x_interpolator_->compute(s);
  result.orientation.y = orientation_y_interpolator_->compute(s);
  result.orientation.z = orientation_z_interpolator_->compute(s);
  result.orientation.w = orientation_w_interpolator_->compute(s);
  return result;
}

std::vector<geometry_msgs::msg::Pose> TrajectoryV2<geometry_msgs::msg::Pose>::restore() const
{
  std::vector<geometry_msgs::msg::Pose> points;
  for (auto & s : axis_) {
    geometry_msgs::msg::Pose p;
    p.position = BaseClass::compute(s);
    p.orientation.x = orientation_x_interpolator_->compute(s);
    p.orientation.y = orientation_y_interpolator_->compute(s);
    p.orientation.z = orientation_z_interpolator_->compute(s);
    p.orientation.w = orientation_w_interpolator_->compute(s);
    points.push_back(p);
  }
  return points;
}

}  // namespace motion_utils::trajectory_v2::trajectory
