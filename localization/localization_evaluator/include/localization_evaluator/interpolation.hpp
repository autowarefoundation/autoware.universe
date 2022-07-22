// Copyright 2015-2019 Autoware Foundation
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

#ifndef LOCALIZATION_EVALUATOR__INTERPOLATION_HPP_
#define LOCALIZATION_EVALUATOR__INTERPOLATION_HPP_

#include <eigen3/Eigen/Geometry>
#include <rclcpp/time.hpp>

namespace interpolation
{
double getTimeCoeffients(
  rclcpp::Time & curr_time, rclcpp::Time & prev_time, rclcpp::Time & post_time)
{
  auto duration = post_time - prev_time;
  auto step = curr_time - prev_time;
  return step.nanoseconds() / static_cast<double>(duration.nanoseconds());
}

void interpolateTransform(
  double t, Eigen::Affine3d & prev_pose, Eigen::Affine3d & post_pose, Eigen::Affine3d & out_pose)
{
  Eigen::Vector3d prev_translation = prev_pose.translation();
  Eigen::Vector3d post_translation = post_pose.translation();
  Eigen::Translation3d out_translation((1 - t) * prev_translation + t * post_translation);
  Eigen::Quaterniond prev_q(prev_pose.rotation());
  Eigen::Quaterniond post_q(post_pose.rotation());
  Eigen::Quaterniond out_q = prev_q.slerp(t, post_q);
  out_pose = out_translation * out_q;
}
}  // namespace interpolation
#endif  // LOCALIZATION_EVALUATOR__INTERPOLATION_HPP_
