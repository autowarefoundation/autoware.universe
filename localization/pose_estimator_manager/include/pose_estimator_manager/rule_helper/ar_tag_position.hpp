// Copyright 2023 Autoware Foundation
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

#ifndef POSE_ESTIMATOR_MANAGER__RULE_HELPER__AR_TAG_POSITION_HPP_
#define POSE_ESTIMATOR_MANAGER__RULE_HELPER__AR_TAG_POSITION_HPP_

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace multi_pose_estimator
{
// This class finds the distance to the nearest landmark for AR tag based localization.
class ArTagPosition
{
public:
  using TransformStamped = geometry_msgs::msg::TransformStamped;

  explicit ArTagPosition(rclcpp::Node * node);

  double distance_to_nearest_ar_tag_around_ego(const geometry_msgs::msg::Point & ego_point) const;

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
  rclcpp::Logger logger_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  std::optional<TransformStamped> get_transform(
    const std::string & target_frame, const std::string & source_frame) const;
};
}  // namespace multi_pose_estimator

#endif  // POSE_ESTIMATOR_MANAGER__RULE_HELPER__AR_TAG_POSITION_HPP_
