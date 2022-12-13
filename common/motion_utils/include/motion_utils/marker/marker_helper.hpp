// Copyright 2021 Tier IV, Inc.
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

#ifndef MOTION_UTILS__MARKER__MARKER_HELPER_HPP_
#define MOTION_UTILS__MARKER__MARKER_HELPER_HPP_

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <functional>
#include <string>
#include <vector>

namespace motion_utils
{
using geometry_msgs::msg::Pose;
using visualization_msgs::msg::MarkerArray;

visualization_msgs::msg::MarkerArray createStopVirtualWallMarker(
  const Pose & pose, const std::string & module_name, const rclcpp::Time & now, const int32_t id,
  const double longitudinal_offset = 0.0);

visualization_msgs::msg::MarkerArray createSlowDownVirtualWallMarker(
  const Pose & pose, const std::string & module_name, const rclcpp::Time & now, const int32_t id,
  const double longitudinal_offset = 0.0);

visualization_msgs::msg::MarkerArray createDeadLineVirtualWallMarker(
  const Pose & pose, const std::string & module_name, const rclcpp::Time & now, const int32_t id,
  const double longitudinal_offset = 0.0);

visualization_msgs::msg::MarkerArray createDeletedStopVirtualWallMarker(
  const rclcpp::Time & now, const int32_t id);

visualization_msgs::msg::MarkerArray createDeletedSlowDownVirtualWallMarker(
  const rclcpp::Time & now, const int32_t id);

visualization_msgs::msg::MarkerArray createDeletedDeadLineVirtualWallMarker(
  const rclcpp::Time & now, const int32_t id);

visualization_msgs::msg::MarkerArray createDeletedStopVirtualWallMarkerFromStopPoses(
  const std::vector<Pose> & stop_poses, const std::vector<Pose> & stopped_poses,
  const rclcpp::Time & now, int32_t id);

class VirtualWallMarkerCreator
{
public:
  virtual ~VirtualWallMarkerCreator() = default;

private:
  visualization_msgs::msg::MarkerArray base(
    const std::vector<Pose> & poses, const std::string & module_name, const rclcpp::Time & now,
    int32_t id, std::function<MarkerArray(
      const geometry_msgs::msg::Pose & pose, const std::string & module_name,
      const rclcpp::Time & now, const int32_t id, const double longitudinal_offset)>
      function_create_wall_marker,
    std::function<MarkerArray(const rclcpp::Time & now, const int32_t id)>
      function_delete_wall_marker,
    std::vector<geometry_msgs::msg::Pose> & previous_poses,const double longitudinal_offset = 0.0);

  visualization_msgs::msg::MarkerArray createStopVirtualWallMarker(
    const std::vector<Pose> & stop_pose, const std::string & module_name, const rclcpp::Time & now,
    int32_t id, const double longitudinal_offset = 0.0);
  std::vector<Pose> previous_stop_pose_;
  std::vector<Pose> previous_slowdown_poses_;
};
}  // namespace motion_utils

#endif  // MOTION_UTILS__MARKER__MARKER_HELPER_HPP_
