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

#ifndef LANDMARK_MANAGER__LANDMARK_MANAGER_HPP_
#define LANDMARK_MANAGER__LANDMARK_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <string>

class LandmarkManager
{
public:
  void parse_landmark(
    const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr & msg,
    const std::string & target_subtype, const rclcpp::Logger & logger);

  visualization_msgs::msg::MarkerArray get_landmarks_marker_array_msg() const;

  std::optional<geometry_msgs::msg::Pose> convert_landmark_pose_to_ego_pose(
    const geometry_msgs::msg::Pose & base_link_to_landmark, const std::string & landmark_id) const;

private:
  // Landmark is held as a map of <landmark_name(std::string), Pose>
  std::map<std::string, geometry_msgs::msg::Pose> landmarks_;
};

#endif  // LANDMARK_MANAGER__LANDMARK_MANAGER_HPP_
