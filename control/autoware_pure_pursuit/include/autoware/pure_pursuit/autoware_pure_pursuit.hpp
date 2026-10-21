// Copyright 2020 Tier IV, Inc.
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
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_HPP_
#define AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace autoware::pure_pursuit
{
class PurePursuit
{
public:
  PurePursuit() : lookahead_distance_(0.0), closest_thr_dist_(3.0), closest_thr_ang_(M_PI / 4) {}
  ~PurePursuit() = default;

  /// Why run() failed, with the values that the log line about the failure carries.
  struct Failure
  {
    enum class Reason {
      data_not_ready,
      closest_waypoint_not_found,
      lane_direction_unknown,
      next_waypoint_not_found,
      waypoint_interval_too_short,
      lateral_error_beyond_lookahead
    };
    Reason reason{Reason::data_not_ready};
    int32_t closest_index{0};        // for closest_waypoint_not_found
    double lateral_error{0.0};       // for lateral_error_beyond_lookahead
    double lookahead_distance{0.0};  // for lateral_error_beyond_lookahead
  };

  /// What run() returns. The caller writes the log line about a failure.
  struct PpResult
  {
    bool success{false};
    double curvature{0.0};
    Failure failure{};  // valid when success is false
  };

  // setter
  void setCurrentPose(const geometry_msgs::msg::Pose & msg);
  void setWaypoints(const std::vector<geometry_msgs::msg::Pose> & msg);
  void setLookaheadDistance(double ld) { lookahead_distance_ = ld; }
  void setClosestThreshold(double closest_thr_dist, double closest_thr_ang)
  {
    closest_thr_dist_ = closest_thr_dist;
    closest_thr_ang_ = closest_thr_ang;
  }

  // getter
  geometry_msgs::msg::Point getLocationOfNextWaypoint() const { return loc_next_wp_; }
  geometry_msgs::msg::Point getLocationOfNextTarget() const { return loc_next_tgt_; }

  bool isDataReady();
  PpResult run();  // calculate curvature

private:
  // variables for debug
  geometry_msgs::msg::Point loc_next_wp_;
  geometry_msgs::msg::Point loc_next_tgt_;

  // variables got from outside
  double lookahead_distance_, closest_thr_dist_, closest_thr_ang_;
  std::shared_ptr<std::vector<geometry_msgs::msg::Pose>> curr_wps_ptr_;
  std::shared_ptr<geometry_msgs::msg::Pose> curr_pose_ptr_;

  // functions
  int32_t findNextPointIdx(int32_t search_start_idx, PpResult & result);
  std::pair<bool, geometry_msgs::msg::Point> lerpNextTarget(int32_t next_wp_idx, PpResult & result);
};

}  // namespace autoware::pure_pursuit

#endif  // AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_HPP_
