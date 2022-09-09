// Copyright 2020 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include "scene_module/speed_bump/scene.hpp"

#include "motion_utils/motion_utils.hpp"
#include "scene_module/crosswalk/util.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace behavior_velocity_planner
{
using motion_utils::calcLongitudinalOffsetPoint;
using tier4_autoware_utils::createPoint;

namespace
{
}  // namespace

SpeedBumpModule::SpeedBumpModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::SpeedBump & speed_bump_reg_elem, const PlannerParam & planner_param,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  module_id_(module_id),
  lane_id_(lane_id),
  speed_bump_reg_elem_(std::move(speed_bump_reg_elem)),
  planner_param_(planner_param),
  state_(State::SLOW_DOWN),
  passed_slow_start_point_(false)
{
  // point.x : height [m] -- point.y : speed [m/s]
  Point32 p1;
  Point32 p2;

  p1.x = 0.05;  // min speed bump height
  p2.x = 0.30;  // max speed bump height

  p1.y = 2.78;  // [10 kph] max speed for any speed bump
  p2.y = 1.39;  // [5 kph] min speed for any speed bump

  auto const & constants = getLinearEquation(p1, p2);
  auto const & m = constants.first;
  auto const & b = constants.second;

  // Read speed bump height [m] from map
  speed_bump_height_ =
    static_cast<float>(speed_bump_reg_elem_.speedBump().attributeOr("height", 0.5));

  // Calculate the speed [m/s] for speed bump
  speed_bump_slow_down_speed_ = m * speed_bump_height_ + b;

  if (planner_param_.print_debug_info) {
    std::cout << "------------------------------" << std::endl;
    std::cout << "Speed Bump ID: " << module_id_ << std::endl;
    std::cout << "Speed Bump Height [cm]: " << speed_bump_height_ * 100 << std::endl;
    std::cout << "Calculated Speed [kph]: " << speed_bump_slow_down_speed_ * 3.6 << std::endl;
    std::cout << "------------------------------" << std::endl;
  }
}

bool SpeedBumpModule::modifyPathVelocity(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  auto ego_path = *path;

  path_intersects_.clear();

  const auto & ego_pos = planner_data_->current_pose.pose.position;
  const auto speed_bump = speed_bump_reg_elem_.speedBump();

  const auto intersects =
    getPolygonIntersects(ego_path, lanelet::utils::to2D(speed_bump).basicPolygon(), ego_pos, 2);

  for (const auto & p : intersects) {
    path_intersects_.push_back(createPoint(p.x(), p.y(), ego_pos.z));
  }

  debug_data_.path_polygon_intersection_points = path_intersects_;

  for (const auto & p : speed_bump_reg_elem_.speedBump().basicPolygon()) {
    debug_data_.speed_bump_polygon.push_back(createPoint(p.x(), p.y(), ego_pos.z));
  }

  return applySlowDownSpeed(*path);
}

bool SpeedBumpModule::applySlowDownSpeed(PathWithLaneId & output)
{
  if (path_intersects_.empty()) {
    return false;
  }

  const auto & ego_pos = planner_data_->current_pose.pose.position;
  const auto ego_path = output;

  // the range until to the point where ego will start accelerate
  auto slow_end_point_range =
    calcSignedArcLength(ego_path.points, ego_pos, path_intersects_.back());
  const auto & slow_end_margin_to_base_link =
    planner_data_->vehicle_info_.rear_overhang_m + planner_param_.slow_end_margin;
  slow_end_point_range += slow_end_margin_to_base_link;

  const auto & p_slow_end =
    calcLongitudinalOffsetPoint(ego_path.points, ego_pos, slow_end_point_range);

  if (!p_slow_end) {
    return false;
  }

  debug_data_.slow_end_points.push_back(p_slow_end.get());

  if (!passed_slow_start_point_) {
    state_ = State::SLOW_DOWN;

    // the range until to the point where ego will have a const slow down speed
    auto slow_start_point_range =
      calcSignedArcLength(ego_path.points, ego_pos, path_intersects_.front());
    const auto & slow_start_margin_from_base_link =
      planner_data_->vehicle_info_.max_longitudinal_offset_m + planner_param_.slow_start_margin;
    slow_start_point_range -= slow_start_margin_from_base_link;

    const auto & p_slow_start =
      calcLongitudinalOffsetPoint(ego_path.points, ego_pos, slow_start_point_range);

    if (!p_slow_start) {
      return false;
    }

    insertDecelPointWithDebugInfo(p_slow_start.get(), speed_bump_slow_down_speed_, output);

    if (slow_start_point_range < 0.0) {
      passed_slow_start_point_ = true;
    }
  } else if (slow_end_point_range > 0) {
    state_ = State::INSIDE;

    // insert constant ego speed until the end of the speed bump
    for (auto & p : output.points) {
      const auto & original_velocity = p.point.longitudinal_velocity_mps;
      p.point.longitudinal_velocity_mps = std::min(original_velocity, speed_bump_slow_down_speed_);
    }
  } else {
    state_ = State::OUT;
  }
  return true;
}

void SpeedBumpModule::insertDecelPointWithDebugInfo(
  const geometry_msgs::msg::Point & slow_point, const float target_velocity,
  PathWithLaneId & output)
{
  const auto slow_pose = planning_utils::insertDecelPoint(slow_point, output, target_velocity);
  if (!slow_pose) {
    return;
  }
  const auto & ego_pos = planner_data_->current_pose.pose.position;

  setDistance(calcSignedArcLength(output.points, ego_pos, slow_pose->position));

  debug_data_.slow_start_poses.push_back(*slow_pose);
}

std::pair<float, float> SpeedBumpModule::getLinearEquation(Point32 p1, Point32 p2)
{
  float m = (p1.y - p2.y) / (p1.x - p2.x);
  float b = p1.y - (m * p1.x);

  return std::make_pair(m, b);
}

}  // namespace behavior_velocity_planner
