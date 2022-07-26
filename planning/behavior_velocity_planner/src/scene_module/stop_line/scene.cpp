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

#include <motion_utils/trajectory/trajectory.hpp>
#include <scene_module/stop_line/scene.hpp>
#include <utilization/util.hpp>

#include <algorithm>
#include <vector>

namespace behavior_velocity_planner
{

namespace bg = boost::geometry;
using motion_utils::calcLongitudinalOffsetPoint;
using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcSignedArcLength;
using motion_utils::insertTargetPoint;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::getPoint;
using tier4_autoware_utils::getPose;

StopLineModule::StopLineModule(
  const int64_t module_id, const size_t lane_id, const lanelet::ConstLineString3d & stop_line,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  module_id_(module_id),
  stop_line_(stop_line),
  lane_id_(lane_id),
  state_(State::APPROACH)
{
  planner_param_ = planner_param;
}

boost::optional<StopLineModule::SegmentIndexWithPoint2d> StopLineModule::findCollision(
  const PathWithLaneId & path, const LineString2d & stop_line,
  const SearchRangeIndex & search_index)
{
  const size_t min_search_index = std::max(static_cast<size_t>(0), search_index.min_idx);
  const size_t max_search_index = std::min(search_index.max_idx, path.points.size() - 1);

  // for creating debug marker
  debug_data_.search_stopline = stop_line;
  for (size_t i = min_search_index; i < max_search_index; ++i) {
    const auto & p_front = path.points.at(i).point.pose.position;
    const auto & p_back = path.points.at(i + 1).point.pose.position;
    const LineString2d path_segment = {{p_front.x, p_front.y}, {p_back.x, p_back.y}};
    debug_data_.search_segments.push_back(path_segment);
  }

  for (size_t i = min_search_index; i < max_search_index; ++i) {
    const auto & p_front = path.points.at(i).point.pose.position;
    const auto & p_back = path.points.at(i + 1).point.pose.position;

    // Find intersection
    const LineString2d path_segment = {{p_front.x, p_front.y}, {p_back.x, p_back.y}};
    std::vector<Point2d> collision_points;
    bg::intersection(stop_line, path_segment, collision_points);

    // Ignore if no collision found
    if (collision_points.empty()) {
      continue;
    }

    // Select first collision
    const auto & collision_point = collision_points.at(0);

    return StopLineModule::SegmentIndexWithPoint2d{i, collision_point};
  }

  return {};
}

bool StopLineModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  const auto & base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  debug_data_ = DebugData();
  debug_data_.base_link2front = base_link2front;
  *stop_reason = planning_utils::initializeStopReason(StopReason::STOP_LINE);

  const auto ego_path = *path;
  const auto & ego_pos = planner_data_->current_pose.pose.position;

  const LineString2d stop_line = planning_utils::extendLine(
    stop_line_[0], stop_line_[1], planner_data_->stop_line_extend_length);
  SearchRangeIndex dst_search_range =
    planning_utils::getPathIndexRangeIncludeLaneId(ego_path, lane_id_);

  // extend following and previous search range to avoid no collision
  if (dst_search_range.max_idx < ego_path.points.size() - 1) dst_search_range.max_idx++;
  if (dst_search_range.min_idx > 0) dst_search_range.min_idx--;

  // Find collision
  const auto collision = findCollision(ego_path, stop_line, dst_search_range);

  // If no collision found, do nothing
  if (!collision) {
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 5000 /* ms */, "is no collision");
    return true;
  }

  const auto p_stop_line =
    createPoint(collision.get().point.x(), collision.get().point.y(), ego_pos.z);
  const auto margin = planner_param_.stop_margin + base_link2front;
  const auto stop_pose = calcLongitudinalOffsetPose(ego_path.points, p_stop_line, -margin);

  if (!stop_pose) {
    return false;
  }

  StopFactor stop_factor;
  stop_factor.stop_pose = stop_pose.get();
  stop_factor.stop_factor_points.push_back(p_stop_line);

  /**
   * @brief : calculate signed arc length consider stop margin from stop line
   *
   * |----------------------------|
   * s---ego----------x--|--------g
   */
  const auto signed_arc_dist_to_stop_point =
    calcSignedArcLength(ego_path.points, ego_pos, stop_pose.get().position);

  switch (state_) {
    case State::APPROACH: {
      insertStopPoint(stop_pose.get().position, *path);
      planning_utils::appendStopReason(stop_factor, stop_reason);

      debug_data_.stop_pose = stop_pose.get();

      if (
        signed_arc_dist_to_stop_point < planner_param_.stop_check_dist &&
        planner_data_->isVehicleStopped()) {
        RCLCPP_INFO(logger_, "APPROACH -> STOPPED");

        state_ = State::STOPPED;
        stopped_time_ = std::make_shared<const rclcpp::Time>(clock_->now());

        if (signed_arc_dist_to_stop_point < -planner_param_.stop_check_dist) {
          RCLCPP_ERROR(
            logger_, "Failed to stop near stop line but ego stopped. Change state to STOPPED");
        }
      }

      break;
    }

    case State::STOPPED: {
      const auto ego_pos_on_path = calcLongitudinalOffsetPoint(ego_path.points, ego_pos, 0.0);

      if (!ego_pos_on_path) {
        break;
      }

      insertStopPoint(ego_pos_on_path.get(), *path);
      planning_utils::appendStopReason(stop_factor, stop_reason);

      debug_data_.stop_pose = stop_pose.get();

      const auto elapsed_time = (clock_->now() - *stopped_time_).seconds();

      if (planner_param_.stop_duration_sec < elapsed_time) {
        RCLCPP_INFO(logger_, "STOPPED -> START");
        state_ = State::START;
      }

      break;
    }

    case State::START: {
      // Initialize if vehicle is far from stop_line
      if (planner_param_.use_initialization_stop_line_state) {
        if (signed_arc_dist_to_stop_point > planner_param_.stop_check_dist) {
          RCLCPP_INFO(logger_, "START -> APPROACH");
          state_ = State::APPROACH;
        }
      }

      break;
    }

    default:
      RCLCPP_ERROR(logger_, "Unknown state.");
  }

  return true;
}

void StopLineModule::insertStopPoint(
  const geometry_msgs::msg::Point & stop_point, PathWithLaneId & path) const
{
  const size_t base_idx = findNearestSegmentIndex(path.points, stop_point);
  const auto insert_idx = insertTargetPoint(base_idx, stop_point, path.points);

  if (!insert_idx) {
    return;
  }

  for (size_t i = insert_idx.get(); i < path.points.size(); ++i) {
    path.points.at(i).point.longitudinal_velocity_mps = 0.0;
  }
}
}  // namespace behavior_velocity_planner
