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
using motion_utils::calcSignedArcLength;
using motion_utils::insertTargetPoint;
using tier4_autoware_utils::getPoint;
using tier4_autoware_utils::getPose;

namespace
{
double calcYawFromPoints(
  const geometry_msgs::msg::Point & p_front, const geometry_msgs::msg::Point & p_back)
{
  return std::atan2(p_back.y - p_front.y, p_back.x - p_front.x);
}

geometry_msgs::msg::Pose calcInterpolatedPose(
  const PathWithLaneId & path, const StopLineModule::SegmentIndexWithOffset & offset_segment)
{
  // Get segment points
  const auto & p_front = path.points.at(offset_segment.index).point.pose.position;
  const auto & p_back = path.points.at(offset_segment.index + 1).point.pose.position;

  // To Eigen point
  const auto p_eigen_front = Eigen::Vector2d(p_front.x, p_front.y);
  const auto p_eigen_back = Eigen::Vector2d(p_back.x, p_back.y);

  // Calculate interpolation ratio
  const auto interpolate_ratio = offset_segment.offset / (p_eigen_back - p_eigen_front).norm();

  // Add offset to front point
  const auto interpolated_point_2d =
    p_eigen_front + interpolate_ratio * (p_eigen_back - p_eigen_front);
  const double interpolated_z = p_front.z + interpolate_ratio * (p_back.z - p_front.z);

  // Calculate orientation so that X-axis would be along the trajectory
  tf2::Quaternion quat;
  quat.setRPY(0, 0, calcYawFromPoints(p_front, p_back));

  // To Pose
  geometry_msgs::msg::Pose interpolated_pose;
  interpolated_pose.position.x = interpolated_point_2d.x();
  interpolated_pose.position.y = interpolated_point_2d.y();
  interpolated_pose.position.z = interpolated_z;
  interpolated_pose.orientation = tf2::toMsg(quat);

  return interpolated_pose;
}

boost::optional<StopLineModule::SegmentIndexWithOffset> findBackwardOffsetSegment(
  const PathWithLaneId & path, const size_t base_idx, const double offset_length)
{
  double sum_length = 0.0;
  const auto start = static_cast<std::int32_t>(base_idx) - 1;
  for (std::int32_t i = start; i >= 0; --i) {
    const auto p_front = to_bg2d(path.points.at(i).point.pose.position);
    const auto p_back = to_bg2d(path.points.at(i + 1).point.pose.position);

    sum_length += bg::distance(p_front, p_back);

    // If it's over offset point, return front index and remain offset length
    if (sum_length >= offset_length) {
      const auto k = static_cast<std::size_t>(i);
      return StopLineModule::SegmentIndexWithOffset{k, sum_length - offset_length};
    }
  }

  // No enough path length
  return {};
}

}  // namespace

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
  if (planner_param_.show_stopline_collision_check) {
    debug_data_.search_stopline = stop_line;
    for (size_t i = min_search_index; i < max_search_index; ++i) {
      const auto & p_front = path.points.at(i).point.pose.position;
      const auto & p_back = path.points.at(i + 1).point.pose.position;
      const LineString2d path_segment = {{p_front.x, p_front.y}, {p_back.x, p_back.y}};
      debug_data_.search_segments.push_back(path_segment);
    }
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

boost::optional<StopLineModule::SegmentIndexWithOffset> StopLineModule::findOffsetSegment(
  const PathWithLaneId & path, const StopLineModule::SegmentIndexWithPoint2d & collision)
{
  const auto base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  const auto base_backward_length = planner_param_.stop_margin + base_link2front;

  const auto & p_back = to_bg2d(path.points.at(collision.index + 1).point.pose.position);

  return findBackwardOffsetSegment(
    path, collision.index + 1, base_backward_length + bg::distance(p_back, collision.point));
}

boost::optional<StopLineModule::SegmentIndexWithPose> StopLineModule::calcStopPose(
  const PathWithLaneId & path,
  const boost::optional<StopLineModule::SegmentIndexWithOffset> & offset_segment)
{
  // If no stop point found due to out of range, use front point on path
  if (!offset_segment) {
    return StopLineModule::SegmentIndexWithPose{0, path.points.front().point.pose};
  }

  return StopLineModule::SegmentIndexWithPose{
    offset_segment->index, calcInterpolatedPose(path, *offset_segment)};
}

bool StopLineModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  const auto base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  debug_data_ = DebugData();
  debug_data_.base_link2front = base_link2front;
  *stop_reason = planning_utils::initializeStopReason(StopReason::STOP_LINE);

  const LineString2d stop_line = planning_utils::extendLine(
    stop_line_[0], stop_line_[1], planner_data_->stop_line_extend_length);
  const auto & current_position = planner_data_->current_pose.pose.position;
  SearchRangeIndex dst_search_range =
    planning_utils::getPathIndexRangeIncludeLaneId(*path, lane_id_);

  // extend following and previous search range to avoid no collision
  if (dst_search_range.max_idx < path->points.size() - 1) dst_search_range.max_idx++;
  if (dst_search_range.min_idx > 0) dst_search_range.min_idx--;

  // Find collision
  const auto collision = findCollision(*path, stop_line, dst_search_range);

  // If no collision found, do nothing
  if (!collision) {
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 5000 /* ms */, "is no collision");
    return true;
  }

  // Find offset segment
  const auto offset_segment = findOffsetSegment(*path, *collision);

  // Calculate stop pose and insert index
  const auto stop_pose_with_index = calcStopPose(*path, offset_segment);

  /**
   * @brief : calculate signed arc length consider stop margin from stop line
   *
   * |----------------------------|
   * s---ego----------x--|--------g
   */
  const auto stop_point = getPoint(stop_pose_with_index.get().pose);
  const auto signed_arc_dist_to_stop_point =
    calcSignedArcLength(path->points, current_position, stop_point);

  switch (state_) {
    case State::APPROACH: {
      if (!stop_pose_with_index) {
        break;
      }
      // Insert stop pose
      insertStopPoint(stop_point, *path);

      debug_data_.stop_pose = stop_pose_with_index.get().pose;

      // Move to stopped state if stopped
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
      // Change state after vehicle departure
      insertStopPoint(current_position, *path);

      debug_data_.stop_pose = stop_pose_with_index.get().pose;

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

geometry_msgs::msg::Point StopLineModule::getCenterOfStopLine(
  const lanelet::ConstLineString3d & stop_line)
{
  geometry_msgs::msg::Point center_point;
  center_point.x = (stop_line[0].x() + stop_line[1].x()) / 2.0;
  center_point.y = (stop_line[0].y() + stop_line[1].y()) / 2.0;
  center_point.z = (stop_line[0].z() + stop_line[1].z()) / 2.0;
  return center_point;
}
}  // namespace behavior_velocity_planner
