
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

#include <autoware/behavior_path_lane_change_module/utils/calculation.hpp>
#include <autoware/behavior_path_planner_common/utils/utils.hpp>

#include <boost/geometry/algorithms/buffer.hpp>

namespace autoware::behavior_path_planner::utils::lane_change::calculation
{
double calc_dist_from_pose_to_terminal_end(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & lanes,
  const Pose & src_pose)
{
  if (lanes.empty()) {
    return 0.0;
  }

  const auto in_goal_route_section =
    common_data_ptr->route_handler_ptr->isInGoalRouteSection(lanes.back());
  if (in_goal_route_section) {
    const auto & goal_pose = common_data_ptr->route_handler_ptr->getGoalPose();
    return utils::getSignedDistance(src_pose, goal_pose, lanes);
  }
  return utils::getDistanceToEndOfLane(src_pose, lanes);
}

double calc_stopping_distance(const LCParamPtr & lc_param_ptr)
{
  // v^2 = u^2 + 2ad
  const auto min_lc_vel = lc_param_ptr->minimum_lane_changing_velocity;
  const auto min_lon_acc = lc_param_ptr->min_longitudinal_acc;
  const auto min_back_dist = std::abs((min_lc_vel * min_lc_vel) / (2 * min_lon_acc));

  const auto param_back_dist = lc_param_ptr->backward_length_buffer_for_end_of_lane;
  return std::max(min_back_dist, param_back_dist);
}

double calc_dist_to_last_fit_width(
  const lanelet::ConstLanelets & lanelets, const Pose & src_pose,
  const BehaviorPathPlannerParameters & bpp_param, const double margin)
{
  if (lanelets.empty()) return 0.0;

  const auto lane_polygon = lanelets.back().polygon2d().basicPolygon();
  const auto center_line = lanelet::utils::generateFineCenterline(lanelets.back(), 1.0);

  if (center_line.size() <= 1) return 0.0;

  universe_utils::LineString2d line_string;
  line_string.reserve(center_line.size() - 1);
  std::for_each(center_line.begin() + 1, center_line.end(), [&line_string](const auto & point) {
    boost::geometry::append(line_string, universe_utils::Point2d(point.x(), point.y()));
  });

  const double buffer_distance = 0.5 * bpp_param.vehicle_width + margin;
  universe_utils::MultiPolygon2d center_line_polygon;
  namespace strategy = boost::geometry::strategy::buffer;
  boost::geometry::buffer(
    line_string, center_line_polygon, strategy::distance_symmetric<double>(buffer_distance),
    strategy::side_straight(), strategy::join_miter(), strategy::end_flat(),
    strategy::point_square());

  if (center_line_polygon.empty()) return 0.0;

  std::vector<universe_utils::Point2d> intersection_points;
  boost::geometry::intersection(lane_polygon, center_line_polygon, intersection_points);

  if (intersection_points.empty()) {
    return utils::getDistanceToEndOfLane(src_pose, lanelets);
  }

  Pose pose;
  double distance = std::numeric_limits<double>::max();
  for (const auto & point : intersection_points) {
    pose.position.x = boost::geometry::get<0>(point);
    pose.position.y = boost::geometry::get<1>(point);
    distance = std::min(distance, utils::getSignedDistance(src_pose, pose, lanelets));
  }

  return std::max(distance - (bpp_param.base_link2front + margin), 0.0);
}

double calc_maximum_prepare_length(const CommonDataPtr & common_data_ptr)
{
  const auto max_prepare_duration = common_data_ptr->lc_param_ptr->lane_change_prepare_duration;
  const auto ego_max_speed = common_data_ptr->bpp_param_ptr->max_vel;

  return max_prepare_duration * ego_max_speed;
}

double calc_ego_dist_to_lanes_start(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes)
{
  const auto & route_handler_ptr = common_data_ptr->route_handler_ptr;

  if (!route_handler_ptr || target_lanes.empty() || current_lanes.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto & target_bound =
    common_data_ptr->direction == autoware::route_handler::Direction::RIGHT
      ? target_lanes.front().leftBound()
      : target_lanes.front().rightBound();

  if (target_bound.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto & path = common_data_ptr->current_lanes_path;

  if (path.points.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto target_front_pt = lanelet::utils::conversion::toGeomMsgPt(target_bound.front());
  const auto ego_position = common_data_ptr->get_ego_pose().position;

  return motion_utils::calcSignedArcLength(path.points, ego_position, target_front_pt);
}

std::vector<double> calc_all_min_lc_lengths(
  const LCParamPtr & lc_param_ptr, const std::vector<double> & shift_intervals)
{
  if (shift_intervals.empty()) {
    return {};
  }

  const auto min_vel = lc_param_ptr->minimum_lane_changing_velocity;
  const auto min_max_lat_acc = lc_param_ptr->lane_change_lat_acc_map.find(min_vel);
  const auto max_lat_acc = std::get<1>(min_max_lat_acc);
  const auto lat_jerk = lc_param_ptr->lane_changing_lateral_jerk;

  std::vector<double> min_lc_lengths{};
  min_lc_lengths.reserve(shift_intervals.size());

  const auto min_lc_length = [&](const auto shift_interval) {
    const auto t = PathShifter::calcShiftTimeFromJerk(shift_interval, lat_jerk, max_lat_acc);
    return min_vel * t;
  };

  std::transform(
    shift_intervals.cbegin(), shift_intervals.cend(), std::back_inserter(min_lc_lengths),
    min_lc_length);

  return min_lc_lengths;
}

std::vector<double> calc_all_max_lc_lengths(
  const CommonDataPtr & common_data_ptr, const std::vector<double> & shift_intervals)
{
  if (shift_intervals.empty()) {
    return {};
  }

  const auto & lc_param_ptr = common_data_ptr->lc_param_ptr;
  const auto lat_jerk = lc_param_ptr->lane_changing_lateral_jerk;
  const auto t_prepare = lc_param_ptr->lane_change_prepare_duration;
  const auto max_acc = common_data_ptr->transient_data.acc.max;

  const auto limit_vel = [&](const auto vel) {
    const auto max_global_vel = common_data_ptr->bpp_param_ptr->max_vel;
    return std::clamp(vel, lc_param_ptr->minimum_lane_changing_velocity, max_global_vel);
  };

  auto vel = limit_vel(common_data_ptr->get_ego_speed());

  std::vector<double> max_lc_lengths{};

  const auto max_lc_length = [&](const auto shift_interval) {
    // prepare section
    const auto prepare_length = vel * t_prepare + 0.5 * max_acc * t_prepare * t_prepare;
    vel = limit_vel(vel + max_acc * t_prepare);

    // lane changing section
    const auto [min_lat_acc, max_lat_acc] = lc_param_ptr->lane_change_lat_acc_map.find(vel);
    const auto t_lane_changing =
      PathShifter::calcShiftTimeFromJerk(shift_interval, lat_jerk, max_lat_acc);
    const auto lane_changing_length =
      vel * t_lane_changing + 0.5 * max_acc * t_lane_changing * t_lane_changing;

    vel = limit_vel(vel + max_acc * t_lane_changing);
    return prepare_length + lane_changing_length;
  };

  std::transform(
    shift_intervals.cbegin(), shift_intervals.cend(), std::back_inserter(max_lc_lengths),
    max_lc_length);
  return max_lc_lengths;
}

double calc_distance_buffer(
  const LCParamPtr & lc_param_ptr, const std::vector<double> & min_lc_lengths)
{
  if (min_lc_lengths.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto finish_judge_buffer = lc_param_ptr->lane_change_finish_judge_buffer;
  const auto backward_buffer = calc_stopping_distance(lc_param_ptr);
  const auto lengths_sum = std::accumulate(min_lc_lengths.begin(), min_lc_lengths.end(), 0.0);
  const auto num_of_lane_changes = static_cast<double>(min_lc_lengths.size());
  return lengths_sum + (num_of_lane_changes * finish_judge_buffer) +
         ((num_of_lane_changes - 1.0) * backward_buffer);
}

std::vector<double> calc_shift_intervals(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & lanes)
{
  if (!common_data_ptr || !common_data_ptr->is_data_available() || lanes.empty()) {
    return {};
  }

  const auto & route_handler_ptr = common_data_ptr->route_handler_ptr;
  const auto direction = common_data_ptr->direction;

  return route_handler_ptr->getLateralIntervalsToPreferredLane(lanes.back(), direction);
}

std::pair<Boundary, Boundary> calc_lc_length_and_dist_buffer(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & lanes)
{
  if (!common_data_ptr || !common_data_ptr->is_data_available() || lanes.empty()) {
    return {};
  }
  const auto shift_intervals = calculation::calc_shift_intervals(common_data_ptr, lanes);
  const auto all_min_lc_lengths =
    calculation::calc_all_min_lc_lengths(common_data_ptr->lc_param_ptr, shift_intervals);
  const auto min_lc_length =
    !all_min_lc_lengths.empty() ? all_min_lc_lengths.front() : std::numeric_limits<double>::max();
  const auto min_dist_buffer =
    calculation::calc_distance_buffer(common_data_ptr->lc_param_ptr, all_min_lc_lengths);

  const auto all_max_lc_lengths =
    calculation::calc_all_max_lc_lengths(common_data_ptr, shift_intervals);
  const auto max_lc_length =
    !all_max_lc_lengths.empty() ? all_max_lc_lengths.front() : std::numeric_limits<double>::max();
  const auto max_dist_buffer =
    calculation::calc_distance_buffer(common_data_ptr->lc_param_ptr, all_max_lc_lengths);

  Boundary lc_length;
  lc_length.min = min_lc_length;
  lc_length.max = max_lc_length;

  Boundary dist_buffer;
  dist_buffer.min = min_dist_buffer;
  dist_buffer.max = max_dist_buffer;

  return {lc_length, dist_buffer};
}
}  // namespace autoware::behavior_path_planner::utils::lane_change::calculation
