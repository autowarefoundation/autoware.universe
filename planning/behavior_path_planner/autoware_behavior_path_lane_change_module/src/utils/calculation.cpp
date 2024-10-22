
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
#include <autoware/motion_utils/trajectory/path_shift.hpp>

#include <boost/geometry/algorithms/buffer.hpp>

namespace autoware::behavior_path_planner::utils::lane_change::calculation
{

rclcpp::Logger get_logger()
{
  constexpr const char * name{"lane_change.utils"};
  static rclcpp::Logger logger = rclcpp::get_logger(name);
  return logger;
}

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

double calc_maximum_lane_change_length(
  const double current_velocity, const LaneChangeParameters & lane_change_parameters,
  const std::vector<double> & shift_intervals, const double max_acc)
{
  if (shift_intervals.empty()) {
    return 0.0;
  }

  const auto finish_judge_buffer = lane_change_parameters.lane_change_finish_judge_buffer;
  const auto lat_jerk = lane_change_parameters.lane_changing_lateral_jerk;
  const auto t_prepare = lane_change_parameters.lane_change_prepare_duration;

  auto vel = current_velocity;

  const auto calc_sum = [&](double sum, double shift_interval) {
    // prepare section
    const auto prepare_length = vel * t_prepare + 0.5 * max_acc * t_prepare * t_prepare;
    vel = vel + max_acc * t_prepare;

    // lane changing section
    const auto [min_lat_acc, max_lat_acc] =
      lane_change_parameters.lane_change_lat_acc_map.find(vel);
    const auto t_lane_changing =
      autoware::motion_utils::calc_shift_time_from_jerk(shift_interval, lat_jerk, max_lat_acc);
    const auto lane_changing_length =
      vel * t_lane_changing + 0.5 * max_acc * t_lane_changing * t_lane_changing;

    vel = vel + max_acc * t_lane_changing;
    return sum + (prepare_length + lane_changing_length + finish_judge_buffer);
  };

  const auto total_length =
    std::accumulate(shift_intervals.begin(), shift_intervals.end(), 0.0, calc_sum);

  const auto backward_buffer = lane_change_parameters.backward_length_buffer_for_end_of_lane;
  return total_length + backward_buffer * (static_cast<double>(shift_intervals.size()) - 1.0);
}

double calc_maximum_lane_change_length(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelet & current_terminal_lanelet,
  const double max_acc)
{
  const auto shift_intervals =
    common_data_ptr->route_handler_ptr->getLateralIntervalsToPreferredLane(
      current_terminal_lanelet);
  const auto vel = std::max(
    common_data_ptr->get_ego_speed(),
    common_data_ptr->lc_param_ptr->minimum_lane_changing_velocity);
  return calc_maximum_lane_change_length(
    vel, *common_data_ptr->lc_param_ptr, shift_intervals, max_acc);
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
    const auto t =
      autoware::motion_utils::calc_shift_time_from_jerk(shift_interval, lat_jerk, max_lat_acc);
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
    vel = limit_vel(vel + max_acc * t_prepare);
    const auto prepare_length = vel * t_prepare + 0.5 * max_acc * t_prepare * t_prepare;

    // lane changing section
    const auto [min_lat_acc, max_lat_acc] = lc_param_ptr->lane_change_lat_acc_map.find(vel);
    const auto t_lane_changing =
      autoware::motion_utils::calc_shift_time_from_jerk(shift_interval, lat_jerk, max_lat_acc);
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

std::pair<MinMaxValue, MinMaxValue> calc_lc_length_and_dist_buffer(
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

  return {{min_lc_length, max_lc_length}, {min_dist_buffer, max_dist_buffer}};
}

double calc_phase_length(
  const double velocity, const double maximum_velocity, const double acceleration,
  const double duration)
{
  const auto length_with_acceleration =
    velocity * duration + 0.5 * acceleration * std::pow(duration, 2);
  const auto length_with_max_velocity = maximum_velocity * duration;
  return std::min(length_with_acceleration, length_with_max_velocity);
}

double calc_lane_changing_acceleration(
  const double initial_lane_changing_velocity, const double max_path_velocity,
  const double lane_changing_time, const double prepare_longitudinal_acc)
{
  if (prepare_longitudinal_acc <= 0.0) {
    return 0.0;
  }

  return std::clamp(
    (max_path_velocity - initial_lane_changing_velocity) / lane_changing_time, 0.0,
    prepare_longitudinal_acc);
}

std::vector<PhaseMetrics> calc_prepare_phase_metrics(
  const CommonDataPtr & common_data_ptr, const std::vector<double> & prepare_durations,
  const std::vector<double> & lon_accel_values, const double current_velocity,
  const double min_length_threshold, const double max_length_threshold)
{
  const auto & min_lc_vel = common_data_ptr->lc_param_ptr->minimum_lane_changing_velocity;
  const auto & max_vel = common_data_ptr->bpp_param_ptr->max_vel;

  std::vector<PhaseMetrics> metrics;

  auto is_skip = [&](const double prepare_length) {
    if (prepare_length > max_length_threshold || prepare_length < min_length_threshold) {
      RCLCPP_DEBUG(
        get_logger(),
        "Skip: prepare length out of expected range. length: %.5f, threshold min: %.5f, max: %.5f",
        prepare_length, min_length_threshold, max_length_threshold);
      return true;
    }
    return false;
  };

  metrics.reserve(prepare_durations.size() * lon_accel_values.size());
  for (const auto & prepare_duration : prepare_durations) {
    for (const auto & lon_accel : lon_accel_values) {
      const auto prepare_velocity =
        std::clamp(current_velocity + lon_accel * prepare_duration, min_lc_vel, max_vel);

      // compute actual longitudinal acceleration
      const double prepare_accel = (prepare_duration < 1e-3)
                                     ? 0.0
                                     : ((prepare_velocity - current_velocity) / prepare_duration);

      const auto prepare_length =
        calc_phase_length(current_velocity, max_vel, prepare_accel, prepare_duration);

      if (is_skip(prepare_length)) continue;

      metrics.emplace_back(
        prepare_duration, prepare_length, prepare_velocity, lon_accel, prepare_accel, 0.0);
    }
  }

  return metrics;
}

std::vector<PhaseMetrics> calc_shift_phase_metrics(
  const CommonDataPtr & common_data_ptr, const double shift_length, const double initial_velocity,
  const double max_path_velocity, const double lon_accel, const double max_length_threshold)
{
  const auto & min_lc_vel = common_data_ptr->lc_param_ptr->minimum_lane_changing_velocity;
  const auto & max_vel = common_data_ptr->bpp_param_ptr->max_vel;

  // get lateral acceleration range
  const auto [min_lateral_acc, max_lateral_acc] =
    common_data_ptr->lc_param_ptr->lane_change_lat_acc_map.find(initial_velocity);
  const auto lateral_acc_resolution = std::abs(max_lateral_acc - min_lateral_acc) /
                                      common_data_ptr->lc_param_ptr->lateral_acc_sampling_num;

  std::vector<PhaseMetrics> metrics;

  auto is_skip = [&](const double lane_changing_length) {
    if (lane_changing_length > max_length_threshold) {
      RCLCPP_DEBUG(
        get_logger(),
        "Skip: lane changing length exceeds maximum threshold. length: %.5f, threshold: %.5f",
        lane_changing_length, max_length_threshold);
      return true;
    }
    return false;
  };

  for (double lat_acc = min_lateral_acc; lat_acc < max_lateral_acc + eps;
       lat_acc += lateral_acc_resolution) {
    const auto lane_changing_duration = autoware::motion_utils::calc_shift_time_from_jerk(
      shift_length, common_data_ptr->lc_param_ptr->lane_changing_lateral_jerk, lat_acc);

    const double lane_changing_accel = calc_lane_changing_acceleration(
      initial_velocity, max_path_velocity, lane_changing_duration, lon_accel);

    const auto lane_changing_length = calculation::calc_phase_length(
      initial_velocity, max_vel, lane_changing_accel, lane_changing_duration);

    if (is_skip(lane_changing_length)) continue;

    const auto lane_changing_velocity = std::clamp(
      initial_velocity + lane_changing_accel * lane_changing_duration, min_lc_vel, max_vel);

    metrics.emplace_back(
      lane_changing_duration, lane_changing_length, lane_changing_velocity, lon_accel,
      lane_changing_accel, lat_acc);
  }

  return metrics;
}
}  // namespace autoware::behavior_path_planner::utils::lane_change::calculation
