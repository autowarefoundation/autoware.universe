// Copyright 2022 TIER IV, Inc.
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

#include "autoware/behavior_path_goal_planner_module/pull_over_planner/shift_pull_over.hpp"

#include "autoware/behavior_path_goal_planner_module/util.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"

#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
ShiftPullOver::ShiftPullOver(rclcpp::Node & node, const GoalPlannerParameters & parameters)
: PullOverPlannerBase{node, parameters},
  lane_departure_checker_{[&]() {
    auto lane_departure_checker_params = lane_departure_checker::Param{};
    lane_departure_checker_params.footprint_extra_margin =
      parameters.lane_departure_check_expansion_margin;
    return LaneDepartureChecker{lane_departure_checker_params, vehicle_info_};
  }()},
  left_side_parking_{parameters.parking_policy == ParkingPolicy::LEFT_SIDE}
{
}
std::optional<PullOverPath> ShiftPullOver::plan(
  const GoalCandidate & modified_goal_pose, const size_t id,
  const std::shared_ptr<const PlannerData> planner_data,
  const BehaviorModuleOutput & upstream_module_output)
{
  const auto & route_handler = planner_data->route_handler;
  const double min_jerk = parameters_.minimum_lateral_jerk;
  const double max_jerk = parameters_.maximum_lateral_jerk;
  const double backward_search_length = parameters_.backward_goal_search_length;
  const double forward_search_length = parameters_.forward_goal_search_length;
  const int shift_sampling_num = parameters_.shift_sampling_num;
  const double jerk_resolution = std::abs(max_jerk - min_jerk) / shift_sampling_num;

  const auto road_lanes = utils::getExtendedCurrentLanesFromPath(
    upstream_module_output.path, planner_data, backward_search_length, forward_search_length,
    /*forward_only_in_route*/ false);

  const auto pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *route_handler, left_side_parking_, backward_search_length, forward_search_length);
  if (road_lanes.empty() || pull_over_lanes.empty()) {
    return {};
  }

  // find safe one from paths with different jerk
  for (double lateral_jerk = min_jerk; lateral_jerk <= max_jerk; lateral_jerk += jerk_resolution) {
    const auto pull_over_path = generatePullOverPath(
      modified_goal_pose, id, planner_data, upstream_module_output, road_lanes, pull_over_lanes,
      lateral_jerk);
    if (!pull_over_path) continue;
    return pull_over_path;
  }

  return {};
}

PathWithLaneId ShiftPullOver::generateReferencePath(
  const std::shared_ptr<const PlannerData> planner_data, const lanelet::ConstLanelets & road_lanes,
  const Pose & end_pose) const
{
  const auto & route_handler = planner_data->route_handler;
  const Pose & current_pose = planner_data->self_odometry->pose.pose;
  const double backward_path_length = planner_data->parameters.backward_path_length;
  const double pull_over_velocity = parameters_.pull_over_velocity;
  const double deceleration_interval = parameters_.deceleration_interval;

  const auto current_road_arc_coords = lanelet::utils::getArcCoordinates(road_lanes, current_pose);
  const double s_start = current_road_arc_coords.length - backward_path_length;
  const double s_end = std::max(
    lanelet::utils::getArcCoordinates(road_lanes, end_pose).length,
    s_start + std::numeric_limits<double>::epsilon());
  auto road_lane_reference_path = route_handler->getCenterLinePath(road_lanes, s_start, s_end);

  // decelerate velocity linearly to minimum pull over velocity
  // (or keep original velocity if it is lower than pull over velocity)
  for (auto & point : road_lane_reference_path.points) {
    const auto arclength = lanelet::utils::getArcCoordinates(road_lanes, point.point.pose).length;
    const double distance_to_pull_over_start =
      std::clamp(s_end - arclength, 0.0, deceleration_interval);
    const auto decelerated_velocity = static_cast<float>(
      distance_to_pull_over_start / deceleration_interval *
        (point.point.longitudinal_velocity_mps - pull_over_velocity) +
      pull_over_velocity);
    point.point.longitudinal_velocity_mps =
      std::min(point.point.longitudinal_velocity_mps, decelerated_velocity);
  }
  return road_lane_reference_path;
}

std::optional<PathWithLaneId> ShiftPullOver::cropPrevModulePath(
  const PathWithLaneId & prev_module_path, const Pose & shift_end_pose) const
{
  // clip previous module path to shift end pose nearest segment index
  const size_t shift_end_idx = autoware::motion_utils::findNearestSegmentIndex(
    prev_module_path.points, shift_end_pose.position);
  std::vector<PathPointWithLaneId> clipped_points{
    prev_module_path.points.begin(), prev_module_path.points.begin() + shift_end_idx};
  if (clipped_points.empty()) {
    return std::nullopt;
  }

  // add projected shift end pose to clipped points
  PathPointWithLaneId projected_point = clipped_points.back();
  const double offset = autoware::motion_utils::calcSignedArcLength(
    prev_module_path.points, shift_end_idx, shift_end_pose.position);
  projected_point.point.pose =
    autoware::universe_utils::calcOffsetPose(clipped_points.back().point.pose, offset, 0, 0);
  clipped_points.push_back(projected_point);
  auto clipped_prev_module_path = prev_module_path;
  clipped_prev_module_path.points = clipped_points;

  return clipped_prev_module_path;
}

std::optional<PullOverPath> ShiftPullOver::generatePullOverPath(
  const GoalCandidate & goal_candidate, const size_t id,
  const std::shared_ptr<const PlannerData> planner_data,
  const BehaviorModuleOutput & upstream_module_output, const lanelet::ConstLanelets & road_lanes,
  const lanelet::ConstLanelets & pull_over_lanes, const double lateral_jerk) const
{
  const double pull_over_velocity = parameters_.pull_over_velocity;
  const double after_shift_straight_distance = parameters_.after_shift_straight_distance;

  const auto & goal_pose = goal_candidate.goal_pose;

  // shift end pose is longitudinal offset from goal pose to improve parking angle accuracy
  const Pose shift_end_pose =
    autoware::universe_utils::calcOffsetPose(goal_pose, -after_shift_straight_distance, 0, 0);

  // calculate lateral shift of previous module path terminal pose from road lane reference path
  const auto road_lane_reference_path_to_shift_end = utils::resamplePathWithSpline(
    generateReferencePath(planner_data, road_lanes, shift_end_pose),
    parameters_.center_line_path_interval);
  const auto prev_module_path = utils::resamplePathWithSpline(
    upstream_module_output.path, parameters_.center_line_path_interval);
  const auto prev_module_path_terminal_pose = prev_module_path.points.back().point.pose;

  // process previous module path for path shifter input path
  // case1) extend path if shift end pose is behind of previous module path terminal pose
  // case2) crop path if shift end pose is ahead of previous module path terminal pose
  const auto processed_prev_module_path = std::invoke([&]() -> std::optional<PathWithLaneId> {
    const bool extend_previous_module_path =
      lanelet::utils::getArcCoordinates(road_lanes, shift_end_pose).length >
      lanelet::utils::getArcCoordinates(road_lanes, prev_module_path_terminal_pose).length;
    if (extend_previous_module_path) {  // case1
      // NOTE: The previous module may insert a zero velocity at the end of the path, so remove it
      // by setting remove_connected_zero_velocity=true. Inserting a velocity of 0 into the goal is
      // the role of the goal planner, and the intermediate zero velocity after extension is
      // unnecessary.
      return goal_planner_utils::extendPath(
        prev_module_path, road_lane_reference_path_to_shift_end, shift_end_pose, true);
    } else {  // case2
      return goal_planner_utils::cropPath(prev_module_path, shift_end_pose);
    }
  });
  if (!processed_prev_module_path || processed_prev_module_path->points.empty()) {
    return {};
  }

  // calculate shift length
  const Pose & shift_end_pose_prev_module_path =
    processed_prev_module_path->points.back().point.pose;
  const double shift_end_road_to_target_distance =
    autoware::universe_utils::inverseTransformPoint(
      shift_end_pose.position, shift_end_pose_prev_module_path)
      .y;

  // calculate shift start pose on road lane
  const double pull_over_distance = autoware::motion_utils::calc_longitudinal_dist_from_jerk(
    shift_end_road_to_target_distance, lateral_jerk, pull_over_velocity);
  const double before_shifted_pull_over_distance = calcBeforeShiftedArcLength(
    processed_prev_module_path.value(), pull_over_distance, shift_end_road_to_target_distance);
  const auto shift_start_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    processed_prev_module_path->points, shift_end_pose_prev_module_path.position,
    -before_shifted_pull_over_distance);

  // set path shifter and generate shifted path
  PathShifter path_shifter{};
  path_shifter.setPath(processed_prev_module_path.value());
  ShiftLine shift_line{};
  shift_line.start = *shift_start_pose;
  shift_line.end = shift_end_pose;
  shift_line.end_shift_length = shift_end_road_to_target_distance;
  path_shifter.addShiftLine(shift_line);
  ShiftedPath shifted_path{};
  const bool offset_back = true;  // offset front side from reference path
  if (!path_shifter.generate(&shifted_path, offset_back)) {
    return {};
  }
  shifted_path.path.points = autoware::motion_utils::removeOverlapPoints(shifted_path.path.points);
  autoware::motion_utils::insertOrientation(shifted_path.path.points, true);

  // for debug. result of shift is not equal to the target
  const Pose actual_shift_end_pose = shifted_path.path.points.back().point.pose;

  // interpolate between shift end pose to goal pose
  std::vector<Pose> interpolated_poses =
    utils::interpolatePose(shifted_path.path.points.back().point.pose, goal_pose, 0.5);
  for (const auto & pose : interpolated_poses) {
    PathPointWithLaneId p = shifted_path.path.points.back();
    p.point.pose = pose;
    shifted_path.path.points.push_back(p);
  }

  // combine road lanes and shoulder lanes to find closest lanelet id
  const auto lanes = std::invoke([&]() -> lanelet::ConstLanelets {
    auto lanes = road_lanes;
    lanes.insert(lanes.end(), pull_over_lanes.begin(), pull_over_lanes.end());
    return lanes;  // not copy the value (Return Value Optimization)
  });

  // set goal pose with velocity 0
  {
    PathPointWithLaneId p{};
    p.point.longitudinal_velocity_mps = 0.0;
    p.point.pose = goal_pose;
    lanelet::Lanelet goal_lanelet{};
    if (lanelet::utils::query::getClosestLanelet(lanes, goal_pose, &goal_lanelet)) {
      p.lane_ids = {goal_lanelet.id()};
    } else {
      p.lane_ids = shifted_path.path.points.back().lane_ids;
    }
    shifted_path.path.points.push_back(p);
  }

  // set lane_id and velocity to shifted_path
  for (size_t i = path_shifter.getShiftLines().front().start_idx;
       i < shifted_path.path.points.size() - 1; ++i) {
    auto & point = shifted_path.path.points.at(i);
    point.point.longitudinal_velocity_mps =
      std::min(point.point.longitudinal_velocity_mps, static_cast<float>(pull_over_velocity));
    lanelet::Lanelet lanelet{};
    if (lanelet::utils::query::getClosestLanelet(lanes, point.point.pose, &lanelet)) {
      point.lane_ids = {lanelet.id()};  // overwrite lane_ids
    } else {
      point.lane_ids = shifted_path.path.points.at(i - 1).lane_ids;
    }
  }

  // set pull over path
  auto pull_over_path_opt = PullOverPath::create(
    getPlannerType(), id, {shifted_path.path}, path_shifter.getShiftLines().front().start,
    goal_candidate, {std::make_pair(pull_over_velocity, 0)});

  if (!pull_over_path_opt) {
    return {};
  }
  auto & pull_over_path = pull_over_path_opt.value();
  pull_over_path.debug_poses.push_back(shift_end_pose_prev_module_path);
  pull_over_path.debug_poses.push_back(actual_shift_end_pose);
  pull_over_path.debug_poses.push_back(goal_pose);
  pull_over_path.debug_poses.push_back(shift_end_pose);
  pull_over_path.debug_poses.push_back(
    road_lane_reference_path_to_shift_end.points.back().point.pose);
  pull_over_path.debug_poses.push_back(prev_module_path_terminal_pose);

  // check if the parking path will leave drivable area and lanes
  const bool is_in_parking_lots = std::invoke([&]() -> bool {
    const auto & p = planner_data->parameters;
    const auto parking_lot_polygons =
      lanelet::utils::query::getAllParkingLots(planner_data->route_handler->getLaneletMapPtr());
    const auto path_footprints = goal_planner_utils::createPathFootPrints(
      pull_over_path.parking_path(), p.base_link2front, p.base_link2rear, p.vehicle_width);
    const auto is_footprint_in_any_polygon = [&parking_lot_polygons](const auto & footprint) {
      return std::any_of(
        parking_lot_polygons.begin(), parking_lot_polygons.end(),
        [&footprint](const auto & polygon) {
          return lanelet::geometry::within(footprint, lanelet::utils::to2D(polygon).basicPolygon());
        });
    };
    return std::all_of(
      path_footprints.begin(), path_footprints.end(),
      [&is_footprint_in_any_polygon](const auto & footprint) {
        return is_footprint_in_any_polygon(footprint);
      });
  });

  const auto departure_check_lane = goal_planner_utils::createDepartureCheckLanelet(
    pull_over_lanes, *planner_data->route_handler, left_side_parking_);
  const bool is_in_lanes = !lane_departure_checker_.checkPathWillLeaveLane(
    {departure_check_lane}, pull_over_path.parking_path());

  if (!is_in_parking_lots && !is_in_lanes) {
    return {};
  }

  return pull_over_path;
}

double ShiftPullOver::calcBeforeShiftedArcLength(
  const PathWithLaneId & path, const double target_after_arc_length, const double dr)
{
  // reverse path for checking from the end point
  // note that the sign of curvature is also reversed
  PathWithLaneId reversed_path{};
  std::reverse_copy(
    path.points.begin(), path.points.end(), std::back_inserter(reversed_path.points));

  double before_arc_length{0.0};
  double after_arc_length{0.0};

  const auto curvature_and_segment_length =
    autoware::motion_utils::calcCurvatureAndSegmentLength(reversed_path.points);

  for (size_t i = 0; i < curvature_and_segment_length.size(); ++i) {
    const auto & [k, segment_length_pair] = curvature_and_segment_length[i];

    // If it is the last point, add the lengths of the previous and next segments.
    // For other points, only add the length of the previous segment.
    const double segment_length = i == curvature_and_segment_length.size() - 1
                                    ? segment_length_pair.first
                                    : segment_length_pair.first + segment_length_pair.second;

    // after shifted segment length
    const double after_segment_length =
      k > 0 ? segment_length * (1 + k * dr) : segment_length / (1 - k * dr);
    if (after_arc_length + after_segment_length > target_after_arc_length) {
      const double offset = target_after_arc_length - after_arc_length;
      before_arc_length += k > 0 ? offset / (1 + k * dr) : offset * (1 - k * dr);
      break;
    }
    before_arc_length += segment_length;
    after_arc_length += after_segment_length;
  }

  return before_arc_length;
}
}  // namespace autoware::behavior_path_planner
