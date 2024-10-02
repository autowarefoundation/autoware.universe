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

#include "scene_no_stopping_area.hpp"

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/interpolation/spline_interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/utility/Optional.h>

#include <limits>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace bg = boost::geometry;

NoStoppingAreaModule::NoStoppingAreaModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::NoStoppingArea & no_stopping_area_reg_elem,
  const PlannerParam & planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
  no_stopping_area_reg_elem_(no_stopping_area_reg_elem),
  planner_param_(planner_param),
  debug_data_()
{
  velocity_factor_.init(PlanningBehavior::NO_STOPPING_AREA);
  state_machine_.setState(StateMachine::State::GO);
  state_machine_.setMarginTime(planner_param_.state_clear_time);
}

std::optional<LineString2d> NoStoppingAreaModule::get_stop_line_geometry2d(
  const tier4_planning_msgs::msg::PathWithLaneId & path, const double stop_line_margin) const
{
  const auto & stop_line = no_stopping_area_reg_elem_.stopLine();
  if (stop_line && stop_line->size() >= 2) {
    // get stop line from map
    return planning_utils::extendLine(
      stop_line.value()[0], stop_line.value()[1], planner_data_->stop_line_extend_length);
  }
  return no_stopping_area::generate_stop_line(
    path, no_stopping_area_reg_elem_.noStoppingAreas(),
    planner_data_->vehicle_info_.vehicle_width_m, stop_line_margin);
}

bool NoStoppingAreaModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  // Store original path
  const auto original_path = *path;
  const auto & predicted_obj_arr_ptr = planner_data_->predicted_objects;
  const auto & current_pose = planner_data_->current_odometry;
  if (path->points.size() <= 2) {
    return true;
  }
  // Reset data
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  *stop_reason = planning_utils::initializeStopReason(StopReason::NO_STOPPING_AREA);

  // Get stop line geometry
  const auto stop_line = get_stop_line_geometry2d(original_path, planner_param_.stop_line_margin);
  if (!stop_line) {
    setSafe(true);
    return true;
  }
  const auto stop_point = arc_lane_utils::createTargetPoint(
    original_path, stop_line.value(), planner_param_.stop_margin,
    planner_data_->vehicle_info_.max_longitudinal_offset_m);
  if (!stop_point) {
    setSafe(true);
    return true;
  }
  const auto & stop_pose = stop_point->second;
  setDistance(autoware::motion_utils::calcSignedArcLength(
    original_path.points, current_pose->pose.position, stop_pose.position));
  if (planning_utils::isOverLine(
        original_path, current_pose->pose, stop_pose, planner_param_.dead_line_margin)) {
    // ego can't stop in front of no stopping area -> GO or OR
    state_machine_.setState(StateMachine::State::GO);
    setSafe(true);
    return true;
  }
  const auto & vi = planner_data_->vehicle_info_;
  const double margin = planner_param_.stop_line_margin;
  const double ego_space_in_front_of_stuck_vehicle =
    margin + vi.vehicle_length_m + planner_param_.stuck_vehicle_front_margin;
  const Polygon2d stuck_vehicle_detect_area = generate_ego_no_stopping_area_lane_polygon(
    *path, current_pose->pose, ego_space_in_front_of_stuck_vehicle,
    planner_param_.detection_area_length);
  const double ego_space_in_front_of_stop_line =
    margin + planner_param_.stop_margin + vi.rear_overhang_m;
  const Polygon2d stop_line_detect_area = generate_ego_no_stopping_area_lane_polygon(
    *path, current_pose->pose, ego_space_in_front_of_stop_line,
    planner_param_.detection_area_length);
  if (stuck_vehicle_detect_area.outer().empty() && stop_line_detect_area.outer().empty()) {
    setSafe(true);
    return true;
  }
  debug_data_.stuck_vehicle_detect_area = toGeomPoly(stuck_vehicle_detect_area);
  debug_data_.stop_line_detect_area = toGeomPoly(stop_line_detect_area);
  // Find stuck vehicle in no stopping area
  const bool is_entry_prohibited_by_stuck_vehicle =
    check_stuck_vehicles_in_no_stopping_area(stuck_vehicle_detect_area, predicted_obj_arr_ptr);
  // Find stop line in no stopping area
  const bool is_entry_prohibited_by_stop_line =
    check_stop_lines_in_no_stopping_area(*path, stop_line_detect_area);
  const bool is_entry_prohibited =
    is_entry_prohibited_by_stuck_vehicle || is_entry_prohibited_by_stop_line;
  if (!is_stoppable(current_pose->pose, stop_point->second)) {
    state_machine_.setState(StateMachine::State::GO);
    setSafe(true);
    return false;
  }

  state_machine_.setStateWithMarginTime(
    is_entry_prohibited ? StateMachine::State::STOP : StateMachine::State::GO,
    logger_.get_child("state_machine"), *clock_);

  setSafe(state_machine_.getState() != StateMachine::State::STOP);
  if (!isActivated()) {
    // ----------------stop reason and stop point--------------------------
    no_stopping_area::insert_stop_point(*path, *stop_point);
    // For virtual wall
    debug_data_.stop_poses.push_back(stop_pose);

    // Create StopReason
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = stop_point->second;
      stop_factor.stop_factor_points = debug_data_.stuck_points;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor_.set(
        path->points, planner_data_->current_odometry->pose, stop_point->second,
        VelocityFactor::UNKNOWN);
    }

    // Create legacy StopReason
    {
      const auto insert_idx = stop_point->first + 1;
      if (
        !first_stop_path_point_index_ ||
        static_cast<int>(insert_idx) < first_stop_path_point_index_) {
        debug_data_.first_stop_pose = stop_pose;
        first_stop_path_point_index_ = static_cast<int>(insert_idx);
      }
    }
  } else if (state_machine_.getState() == StateMachine::State::GO) {
    // reset pass judge if current state is go
    is_stoppable_ = true;
    pass_judged_ = false;
  }
  return true;
}

bool NoStoppingAreaModule::check_stuck_vehicles_in_no_stopping_area(
  const Polygon2d & poly,
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & predicted_obj_arr_ptr)
{
  // stuck points by predicted objects
  for (const auto & object : predicted_obj_arr_ptr->objects) {
    if (!no_stopping_area::is_target_stuck_vehicle_type(object)) {
      continue;  // not target vehicle type
    }
    const auto obj_v = std::fabs(object.kinematics.initial_twist_with_covariance.twist.linear.x);
    if (obj_v > planner_param_.stuck_vehicle_vel_thr) {
      setObjectsOfInterestData(
        object.kinematics.initial_pose_with_covariance.pose, object.shape, ColorName::GREEN);
      continue;  // not stop vehicle
    }
    // check if the footprint is in the stuck detect area
    const Polygon2d obj_footprint = autoware::universe_utils::toPolygon2d(object);
    const bool is_in_stuck_area = !bg::disjoint(obj_footprint, poly);
    if (is_in_stuck_area) {
      RCLCPP_DEBUG(logger_, "stuck vehicle found.");
      setObjectsOfInterestData(
        object.kinematics.initial_pose_with_covariance.pose, object.shape, ColorName::RED);
      for (const auto & p : obj_footprint.outer()) {
        geometry_msgs::msg::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = 0.0;
        debug_data_.stuck_points.emplace_back(point);
      }
      return true;
    }
  }
  return false;
}
bool NoStoppingAreaModule::check_stop_lines_in_no_stopping_area(
  const tier4_planning_msgs::msg::PathWithLaneId & path, const Polygon2d & poly)
{
  const double stop_vel = std::numeric_limits<float>::min();

  // if the detected stop point is near goal, it's ignored.
  static constexpr double close_to_goal_distance = 1.0;

  // stuck points by stop line
  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    const auto p0 = path.points.at(i).point.pose.position;
    const auto p1 = path.points.at(i + 1).point.pose.position;
    const auto v0 = path.points.at(i).point.longitudinal_velocity_mps;
    const auto v1 = path.points.at(i + 1).point.longitudinal_velocity_mps;
    if (v0 > stop_vel && v1 > stop_vel) {
      continue;
    }
    // judge if stop point p0 is near goal, by its distance to the path end.
    const double dist_to_path_end =
      autoware::motion_utils::calcSignedArcLength(path.points, i, path.points.size() - 1);
    if (dist_to_path_end < close_to_goal_distance) {
      // exit with false, cause position is near goal.
      return false;
    }

    const LineString2d line{{p0.x, p0.y}, {p1.x, p1.y}};
    std::vector<Point2d> collision_points;
    bg::intersection(poly, line, collision_points);
    if (!collision_points.empty()) {
      geometry_msgs::msg::Point point;
      point.x = collision_points.front().x();
      point.y = collision_points.front().y();
      point.z = 0.0;
      debug_data_.stuck_points.emplace_back(point);
      return true;
    }
  }
  return false;
}

Polygon2d NoStoppingAreaModule::generate_ego_no_stopping_area_lane_polygon(
  const tier4_planning_msgs::msg::PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose,
  const double margin, const double extra_dist) const
{
  Polygon2d ego_area;  // open polygon
  double dist_from_start_sum = 0.0;
  const double interpolation_interval = 0.5;
  bool is_in_area = false;
  tier4_planning_msgs::msg::PathWithLaneId interpolated_path;
  if (!splineInterpolate(path, interpolation_interval, interpolated_path, logger_)) {
    return ego_area;
  }
  auto & pp = interpolated_path.points;
  /* calc closest index */
  const auto closest_idx_opt =
    autoware::motion_utils::findNearestIndex(interpolated_path.points, ego_pose, 3.0, M_PI_4);
  if (!closest_idx_opt) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, *clock_, 1000 /* ms */, "autoware::motion_utils::findNearestIndex fail");
    return ego_area;
  }
  const size_t closest_idx = closest_idx_opt.value();

  const int num_ignore_nearest = 1;  // Do not consider nearest lane polygon
  size_t ego_area_start_idx = closest_idx + num_ignore_nearest;
  // return if area size is not intentional
  if (no_stopping_area_reg_elem_.noStoppingAreas().size() != 1) {
    return ego_area;
  }
  const auto no_stopping_area = no_stopping_area_reg_elem_.noStoppingAreas().front();
  for (size_t i = closest_idx + num_ignore_nearest; i < pp.size() - 1; ++i) {
    dist_from_start_sum += autoware::universe_utils::calcDistance2d(pp.at(i), pp.at(i - 1));
    const auto & p = pp.at(i).point.pose.position;
    if (bg::within(Point2d{p.x, p.y}, lanelet::utils::to2D(no_stopping_area).basicPolygon())) {
      is_in_area = true;
      break;
    }
    if (dist_from_start_sum > extra_dist) {
      return ego_area;
    }
    ++ego_area_start_idx;
  }
  if (ego_area_start_idx > num_ignore_nearest) {
    ego_area_start_idx--;
  }
  if (!is_in_area) {
    return ego_area;
  }
  double dist_from_area_sum = 0.0;
  // decide end idx with extract distance
  size_t ego_area_end_idx = ego_area_start_idx;
  for (size_t i = ego_area_start_idx; i < pp.size() - 1; ++i) {
    dist_from_start_sum += autoware::universe_utils::calcDistance2d(pp.at(i), pp.at(i - 1));
    const auto & p = pp.at(i).point.pose.position;
    if (!bg::within(Point2d{p.x, p.y}, lanelet::utils::to2D(no_stopping_area).basicPolygon())) {
      dist_from_area_sum += autoware::universe_utils::calcDistance2d(pp.at(i), pp.at(i - 1));
    }
    if (dist_from_start_sum > extra_dist || dist_from_area_sum > margin) {
      break;
    }
    ++ego_area_end_idx;
  }

  const auto width = planner_param_.path_expand_width;
  ego_area = planning_utils::generatePathPolygon(
    interpolated_path, ego_area_start_idx, ego_area_end_idx, width);
  return ego_area;
}

bool NoStoppingAreaModule::is_stoppable(
  const geometry_msgs::msg::Pose & self_pose, const geometry_msgs::msg::Pose & line_pose) const
{
  // get vehicle info and compute pass_judge_line_distance
  const auto current_velocity = planner_data_->current_velocity->twist.linear.x;
  const auto current_acceleration = planner_data_->current_acceleration->accel.accel.linear.x;
  const double max_acc = planner_data_->max_stop_acceleration_threshold;
  const double max_jerk = planner_data_->max_stop_jerk_threshold;
  const double delay_response_time = planner_data_->delay_response_time;
  const double stoppable_distance = planning_utils::calcJudgeLineDistWithJerkLimit(
    current_velocity, current_acceleration, max_acc, max_jerk, delay_response_time);
  const double signed_arc_length =
    arc_lane_utils::calcSignedDistance(self_pose, line_pose.position);
  const bool distance_stoppable = stoppable_distance < signed_arc_length;
  const bool slow_velocity = planner_data_->current_velocity->twist.linear.x < 2.0;
  // ego vehicle is high speed and can't stop before stop line -> GO
  const bool not_stoppable = !distance_stoppable && !slow_velocity;
  // stoppable or not is judged only once
  RCLCPP_DEBUG(
    logger_, "stoppable_dist: %lf signed_arc_length: %lf", stoppable_distance, signed_arc_length);
  if (!distance_stoppable && !pass_judged_) {
    pass_judged_ = true;
    // can't stop using maximum brake consider jerk limit
    if (not_stoppable) {
      // pass through
      is_stoppable_ = false;
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "[NoStoppingArea] can't stop in front of no stopping area");
    } else {
      is_stoppable_ = true;
    }
  }
  return is_stoppable_;
}

}  // namespace autoware::behavior_velocity_planner
