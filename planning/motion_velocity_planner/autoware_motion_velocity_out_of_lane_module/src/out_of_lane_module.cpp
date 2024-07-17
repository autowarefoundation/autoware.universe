// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include "out_of_lane_module.hpp"

#include "calculate_slowdown_points.hpp"
#include "debug.hpp"
#include "filter_predicted_objects.hpp"
#include "footprint.hpp"
#include "lanelets_selection.hpp"
#include "types.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/ttc_utils.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>
#include <autoware/universe_utils/ros/update_param.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>

#include <geometry_msgs/msg/detail/point__struct.hpp>

#include <boost/geometry/algorithms/detail/disjoint/interface.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void OutOfLaneModule::init(rclcpp::Node & node, const std::string & module_name)
{
  module_name_ = module_name;
  logger_ = node.get_logger();
  clock_ = node.get_clock();
  init_parameters(node);
  velocity_factor_interface_.init(autoware::motion_utils::PlanningBehavior::ROUTE_OBSTACLE);

  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/debug_markers", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/virtual_walls", 1);
  processing_diag_publisher_ = std::make_shared<autoware::universe_utils::ProcessingTimePublisher>(
    &node, "~/debug/" + ns_ + "/processing_time_ms_diag");
  processing_time_publisher_ = node.create_publisher<tier4_debug_msgs::msg::Float64Stamped>(
    "~/debug/" + ns_ + "/processing_time_ms", 1);
}
void OutOfLaneModule::init_parameters(rclcpp::Node & node)
{
  using autoware::universe_utils::getOrDeclareParameter;
  auto & pp = params_;

  pp.mode = getOrDeclareParameter<std::string>(node, ns_ + ".mode");
  pp.skip_if_already_overlapping =
    getOrDeclareParameter<bool>(node, ns_ + ".skip_if_already_overlapping");

  pp.time_threshold = getOrDeclareParameter<double>(node, ns_ + ".threshold.time_threshold");
  pp.ttc_threshold = getOrDeclareParameter<double>(node, ns_ + ".ttc.threshold");

  pp.objects_min_vel = getOrDeclareParameter<double>(node, ns_ + ".objects.minimum_velocity");
  pp.objects_use_predicted_paths =
    getOrDeclareParameter<bool>(node, ns_ + ".objects.use_predicted_paths");
  pp.objects_min_confidence =
    getOrDeclareParameter<double>(node, ns_ + ".objects.predicted_path_min_confidence");
  pp.objects_dist_buffer = getOrDeclareParameter<double>(node, ns_ + ".objects.distance_buffer");
  pp.objects_cut_predicted_paths_beyond_red_lights =
    getOrDeclareParameter<bool>(node, ns_ + ".objects.cut_predicted_paths_beyond_red_lights");
  pp.objects_ignore_behind_ego =
    getOrDeclareParameter<bool>(node, ns_ + ".objects.ignore_behind_ego");

  pp.overlap_min_dist = getOrDeclareParameter<double>(node, ns_ + ".overlap.minimum_distance");
  pp.overlap_extra_length = getOrDeclareParameter<double>(node, ns_ + ".overlap.extra_length");

  pp.skip_if_over_max_decel =
    getOrDeclareParameter<bool>(node, ns_ + ".action.skip_if_over_max_decel");
  pp.precision = getOrDeclareParameter<double>(node, ns_ + ".action.precision");
  pp.min_decision_duration = getOrDeclareParameter<double>(node, ns_ + ".action.min_duration");
  pp.lon_dist_buffer =
    getOrDeclareParameter<double>(node, ns_ + ".action.longitudinal_distance_buffer");
  pp.lat_dist_buffer = getOrDeclareParameter<double>(node, ns_ + ".action.lateral_distance_buffer");
  pp.slow_velocity = getOrDeclareParameter<double>(node, ns_ + ".action.slowdown.velocity");
  pp.stop_dist_threshold =
    getOrDeclareParameter<double>(node, ns_ + ".action.stop.distance_threshold");

  pp.ego_min_velocity = getOrDeclareParameter<double>(node, ns_ + ".ego.min_assumed_velocity");
  pp.extra_front_offset = getOrDeclareParameter<double>(node, ns_ + ".ego.extra_front_offset");
  pp.extra_rear_offset = getOrDeclareParameter<double>(node, ns_ + ".ego.extra_rear_offset");
  pp.extra_left_offset = getOrDeclareParameter<double>(node, ns_ + ".ego.extra_left_offset");
  pp.extra_right_offset = getOrDeclareParameter<double>(node, ns_ + ".ego.extra_right_offset");
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  pp.front_offset = vehicle_info.max_longitudinal_offset_m;
  pp.rear_offset = vehicle_info.min_longitudinal_offset_m;
  pp.left_offset = vehicle_info.max_lateral_offset_m;
  pp.right_offset = vehicle_info.min_lateral_offset_m;
}

void OutOfLaneModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware::universe_utils::updateParam;
  auto & pp = params_;
  updateParam(parameters, ns_ + ".mode", pp.mode);
  updateParam(parameters, ns_ + ".skip_if_already_overlapping", pp.skip_if_already_overlapping);

  updateParam(parameters, ns_ + ".threshold.time_threshold", pp.time_threshold);
  updateParam(parameters, ns_ + ".ttc.threshold", pp.ttc_threshold);

  updateParam(parameters, ns_ + ".objects.minimum_velocity", pp.objects_min_vel);
  updateParam(parameters, ns_ + ".objects.use_predicted_paths", pp.objects_use_predicted_paths);
  updateParam(
    parameters, ns_ + ".objects.predicted_path_min_confidence", pp.objects_min_confidence);
  updateParam(parameters, ns_ + ".objects.distance_buffer", pp.objects_dist_buffer);
  updateParam(
    parameters, ns_ + ".objects.cut_predicted_paths_beyond_red_lights",
    pp.objects_cut_predicted_paths_beyond_red_lights);
  updateParam(parameters, ns_ + ".objects.ignore_behind_ego", pp.objects_ignore_behind_ego);
  updateParam(parameters, ns_ + ".overlap.minimum_distance", pp.overlap_min_dist);
  updateParam(parameters, ns_ + ".overlap.extra_length", pp.overlap_extra_length);

  updateParam(parameters, ns_ + ".action.skip_if_over_max_decel", pp.skip_if_over_max_decel);
  updateParam(parameters, ns_ + ".action.precision", pp.precision);
  updateParam(parameters, ns_ + ".action.min_duration", pp.min_decision_duration);
  updateParam(parameters, ns_ + ".action.longitudinal_distance_buffer", pp.lon_dist_buffer);
  updateParam(parameters, ns_ + ".action.lateral_distance_buffer", pp.lat_dist_buffer);
  updateParam(parameters, ns_ + ".action.slowdown.velocity", pp.slow_velocity);
  updateParam(parameters, ns_ + ".action.stop.distance_threshold", pp.stop_dist_threshold);

  updateParam(parameters, ns_ + ".ego.min_assumed_velocity", pp.ego_min_velocity);
  updateParam(parameters, ns_ + ".ego.extra_front_offset", pp.extra_front_offset);
  updateParam(parameters, ns_ + ".ego.extra_rear_offset", pp.extra_rear_offset);
  updateParam(parameters, ns_ + ".ego.extra_left_offset", pp.extra_left_offset);
  updateParam(parameters, ns_ + ".ego.extra_right_offset", pp.extra_right_offset);
}

VelocityPlanningResult OutOfLaneModule::plan(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  VelocityPlanningResult result;
  autoware::universe_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();

  out_of_lane::EgoData ego_data;
  ego_data.pose = planner_data->current_odometry.pose.pose;
  ego_data.trajectory_points = ego_trajectory_points;
  ego_data.first_trajectory_idx =
    autoware::motion_utils::findNearestSegmentIndex(ego_trajectory_points, ego_data.pose.position);
  ego_data.longitudinal_offset_to_first_trajectory_index =
    motion_utils::calcLongitudinalOffsetToSegment(
      ego_trajectory_points, ego_data.first_trajectory_idx, ego_data.pose.position);
  ego_data.min_stop_distance = planner_data->calculate_min_deceleration_distance(0.0).value_or(0.0);
  ego_data.min_slowdown_distance =
    planner_data->calculate_min_deceleration_distance(params_.slow_velocity).value_or(0.0);
  if (prev_slowdown_pose_) {
    // Ensure we do not remove the previous slowdown point due to the min distance limit
    const auto prev_slowdown_pose_arc_length = motion_utils::calcSignedArcLength(
      ego_data.trajectory_points, ego_data.first_trajectory_idx, prev_slowdown_pose_->position);
    ego_data.min_stop_distance =
      std::min(prev_slowdown_pose_arc_length, ego_data.min_stop_distance);
    ego_data.min_slowdown_distance =
      std::min(prev_slowdown_pose_arc_length, ego_data.min_slowdown_distance);
  }

  stopwatch.tic("calculate_trajectory_footprints");
  const auto current_ego_footprint =
    out_of_lane::calculate_current_ego_footprint(ego_data, params_, true);
  const auto trajectory_footprints =
    out_of_lane::calculate_trajectory_footprints(ego_data, params_);
  const auto calculate_trajectory_footprints_us = stopwatch.toc("calculate_trajectory_footprints");
  // Calculate lanelets to ignore and consider
  stopwatch.tic("calculate_lanelets");
  constexpr auto use_route_to_get_route_lanelets = true;  // TODO(Maxime): param
  const auto route_lanelets =
    use_route_to_get_route_lanelets
      ? planner_data->route_handler->getRouteLanelets()
      : out_of_lane::calculate_trajectory_lanelets(ego_data, planner_data->route_handler);
  const auto ignored_lanelets =
    out_of_lane::calculate_ignored_lanelets(route_lanelets, planner_data->route_handler);
  for (const auto & ll : route_lanelets) {
    out_of_lane::Polygons tmp;
    boost::geometry::union_(ego_data.drivable_lane_polygons, ll.polygon2d().basicPolygon(), tmp);
    ego_data.drivable_lane_polygons = tmp;
  }
  for (const auto & ll : ignored_lanelets) {
    out_of_lane::Polygons tmp;
    boost::geometry::union_(ego_data.drivable_lane_polygons, ll.polygon2d().basicPolygon(), tmp);
    ego_data.drivable_lane_polygons = tmp;
  }
  const auto calculate_lanelets_us = stopwatch.toc("calculate_lanelets");

  // TODO(Maxime): remove data that can be passed using the ego/out_of_lane data
  debug_data_.reset_data();
  debug_data_.route_lanelets = route_lanelets;
  debug_data_.ignored_lanelets = ignored_lanelets;
  debug_data_.trajectory_points = ego_trajectory_points;
  debug_data_.footprints = trajectory_footprints;
  debug_data_.first_trajectory_idx = ego_data.first_trajectory_idx;
  debug_data_.current_footprint = current_ego_footprint;

  if (
    params_.skip_if_already_overlapping && !ego_data.drivable_lane_polygons.empty() &&
    !lanelet::geometry::within(current_ego_footprint, ego_data.drivable_lane_polygons)) {
    RCLCPP_WARN(logger_, "Ego is already overlapping a lane, skipping the module\n");
    return result;
  }
  // Calculate overlapping ranges
  stopwatch.tic("calculate_out_of_lane_areas");
  out_of_lane::OutOfLaneData out_of_lane_data;
  out_of_lane_data.out_of_lane_lanelets = out_of_lane::calculate_out_of_lane_lanelets(
    ego_data, route_lanelets, ignored_lanelets, planner_data->route_handler, params_);
  for (auto i = 0UL; i < trajectory_footprints.size(); ++i) {
    const auto & footprint = trajectory_footprints[i];
    if (!boost::geometry::disjoint(footprint, ego_data.drivable_lane_polygons)) {
      out_of_lane::OutOfLanePoint p;
      p.trajectory_index = i + ego_data.first_trajectory_idx;
      out_of_lane::Polygons out_of_lane_polygons;
      boost::geometry::difference(footprint, ego_data.drivable_lane_polygons, out_of_lane_polygons);
      for (const auto & area : out_of_lane_polygons) {
        p.outside_rings.push_back(area.outer);
      }
      out_of_lane_data.outside_points.push_back(p);
    }
  }
  const auto calculate_out_of_lane_areas_us = stopwatch.toc("calculate_out_of_lane_areas");
  stopwatch.tic("filter_predicted_objects");
  const auto objects = out_of_lane::filter_predicted_objects(planner_data, ego_data, params_);
  const auto filter_predicted_objects_us = stopwatch.toc("filter_predicted_objects");

  for (const auto & object : objects.objects) {
    const auto time_collisions = calculate_time_collisions_along_trajectory(
      *planner_data->ego_trajectory_collision_checker, object);
    for (auto & out_of_lane_point : out_of_lane_data.outside_points) {
      for (const auto & [t, points] : time_collisions[out_of_lane_point.trajectory_index]) {
        auto & collision_points = out_of_lane_point.time_collisions[t];
        collision_points.insert(collision_points.end(), points.begin(), points.end());
      }
    }
  }
  // calculate times
  for (auto & out_of_lane_point : out_of_lane_data.outside_points) {
    auto min_time = std::numeric_limits<double>::infinity();
    auto max_time = -std::numeric_limits<double>::infinity();
    for (const auto & [t, points] : out_of_lane_point.time_collisions) {
      for (const auto & out_of_lane_area : out_of_lane_point.outside_rings) {
        if (!boost::geometry::disjoint(out_of_lane_area, points)) {
          min_time = std::min(t, min_time);
          max_time = std::max(t, max_time);
          break;
        }
      }
    }
    if (min_time <= max_time) {
      out_of_lane_point.min_object_arrival_time = min_time;
      out_of_lane_point.max_object_arrival_time = max_time;
      const auto & ego_time =
        rclcpp::Duration(ego_trajectory_points[out_of_lane_point.trajectory_index].time_from_start)
          .seconds();
      if (ego_time >= min_time && ego_time <= max_time) {
        out_of_lane_point.ttc = 0.0;
      } else {
        out_of_lane_point.ttc =
          std::min(std::abs(ego_time - min_time), std::abs(ego_time - max_time));
      }
    }
  }
  if (  // reset the previous inserted point if the timer expired
    prev_slowdown_pose_ &&
    (clock_->now() - prev_inserted_pose_time_).seconds() > params_.min_decision_duration) {
    prev_slowdown_pose_.reset();
  }
  auto slowdown_pose = out_of_lane::calculate_slowdown_point(ego_data, out_of_lane_data, params_);
  // if (  // reset the timer if there is no previous inserted point or if we avoid the same lane
  //   slowdown_point &&
  //   (!prev_inserted_pose_ || prev_inserted_pose_->slowdown.lane_to_avoid.id() ==
  //                               point_to_insert->slowdown.lane_to_avoid.id())) {
  //   prev_inserted_pose_time_ = clock_->now();
  // }
  // reuse previous stop pose if there is no new one or if its velocity is not higher than the new
  // one and its arc length is lower
  const auto should_use_prev_inserted_pose = [&]() {
    if (slowdown_pose && prev_slowdown_pose_) {
      const auto arc_length = autoware::motion_utils::calcSignedArcLength(
        ego_trajectory_points, 0LU, slowdown_pose->position);
      const auto prev_arc_length = autoware::motion_utils::calcSignedArcLength(
        ego_trajectory_points, 0LU, prev_slowdown_pose_->position);
      return prev_arc_length < arc_length;
    }
    return !slowdown_pose && prev_slowdown_pose_;
  }();
  if (should_use_prev_inserted_pose) {
    // if the trajectory changed the prev point is no longer on the trajectory so we project it
    const auto new_arc_length = autoware::motion_utils::calcSignedArcLength(
      ego_trajectory_points, ego_data.first_trajectory_idx, prev_slowdown_pose_->position);
    slowdown_pose =
      autoware::motion_utils::calcInterpolatedPose(ego_trajectory_points, new_arc_length);
  }
  if (slowdown_pose) {
    const auto arc_length =
      autoware::motion_utils::calcSignedArcLength(
        ego_trajectory_points, ego_data.first_trajectory_idx, slowdown_pose->position) -
      ego_data.longitudinal_offset_to_first_trajectory_index;
    const auto slowdown_velocity =
      arc_length <= params_.stop_dist_threshold ? 0.0 : params_.slow_velocity;
    prev_slowdown_pose_ = slowdown_pose;
    // RCLCPP_DEBUG(logger_, "Avoiding lane %lu", point_to_insert->slowdown.lane_to_avoid.id());
    if (slowdown_velocity == 0.0)
      result.stop_points.push_back(slowdown_pose->position);
    else
      result.slowdown_intervals.emplace_back(
        slowdown_pose->position, slowdown_pose->position, slowdown_velocity);

    const auto is_approaching =
      autoware::motion_utils::calcSignedArcLength(
        ego_trajectory_points, ego_data.pose.position, slowdown_pose->position) > 0.1 &&
      planner_data->current_odometry.twist.twist.linear.x > 0.1;
    const auto status = is_approaching ? autoware::motion_utils::VelocityFactor::APPROACHING
                                       : autoware::motion_utils::VelocityFactor::STOPPED;
    velocity_factor_interface_.set(
      ego_trajectory_points, ego_data.pose, *slowdown_pose, status, "out_of_lane");
    result.velocity_factor = velocity_factor_interface_.get();
  } else if (!out_of_lane_data.outside_points.empty()) {
    RCLCPP_WARN(logger_, "Could not insert stop point because of deceleration limits");
  }
  const auto total_time_us = stopwatch.toc();
  // RCLCPP_DEBUG(
  //   logger_,
  //   "Total time = %2.2fus\n"
  //   "\tcalculate_lanelets = %2.0fus\n"
  //   "\tcalculate_trajectory_footprints = %2.0fus\n"
  //   "\tcalculate_overlapping_areas = %2.0fus\n"
  //   "\tfilter_pred_objects = %2.0fus\n"
  //   total_time_us, calculate_lanelets_us, calculate_trajectory_footprints_us,
  //   calculate_overlapping_areas_us, filter_predicted_objects_us, calculate_decisions_us,
  //   calc_slowdown_points_us, insert_slowdown_points_us);
  // debug_publisher_->publish(out_of_lane::debug::create_debug_marker_array(debug_data_));
  // virtual_wall_marker_creator.add_virtual_walls(
  //   out_of_lane::debug::create_virtual_walls(, params_));
  virtual_wall_publisher_->publish(virtual_wall_marker_creator.create_markers(clock_->now()));
  std::map<std::string, double> processing_times;
  processing_times["calculate_lanelets"] = calculate_lanelets_us / 1000;
  processing_times["calculate_trajectory_footprints"] = calculate_trajectory_footprints_us / 1000;
  processing_times["calculate_out_of_lane_areas"] = calculate_out_of_lane_areas_us / 1000;
  processing_times["filter_pred_objects"] = filter_predicted_objects_us / 1000;
  processing_times["Total"] = total_time_us / 1000;
  processing_diag_publisher_->publish(processing_times);
  tier4_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = clock_->now();
  processing_time_msg.data = processing_times["Total"];
  processing_time_publisher_->publish(processing_time_msg);
  return result;
}

}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::OutOfLaneModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
