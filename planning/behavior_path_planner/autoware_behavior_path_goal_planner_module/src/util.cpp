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

#include "autoware/behavior_path_goal_planner_module/util.hpp"

#include "autoware/behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware_lanelet2_extension/regulatory_elements/bus_stop_area.hpp"

#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::goal_planner_utils
{

using autoware::universe_utils::calcDistance2d;
using autoware::universe_utils::calcOffsetPose;
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerScale;
using autoware::universe_utils::createPoint;

lanelet::ConstLanelets getPullOverLanes(
  const RouteHandler & route_handler, const bool left_side, const double backward_distance,
  const double forward_distance)
{
  const Pose goal_pose = route_handler.getOriginalGoalPose();

  // Buffer to get enough lanes in front of the goal, need much longer than the pull over distance.
  // In the case of loop lanes, it may not be possible to extend the lane forward.
  // todo(kosuek55): automatically calculates this distance.
  const double backward_distance_with_buffer = backward_distance + 100;

  const auto target_shoulder_lane = route_handler.getPullOverTarget(goal_pose);
  if (target_shoulder_lane) {
    // pull over on shoulder lane
    return route_handler.getShoulderLaneletSequence(
      *target_shoulder_lane, goal_pose, backward_distance_with_buffer, forward_distance);
  }

  lanelet::ConstLanelet closest_lane{};
  route_handler.getClosestLaneletWithinRoute(goal_pose, &closest_lane);
  lanelet::ConstLanelet outermost_lane{};
  if (left_side) {
    outermost_lane = route_handler.getMostLeftLanelet(closest_lane, false, true);
  } else {
    outermost_lane = route_handler.getMostRightLanelet(closest_lane, false, true);
  }

  constexpr bool only_route_lanes = false;
  return route_handler.getLaneletSequence(
    outermost_lane, backward_distance_with_buffer, forward_distance, only_route_lanes);
}

static double getOffsetToLanesBoundary(
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose target_pose,
  const bool left_side)
{
  lanelet::ConstLanelet closest_lanelet;
  lanelet::utils::query::getClosestLanelet(lanelet_sequence, target_pose, &closest_lanelet);

  // the boundary closer to ego. if left_side, take right boundary
  const auto & boundary3d = left_side ? closest_lanelet.rightBound() : closest_lanelet.leftBound();
  const auto boundary = lanelet::utils::to2D(boundary3d);
  using lanelet::utils::conversion::toLaneletPoint;
  const auto arc_coords = lanelet::geometry::toArcCoordinates(
    boundary, lanelet::utils::to2D(toLaneletPoint(target_pose.position)).basicPoint());
  return arc_coords.distance;
}

lanelet::ConstLanelets generateBetweenEgoAndExpandedPullOverLanes(
  const lanelet::ConstLanelets & pull_over_lanes, const bool left_side,
  const geometry_msgs::msg::Pose ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double outer_road_offset,
  const double inner_road_offset)
{
  const double front_overhang = vehicle_info.front_overhang_m,
               wheel_base = vehicle_info.wheel_base_m, wheel_tread = vehicle_info.wheel_tread_m;
  const double side_overhang =
    left_side ? vehicle_info.left_overhang_m : vehicle_info.right_overhang_m;
  const double ego_length_to_front = wheel_base + front_overhang;
  const double ego_width_to_front =
    !left_side ? (-wheel_tread / 2.0 - side_overhang) : (wheel_tread / 2.0 + side_overhang);
  autoware::universe_utils::Point2d front_edge_local{ego_length_to_front, ego_width_to_front};
  const auto front_edge_glob = autoware::universe_utils::transformPoint(
    front_edge_local, autoware::universe_utils::pose2transform(ego_pose));
  geometry_msgs::msg::Pose ego_front_pose;
  ego_front_pose.position =
    createPoint(front_edge_glob.x(), front_edge_glob.y(), ego_pose.position.z);

  // ==========================================================================================
  // NOTE: the point which is on the right side of a directed line has negative distance
  // getExpandedLanelet(1.0, -2.0) expands a lanelet by 1.0 to the left and by 2.0 to the right
  // ==========================================================================================
  const double ego_offset_to_closer_boundary =
    getOffsetToLanesBoundary(pull_over_lanes, ego_front_pose, left_side);
  return left_side ? lanelet::utils::getExpandedLanelets(
                       pull_over_lanes, outer_road_offset,
                       ego_offset_to_closer_boundary - inner_road_offset)
                   : lanelet::utils::getExpandedLanelets(
                       pull_over_lanes, ego_offset_to_closer_boundary + inner_road_offset,
                       -outer_road_offset);
}

std::optional<Polygon2d> generateObjectExtractionPolygon(
  const lanelet::ConstLanelets & pull_over_lanes, const bool left_side, const double outer_offset,
  const double inner_offset)
{
  // generate base boundary poses without orientation
  std::vector<Pose> base_boundary_poses{};
  for (const auto & lane : pull_over_lanes) {
    const auto & bound = left_side ? lane.leftBound() : lane.rightBound();
    for (const auto & p : bound) {
      Pose pose{};
      pose.position = createPoint(p.x(), p.y(), p.z());
      if (std::any_of(base_boundary_poses.begin(), base_boundary_poses.end(), [&](const auto & p) {
            return calcDistance2d(p.position, pose.position) < 0.1;
          })) {
        continue;
      }
      base_boundary_poses.push_back(pose);
    }
  }
  if (base_boundary_poses.size() < 2) {
    return std::nullopt;
  }

  // set orientation to next point
  for (auto it = base_boundary_poses.begin(); it != std::prev(base_boundary_poses.end()); ++it) {
    const auto & p = it->position;
    const auto & next_p = std::next(it)->position;
    const double yaw = autoware::universe_utils::calcAzimuthAngle(p, next_p);
    it->orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);
  }
  base_boundary_poses.back().orientation =
    base_boundary_poses[base_boundary_poses.size() - 2].orientation;

  // generate outer and inner boundary poses
  std::vector<Point> outer_boundary_points{};
  std::vector<Point> inner_boundary_points{};
  const double outer_direction_sign = left_side ? 1.0 : -1.0;
  for (const auto & base_pose : base_boundary_poses) {
    const Pose outer_pose = calcOffsetPose(base_pose, 0, outer_direction_sign * outer_offset, 0);
    const Pose inner_pose = calcOffsetPose(base_pose, 0, -outer_direction_sign * inner_offset, 0);
    outer_boundary_points.push_back(outer_pose.position);
    inner_boundary_points.push_back(inner_pose.position);
  }

  // remove self intersection
  // if bound is intersected, remove them and insert intersection point
  using BoostPoint = boost::geometry::model::d2::point_xy<double>;
  using LineString = boost::geometry::model::linestring<BoostPoint>;
  const auto remove_self_intersection = [](const std::vector<Point> & bound) {
    constexpr double INTERSECTION_CHECK_DISTANCE = 10.0;
    std::vector<Point> modified_bound{};
    size_t i = 0;
    while (i < bound.size() - 1) {
      BoostPoint p1(bound.at(i).x, bound.at(i).y);
      BoostPoint p2(bound.at(i + 1).x, bound.at(i + 1).y);
      LineString p_line{};
      p_line.push_back(p1);
      p_line.push_back(p2);
      bool intersection_found = false;
      for (size_t j = i + 2; j < bound.size() - 1; j++) {
        const double distance = autoware::universe_utils::calcDistance2d(bound.at(i), bound.at(j));
        if (distance > INTERSECTION_CHECK_DISTANCE) {
          break;
        }
        LineString q_line{};
        BoostPoint q1(bound.at(j).x, bound.at(j).y);
        BoostPoint q2(bound.at(j + 1).x, bound.at(j + 1).y);
        q_line.push_back(q1);
        q_line.push_back(q2);
        std::vector<BoostPoint> intersection_points;
        boost::geometry::intersection(p_line, q_line, intersection_points);
        if (intersection_points.empty()) {
          continue;
        }
        modified_bound.push_back(bound.at(i));
        Point intersection_point{};
        intersection_point.x = intersection_points.at(0).x();
        intersection_point.y = intersection_points.at(0).y();
        intersection_point.z = bound.at(i).z;
        modified_bound.push_back(intersection_point);
        i = j + 1;
        intersection_found = true;
        break;
      }
      if (!intersection_found) {
        modified_bound.push_back(bound.at(i));
        i++;
      }
    }
    modified_bound.push_back(bound.back());
    return modified_bound;
  };
  outer_boundary_points = remove_self_intersection(outer_boundary_points);
  inner_boundary_points = remove_self_intersection(inner_boundary_points);

  // create clockwise polygon
  Polygon2d polygon{};
  const auto & left_boundary_points = left_side ? outer_boundary_points : inner_boundary_points;
  const auto & right_boundary_points = left_side ? inner_boundary_points : outer_boundary_points;
  std::vector<Point> reversed_right_boundary_points = right_boundary_points;
  std::reverse(reversed_right_boundary_points.begin(), reversed_right_boundary_points.end());
  for (const auto & left_point : left_boundary_points) {
    autoware::universe_utils::Point2d point{left_point.x, left_point.y};
    polygon.outer().push_back(point);
  }
  for (const auto & right_point : reversed_right_boundary_points) {
    autoware::universe_utils::Point2d point{right_point.x, right_point.y};
    polygon.outer().push_back(point);
  }
  autoware::universe_utils::Point2d first_point{
    left_boundary_points.front().x, left_boundary_points.front().y};
  polygon.outer().push_back(first_point);

  if (polygon.outer().size() < 3) {
    return std::nullopt;
  }

  return polygon;
}

PredictedObjects filterObjectsByLateralDistance(
  const Pose & ego_pose, const double vehicle_width, const PredictedObjects & objects,
  const double distance_thresh, const bool filter_inside)
{
  PredictedObjects filtered_objects;
  for (const auto & object : objects.objects) {
    const double distance =
      utils::calcLateralDistanceFromEgoToObject(ego_pose, vehicle_width, object);
    if (filter_inside ? distance < distance_thresh : distance > distance_thresh) {
      filtered_objects.objects.push_back(object);
    }
  }

  return filtered_objects;
}

bool isIntersectingAreas(
  const LinearRing2d & footprint, const std::vector<lanelet::BasicPolygon2d> & areas)
{
  for (const auto & area : areas) {
    if (boost::geometry::intersects(area, footprint)) {
      return true;
    }
  }
  return false;
}

bool isWithinAreas(
  const LinearRing2d & footprint, const std::vector<lanelet::BasicPolygon2d> & areas)
{
  for (const auto & area : areas) {
    if (boost::geometry::within(footprint, area)) {
      return true;
    }
  }
  return false;
}

std::vector<lanelet::BasicPolygon2d> getBusStopAreaPolygons(const lanelet::ConstLanelets & lanes)
{
  std::vector<lanelet::BasicPolygon2d> area_polygons{};
  for (const auto & bus_stop_area_reg_elem : lanelet::utils::query::busStopAreas(lanes)) {
    for (const auto & area : bus_stop_area_reg_elem->busStopAreas()) {
      const auto & area_poly = lanelet::utils::to2D(area).basicPolygon();
      area_polygons.push_back(area_poly);
    }
  }
  return area_polygons;
}

bool checkObjectsCollision(
  const PathWithLaneId & path, const std::vector<double> & curvatures,
  const PredictedObjects & static_target_objects, const PredictedObjects & dynamic_target_objects,
  const BehaviorPathPlannerParameters & behavior_path_parameters,
  const double collision_check_margin, const bool extract_static_objects,
  const double maximum_deceleration,
  const double object_recognition_collision_check_max_extra_stopping_margin,
  std::vector<Polygon2d> & debug_ego_polygons_expanded, const bool update_debug_data)
{
  if (path.points.size() != curvatures.size()) {
    RCLCPP_WARN(
      rclcpp::get_logger("goal_planner_util"),
      "path.points.size() != curvatures.size() in checkObjectsCollision(). judge as non collision");
    return false;
  }

  const auto & target_objects =
    extract_static_objects ? static_target_objects : dynamic_target_objects;
  if (target_objects.objects.empty()) {
    return false;
  }

  // check collision roughly with {min_distance, max_distance} between ego footprint and objects
  // footprint
  std::pair<bool, bool> has_collision_rough =
    utils::path_safety_checker::checkObjectsCollisionRough(
      path, target_objects, collision_check_margin, behavior_path_parameters, false);
  if (!has_collision_rough.first) {
    return false;
  }
  if (has_collision_rough.second) {
    return true;
  }

  std::vector<Polygon2d> obj_polygons;
  for (const auto & object : target_objects.objects) {
    obj_polygons.push_back(autoware::universe_utils::toPolygon2d(object));
  }

  /* Expand ego collision check polygon
   *   - `collision_check_margin` is added in all directions.
   *   - `extra_stopping_margin` adds stopping margin under deceleration constraints forward.
   *   - `extra_lateral_margin` adds the lateral margin on curves.
   */
  std::vector<Polygon2d> ego_polygons_expanded{};
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto p = path.points.at(i);
    const double extra_stopping_margin = std::min(
      std::pow(p.point.longitudinal_velocity_mps, 2) * 0.5 / maximum_deceleration,
      object_recognition_collision_check_max_extra_stopping_margin);

    // The square is meant to imply centrifugal force, but it is not a very well-founded formula.
    // TODO(kosuke55): It is needed to consider better way because there is an inherently
    // different conception of the inside and outside margins.
    const double extra_lateral_margin = std::min(
      extra_stopping_margin,
      std::abs(curvatures.at(i) * std::pow(p.point.longitudinal_velocity_mps, 2)));

    const auto ego_polygon = autoware::universe_utils::toFootprint(
      p.point.pose,
      behavior_path_parameters.base_link2front + collision_check_margin + extra_stopping_margin,
      behavior_path_parameters.base_link2rear + collision_check_margin,
      behavior_path_parameters.vehicle_width + collision_check_margin * 2.0 +
        extra_lateral_margin * 2.0);
    ego_polygons_expanded.push_back(ego_polygon);
  }

  if (update_debug_data) {
    debug_ego_polygons_expanded = ego_polygons_expanded;
  }

  return utils::path_safety_checker::checkPolygonsIntersects(ego_polygons_expanded, obj_polygons);
}

MarkerArray createPullOverAreaMarkerArray(
  const autoware::universe_utils::MultiPolygon2d area_polygons,
  const std_msgs::msg::Header & header, const std_msgs::msg::ColorRGBA & color, const double z)
{
  MarkerArray marker_array{};
  for (size_t i = 0; i < area_polygons.size(); ++i) {
    Marker marker = createDefaultMarker(
      header.frame_id, header.stamp, "pull_over_area_" + std::to_string(i), i,
      visualization_msgs::msg::Marker::LINE_STRIP, createMarkerScale(0.1, 0.0, 0.0), color);
    const auto & poly = area_polygons.at(i);
    for (const auto & p : poly.outer()) {
      marker.points.push_back(createPoint(p.x(), p.y(), z));
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

MarkerArray createPosesMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const std_msgs::msg::ColorRGBA & color)
{
  MarkerArray msg{};
  int32_t i = 0;
  for (const auto & pose : poses) {
    Marker marker = autoware::universe_utils::createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i, Marker::ARROW,
      createMarkerScale(0.5, 0.25, 0.25), color);
    marker.pose = pose;
    marker.id = i++;
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createGoalPriorityTextsMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const std_msgs::msg::ColorRGBA & color)
{
  MarkerArray msg{};
  int32_t i = 0;
  for (const auto & pose : poses) {
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i, Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.3, 0.3, 0.3), color);
    marker.pose = calcOffsetPose(pose, 0, 0, 1.0);
    marker.id = i;
    marker.text = std::to_string(i);
    msg.markers.push_back(marker);
    i++;
  }

  return msg;
}

MarkerArray createNumObjectsToAvoidTextsMarkerArray(
  const GoalCandidates & goal_candidates, std::string && ns, const std_msgs::msg::ColorRGBA & color)
{
  MarkerArray msg{};
  int32_t i = 0;
  for (const auto & goal_candidate : goal_candidates) {
    const Pose & pose = goal_candidate.goal_pose;
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i, Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.3, 0.3, 0.3), color);
    marker.pose = calcOffsetPose(pose, -0.5, 0, 1.0);
    marker.id = i;
    marker.text = std::to_string(goal_candidate.num_objects_to_avoid);
    msg.markers.push_back(marker);
    i++;
  }

  return msg;
}

MarkerArray createGoalCandidatesMarkerArray(
  const GoalCandidates & goal_candidates, const std_msgs::msg::ColorRGBA & color)
{
  GoalCandidates safe_goal_candidates{};
  std::copy_if(
    goal_candidates.begin(), goal_candidates.end(), std::back_inserter(safe_goal_candidates),
    [](const auto & goal_candidate) { return goal_candidate.is_safe; });

  std::vector<Pose> pose_vector{};
  std::transform(
    safe_goal_candidates.begin(), safe_goal_candidates.end(), std::back_inserter(pose_vector),
    [](const auto & goal_candidate) { return goal_candidate.goal_pose; });

  auto marker_array = createPosesMarkerArray(pose_vector, "goal_candidates", color);
  for (const auto & text_marker :
       createGoalPriorityTextsMarkerArray(
         pose_vector, "goal_candidates_priority", createMarkerColor(1.0, 1.0, 1.0, 0.999))
         .markers) {
    marker_array.markers.push_back(text_marker);
  }
  for (const auto & text_marker : createNumObjectsToAvoidTextsMarkerArray(
                                    safe_goal_candidates, "goal_candidates_num_objects_to_avoid",
                                    createMarkerColor(0.5, 0.5, 0.5, 0.999))
                                    .markers) {
    marker_array.markers.push_back(text_marker);
  }

  return marker_array;
}

MarkerArray createLaneletPolygonMarkerArray(
  const lanelet::CompoundPolygon3d & polygon, const std_msgs::msg::Header & header,
  const std::string & ns, const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array{};
  auto marker = createDefaultMarker(
    header.frame_id, header.stamp, ns, 0, visualization_msgs::msg::Marker::LINE_STRIP,
    createMarkerScale(0.1, 0.0, 0.0), color);
  for (const auto & p : polygon) {
    marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
  }
  marker_array.markers.push_back(marker);
  return marker_array;
}

double calcLateralDeviationBetweenPaths(
  const PathWithLaneId & reference_path, const PathWithLaneId & target_path)
{
  double lateral_deviation = 0.0;
  for (const auto & target_point : target_path.points) {
    const size_t nearest_index = autoware::motion_utils::findNearestIndex(
      reference_path.points, target_point.point.pose.position);
    lateral_deviation = std::max(
      lateral_deviation,
      std::abs(autoware::universe_utils::calcLateralDeviation(
        reference_path.points[nearest_index].point.pose, target_point.point.pose.position)));
  }
  return lateral_deviation;
}

bool isReferencePath(
  const PathWithLaneId & reference_path, const PathWithLaneId & target_path,
  const double lateral_deviation_threshold)
{
  return calcLateralDeviationBetweenPaths(reference_path, target_path) <
         lateral_deviation_threshold;
}

std::optional<PathWithLaneId> cropPath(const PathWithLaneId & path, const Pose & end_pose)
{
  const size_t end_idx =
    autoware::motion_utils::findNearestSegmentIndex(path.points, end_pose.position);
  std::vector<PathPointWithLaneId> clipped_points{
    path.points.begin(), path.points.begin() + end_idx};
  if (clipped_points.empty()) {
    return std::nullopt;
  }

  // add projected end pose to clipped points
  PathPointWithLaneId projected_point = clipped_points.back();
  const double offset =
    autoware::motion_utils::calcSignedArcLength(path.points, end_idx, end_pose.position);
  projected_point.point.pose =
    autoware::universe_utils::calcOffsetPose(clipped_points.back().point.pose, offset, 0, 0);
  clipped_points.push_back(projected_point);
  auto clipped_path = path;
  clipped_path.points = clipped_points;

  return clipped_path;
}

PathWithLaneId cropForwardPoints(
  const PathWithLaneId & path, const size_t target_seg_idx, const double forward_length)
{
  const auto & points = path.points;

  double sum_length = 0;
  for (size_t i = target_seg_idx + 1; i < points.size(); ++i) {
    const double seg_length =
      autoware::universe_utils::calcDistance2d(points.at(i), points.at(i - 1));
    if (forward_length < sum_length + seg_length) {
      const auto cropped_points =
        std::vector<PathPointWithLaneId>{points.begin() + target_seg_idx, points.begin() + i};
      PathWithLaneId cropped_path = path;
      cropped_path.points = cropped_points;

      // add precise end pose to cropped points
      const double remaining_length = forward_length - sum_length;
      const Pose precise_end_pose =
        calcOffsetPose(cropped_path.points.back().point.pose, remaining_length, 0, 0);
      if (remaining_length < 0.1) {
        // if precise_end_pose is too close, replace the last point
        cropped_path.points.back().point.pose = precise_end_pose;
      } else {
        auto precise_end_point = cropped_path.points.back();
        precise_end_point.point.pose = precise_end_pose;
        cropped_path.points.push_back(precise_end_point);
      }
      return cropped_path;
    }
    sum_length += seg_length;
  }

  // if forward_length is too long, return points after target_seg_idx
  const auto cropped_points =
    std::vector<PathPointWithLaneId>{points.begin() + target_seg_idx, points.end()};
  PathWithLaneId cropped_path = path;
  cropped_path.points = cropped_points;
  return cropped_path;
}

PathWithLaneId extendPath(
  const PathWithLaneId & target_path, const PathWithLaneId & reference_path,
  const double extend_length, const bool remove_connected_zero_velocity)
{
  const auto & target_terminal_pose = target_path.points.back().point.pose;

  // generate clipped road lane reference path from previous module path terminal pose to shift end
  const size_t target_path_terminal_idx = autoware::motion_utils::findNearestSegmentIndex(
    reference_path.points, target_terminal_pose.position);

  PathWithLaneId clipped_path =
    cropForwardPoints(reference_path, target_path_terminal_idx, extend_length);

  // shift clipped path to previous module path terminal pose
  const double lateral_shift_from_reference_path =
    autoware::motion_utils::calcLateralOffset(reference_path.points, target_terminal_pose.position);
  for (auto & p : clipped_path.points) {
    p.point.pose = autoware::universe_utils::calcOffsetPose(
      p.point.pose, 0, lateral_shift_from_reference_path, 0);
  }

  auto extended_path = target_path;
  auto & target_terminal_vel = extended_path.points.back().point.longitudinal_velocity_mps;
  if (remove_connected_zero_velocity && target_terminal_vel < 0.01) {
    target_terminal_vel = clipped_path.points.front().point.longitudinal_velocity_mps;
  }

  const auto start_point =
    std::find_if(clipped_path.points.begin(), clipped_path.points.end(), [&](const auto & p) {
      const bool is_forward =
        autoware::universe_utils::inverseTransformPoint(p.point.pose.position, target_terminal_pose)
          .x > 0.0;
      const bool is_close = autoware::universe_utils::calcDistance2d(
                              p.point.pose.position, target_terminal_pose.position) < 0.1;
      return is_forward && !is_close;
    });
  std::copy(start_point, clipped_path.points.end(), std::back_inserter(extended_path.points));

  extended_path.points = autoware::motion_utils::removeOverlapPoints(extended_path.points);

  return extended_path;
}

PathWithLaneId extendPath(
  const PathWithLaneId & target_path, const PathWithLaneId & reference_path,
  const Pose & extend_pose, const bool remove_connected_zero_velocity)
{
  const auto & target_terminal_pose = target_path.points.back().point.pose;
  const size_t target_path_terminal_idx = autoware::motion_utils::findNearestSegmentIndex(
    reference_path.points, target_terminal_pose.position);
  const double extend_distance = autoware::motion_utils::calcSignedArcLength(
    reference_path.points, target_path_terminal_idx, extend_pose.position);

  return extendPath(target_path, reference_path, extend_distance, remove_connected_zero_velocity);
}

std::vector<Polygon2d> createPathFootPrints(
  const PathWithLaneId & path, const double base_to_front, const double base_to_rear,
  const double width)
{
  std::vector<Polygon2d> footprints;
  for (const auto & point : path.points) {
    const auto & pose = point.point.pose;
    footprints.push_back(
      autoware::universe_utils::toFootprint(pose, base_to_front, base_to_rear, width));
  }
  return footprints;
}

std::string makePathPriorityDebugMessage(
  const std::vector<size_t> & sorted_path_indices,
  const std::vector<PullOverPath> & pull_over_path_candidates,
  const std::map<size_t, size_t> & goal_id_to_index, const GoalCandidates & goal_candidates,
  const std::map<size_t, double> & path_id_to_rough_margin_map,
  const std::function<bool(const PullOverPath &)> & isSoftMargin,
  const std::function<bool(const PullOverPath &)> & isHighCurvature)
{
  std::stringstream ss;

  // Unsafe goal and its priority are not visible as debug marker in rviz,
  // so exclude unsafe goal from goal_priority
  std::map<size_t, int> goal_id_and_priority;
  for (size_t i = 0; i < goal_candidates.size(); ++i) {
    goal_id_and_priority[goal_candidates[i].id] = goal_candidates[i].is_safe ? i : -1;
  }

  ss << "\n---------------------- path priority ----------------------\n";
  for (size_t i = 0; i < sorted_path_indices.size(); ++i) {
    const auto & path = pull_over_path_candidates[sorted_path_indices[i]];
    // goal_index is same to goal priority including unsafe goal
    const int goal_index = static_cast<int>(goal_id_to_index.at(path.goal_id()));
    const bool is_safe_goal = goal_candidates[goal_index].is_safe;
    const int goal_priority = goal_id_and_priority[path.goal_id()];

    ss << "path_priority: " << i << ", path_type: " << magic_enum::enum_name(path.type())
       << ", path_id: " << path.id() << ", goal_id: " << path.goal_id()
       << ", goal_priority: " << (is_safe_goal ? std::to_string(goal_priority) : "unsafe")
       << ", margin: " << path_id_to_rough_margin_map.at(path.id())
       << (isSoftMargin(path) ? " (soft)" : " (hard)")
       << ", curvature: " << path.parking_path_max_curvature()
       << (isHighCurvature(path) ? " (high)" : " (low)") << "\n";
  }
  ss << "-----------------------------------------------------------\n";
  return ss.str();
}

lanelet::Points3d combineLanePoints(
  const lanelet::Points3d & points, const lanelet::Points3d & points_next)
{
  lanelet::Points3d combined_points;
  std::unordered_set<lanelet::Id> point_ids;
  for (const auto & point : points) {
    if (point_ids.insert(point.id()).second) {
      combined_points.push_back(point);
    }
  }
  for (const auto & point : points_next) {
    if (point_ids.insert(point.id()).second) {
      combined_points.push_back(point);
    }
  }
  return combined_points;
}

lanelet::Lanelet createDepartureCheckLanelet(
  const lanelet::ConstLanelets & pull_over_lanes, const route_handler::RouteHandler & route_handler,
  const bool left_side_parking)
{
  const auto getBoundPoints =
    [&](const lanelet::ConstLanelet & lane, const bool is_outer) -> lanelet::Points3d {
    lanelet::Points3d points;
    const auto & bound = left_side_parking ? (is_outer ? lane.leftBound() : lane.rightBound())
                                           : (is_outer ? lane.rightBound() : lane.leftBound());
    for (const auto & pt : bound) {
      points.push_back(lanelet::Point3d(pt));
    }
    return points;
  };

  const auto getMostInnerLane = [&](const lanelet::ConstLanelet & lane) -> lanelet::ConstLanelet {
    const auto getInnerLane =
      left_side_parking ? &RouteHandler::getMostRightLanelet : &RouteHandler::getMostLeftLanelet;
    const auto getOppositeLane = left_side_parking ? &RouteHandler::getRightOppositeLanelets
                                                   : &RouteHandler::getLeftOppositeLanelets;
    const auto inner_lane = (route_handler.*getInnerLane)(lane, true, true);
    const auto opposite_lanes = (route_handler.*getOppositeLane)(inner_lane);
    return opposite_lanes.empty() ? inner_lane : opposite_lanes.front();
  };

  lanelet::Points3d outer_bound_points{};
  lanelet::Points3d inner_bound_points{};
  for (const auto & lane : pull_over_lanes) {
    const auto current_outer_bound_points = getBoundPoints(lane, true);
    const auto most_inner_lane = getMostInnerLane(lane);
    const auto current_inner_bound_points = getBoundPoints(most_inner_lane, false);
    outer_bound_points = combineLanePoints(outer_bound_points, current_outer_bound_points);
    inner_bound_points = combineLanePoints(inner_bound_points, current_inner_bound_points);
  }

  const auto outer_linestring = lanelet::LineString3d(lanelet::InvalId, outer_bound_points);
  const auto inner_linestring = lanelet::LineString3d(lanelet::InvalId, inner_bound_points);
  return lanelet::Lanelet(
    lanelet::InvalId, left_side_parking ? outer_linestring : inner_linestring,
    left_side_parking ? inner_linestring : outer_linestring);
}

std::optional<Pose> calcRefinedGoal(
  const Pose & goal_pose, const std::shared_ptr<RouteHandler> route_handler,
  const bool left_side_parking, const double vehicle_width, const double base_link2front,
  const double base_link2rear, const GoalPlannerParameters & parameters)
{
  const lanelet::ConstLanelets pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *route_handler, left_side_parking, parameters.backward_goal_search_length,
    parameters.forward_goal_search_length);

  lanelet::Lanelet closest_pull_over_lanelet{};
  lanelet::utils::query::getClosestLanelet(pull_over_lanes, goal_pose, &closest_pull_over_lanelet);

  // calc closest center line pose
  Pose center_pose{};
  {
    // find position
    const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(goal_pose.position);
    const auto segment = lanelet::utils::getClosestSegment(
      lanelet::utils::to2D(lanelet_point), closest_pull_over_lanelet.centerline());
    const auto p1 = segment.front().basicPoint();
    const auto p2 = segment.back().basicPoint();
    const auto direction_vector = (p2 - p1).normalized();
    const auto p1_to_goal = lanelet_point.basicPoint() - p1;
    const double s = direction_vector.dot(p1_to_goal);
    const auto refined_point = p1 + direction_vector * s;

    center_pose.position.x = refined_point.x();
    center_pose.position.y = refined_point.y();
    center_pose.position.z = refined_point.z();

    // find orientation
    const double yaw = std::atan2(direction_vector.y(), direction_vector.x());
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, yaw);
    center_pose.orientation = tf2::toMsg(tf_quat);
  }

  const auto distance_from_bound = utils::getSignedDistanceFromBoundary(
    pull_over_lanes, vehicle_width, base_link2front, base_link2rear, center_pose,
    left_side_parking);
  if (!distance_from_bound) {
    return std::nullopt;
  }

  const double sign = left_side_parking ? -1.0 : 1.0;
  const double offset_from_center_line =
    sign * (distance_from_bound.value() + parameters.margin_from_boundary);

  const auto refined_goal_pose = calcOffsetPose(center_pose, 0, -offset_from_center_line, 0);

  return refined_goal_pose;
}

std::optional<Pose> calcClosestPose(
  const lanelet::ConstLineString3d line, const Point & query_point)
{
  const auto segment =
    lanelet::utils::getClosestSegment(lanelet::BasicPoint2d{query_point.x, query_point.y}, line);
  if (segment.empty()) {
    return std::nullopt;
  }

  const Eigen::Vector2d direction(
    (segment.back().basicPoint2d() - segment.front().basicPoint2d()).normalized());
  const Eigen::Vector2d xf(segment.front().basicPoint2d());
  const Eigen::Vector2d x(query_point.x, query_point.y);
  const Eigen::Vector2d p = xf + (x - xf).dot(direction) * direction;

  geometry_msgs::msg::Pose closest_pose;
  closest_pose.position.x = p.x();
  closest_pose.position.y = p.y();
  closest_pose.position.z = query_point.z;

  const double lane_yaw =
    std::atan2(segment.back().y() - segment.front().y(), segment.back().x() - segment.front().x());
  tf2::Quaternion q;
  q.setRPY(0, 0, lane_yaw);
  closest_pose.orientation = tf2::toMsg(q);

  return closest_pose;
}

}  // namespace autoware::behavior_path_planner::goal_planner_utils
