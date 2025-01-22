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

#include "autoware/behavior_path_goal_planner_module/goal_searcher.hpp"

#include "autoware/behavior_path_goal_planner_module/util.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"
#include "autoware_lanelet2_extension/regulatory_elements/no_parking_area.hpp"
#include "autoware_lanelet2_extension/regulatory_elements/no_stopping_area.hpp"
#include "autoware_lanelet2_extension/utility/query.hpp"
#include "autoware_lanelet2_extension/utility/utilities.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <boost/geometry/algorithms/union.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::universe_utils::calcOffsetPose;
using lanelet::autoware::NoParkingArea;
using lanelet::autoware::NoStoppingArea;

// Sort with smaller longitudinal distances taking precedence over smaller lateral distances.
struct SortByLongitudinalDistance
{
  bool prioritize_goals_before_objects{false};
  explicit SortByLongitudinalDistance(bool prioritize_goals_before_objects)
  : prioritize_goals_before_objects(prioritize_goals_before_objects)
  {
  }

  bool operator()(const GoalCandidate & a, const GoalCandidate & b) const noexcept
  {
    if (prioritize_goals_before_objects) {
      if (a.num_objects_to_avoid != b.num_objects_to_avoid) {
        return a.num_objects_to_avoid < b.num_objects_to_avoid;
      }
    }

    const double diff = a.distance_from_original_goal - b.distance_from_original_goal;
    constexpr double eps = 0.01;
    // If the longitudinal distances are approximately equal, sort based on lateral offset.
    if (std::abs(diff) < eps) {
      return a.lateral_offset < b.lateral_offset;
    }
    return a.distance_from_original_goal < b.distance_from_original_goal;
  }
};

// Sort with the weighted sum of the longitudinal distance and the lateral distance weighted by
// lateral_cost.
struct SortByWeightedDistance
{
  double lateral_cost{0.0};
  bool prioritize_goals_before_objects{false};

  SortByWeightedDistance(double cost, bool prioritize_goals_before_objects)
  : lateral_cost(cost), prioritize_goals_before_objects(prioritize_goals_before_objects)
  {
  }

  bool operator()(const GoalCandidate & a, const GoalCandidate & b) const noexcept
  {
    if (prioritize_goals_before_objects) {
      if (a.num_objects_to_avoid != b.num_objects_to_avoid) {
        return a.num_objects_to_avoid < b.num_objects_to_avoid;
      }
    }

    return a.distance_from_original_goal + lateral_cost * a.lateral_offset <
           b.distance_from_original_goal + lateral_cost * b.lateral_offset;
  }
};

GoalSearcher::GoalSearcher(
  const GoalPlannerParameters & parameters, const LinearRing2d & vehicle_footprint)
: GoalSearcherBase{parameters},
  vehicle_footprint_{vehicle_footprint},
  left_side_parking_{parameters.parking_policy == ParkingPolicy::LEFT_SIDE}
{
}

GoalCandidates GoalSearcher::search(const std::shared_ptr<const PlannerData> & planner_data)
{
  GoalCandidates goal_candidates{};

  const auto reference_goal_pose_opt = goal_planner_utils::calcRefinedGoal(
    planner_data->route_handler->getOriginalGoalPose(), planner_data->route_handler,
    left_side_parking_, planner_data->parameters.vehicle_width,
    planner_data->parameters.base_link2front, planner_data->parameters.base_link2rear, parameters_);

  if (!reference_goal_pose_opt) {
    return goal_candidates;
  }
  const auto & reference_goal_pose = reference_goal_pose_opt.value();

  const auto & route_handler = planner_data->route_handler;
  const double forward_length = parameters_.forward_goal_search_length;
  const double backward_length = parameters_.backward_goal_search_length;
  const double margin_from_boundary = parameters_.margin_from_boundary;
  const bool use_bus_stop_area = parameters_.bus_stop_area.use_bus_stop_area;
  const double lateral_offset_interval = use_bus_stop_area
                                           ? parameters_.bus_stop_area.lateral_offset_interval
                                           : parameters_.lateral_offset_interval;
  const double max_lateral_offset = parameters_.max_lateral_offset;
  const double ignore_distance_from_lane_start = parameters_.ignore_distance_from_lane_start;
  const double vehicle_width = planner_data->parameters.vehicle_width;
  const double base_link2front = planner_data->parameters.base_link2front;
  const double base_link2rear = planner_data->parameters.base_link2rear;

  const auto pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *route_handler, left_side_parking_, parameters_.backward_goal_search_length,
    parameters_.forward_goal_search_length);
  const auto departure_check_lane = goal_planner_utils::createDepartureCheckLanelet(
    pull_over_lanes, *route_handler, left_side_parking_);
  const auto goal_arc_coords =
    lanelet::utils::getArcCoordinates(pull_over_lanes, reference_goal_pose);
  const double s_start = std::max(0.0, goal_arc_coords.length - backward_length);
  const double s_end = goal_arc_coords.length + forward_length;
  const double longitudinal_interval = use_bus_stop_area
                                         ? parameters_.bus_stop_area.goal_search_interval
                                         : parameters_.goal_search_interval;
  auto center_line_path = utils::resamplePathWithSpline(
    route_handler->getCenterLinePath(pull_over_lanes, s_start, s_end), longitudinal_interval);

  const auto no_parking_area_polygons = getNoParkingAreaPolygons(pull_over_lanes);
  const auto no_stopping_area_polygons = getNoStoppingAreaPolygons(pull_over_lanes);
  const auto bus_stop_area_polygons = goal_planner_utils::getBusStopAreaPolygons(pull_over_lanes);

  std::vector<Pose> original_search_poses{};  // for search area visualizing
  size_t goal_id = 0;
  for (const auto & p : center_line_path.points) {
    // todo(kosuke55): fix orientation for inverseTransformPoint temporarily
    Pose center_pose = p.point.pose;
    center_pose.orientation =
      autoware::universe_utils::createQuaternionFromYaw(tf2::getYaw(center_pose.orientation));

    // ignore goal_pose near lane start
    const double distance_from_lane_start =
      lanelet::utils::getArcCoordinates(pull_over_lanes, center_pose).length;
    if (distance_from_lane_start < ignore_distance_from_lane_start) {
      continue;
    }

    const auto distance_from_bound = utils::getSignedDistanceFromBoundary(
      pull_over_lanes, vehicle_width, base_link2front, base_link2rear, center_pose,
      left_side_parking_);
    if (!distance_from_bound) continue;

    const double sign = left_side_parking_ ? -1.0 : 1.0;
    const double offset_from_center_line =
      use_bus_stop_area ? -distance_from_bound.value()
                        : -distance_from_bound.value() + sign * margin_from_boundary;
    // original means non lateral offset poses
    const Pose original_search_pose = calcOffsetPose(center_pose, 0, offset_from_center_line, 0);
    const double longitudinal_distance_from_original_goal =
      std::abs(autoware::motion_utils::calcSignedArcLength(
        center_line_path.points, reference_goal_pose.position, original_search_pose.position));
    original_search_poses.push_back(original_search_pose);  // for createAreaPolygon
    Pose search_pose{};
    // search goal_pose in lateral direction
    for (double dy = 0; dy <= max_lateral_offset; dy += lateral_offset_interval) {
      search_pose = calcOffsetPose(original_search_pose, 0, sign * dy, 0);

      const auto transformed_vehicle_footprint =
        transformVector(vehicle_footprint_, autoware::universe_utils::pose2transform(search_pose));

      if (
        parameters_.bus_stop_area.use_bus_stop_area &&
        !goal_planner_utils::isWithinAreas(transformed_vehicle_footprint, bus_stop_area_polygons)) {
        continue;
      }

      if (goal_planner_utils::isIntersectingAreas(
            transformed_vehicle_footprint, no_parking_area_polygons)) {
        // break here to exclude goals located laterally in no_parking_areas
        break;
      }

      if (goal_planner_utils::isIntersectingAreas(
            transformed_vehicle_footprint, no_stopping_area_polygons)) {
        // break here to exclude goals located laterally in no_stopping_areas
        break;
      }

      if (!boost::geometry::within(
            transformed_vehicle_footprint, departure_check_lane.polygon2d().basicPolygon())) {
        continue;
      }

      // modify the goal_pose orientation so that vehicle footprint front heading is parallel to the
      // lane boundary
      const auto vehicle_front_midpoint =
        (transformed_vehicle_footprint.at(vehicle_info_utils::VehicleInfo::FrontLeftIndex) +
         transformed_vehicle_footprint.at(vehicle_info_utils::VehicleInfo::FrontRightIndex)) /
        2.0;
      lanelet::ConstLanelet vehicle_front_closest_lanelet;
      lanelet::utils::query::getClosestLanelet(
        pull_over_lanes, search_pose, &vehicle_front_closest_lanelet);
      const auto vehicle_front_pose_for_bound_opt = goal_planner_utils::calcClosestPose(
        left_side_parking_ ? vehicle_front_closest_lanelet.leftBound()
                           : vehicle_front_closest_lanelet.rightBound(),
        autoware::universe_utils::createPoint(
          vehicle_front_midpoint.x(), vehicle_front_midpoint.y(), search_pose.position.z));
      if (!vehicle_front_pose_for_bound_opt) {
        continue;
      }
      const auto & vehicle_front_pose_for_bound = vehicle_front_pose_for_bound_opt.value();
      GoalCandidate goal_candidate{};
      goal_candidate.goal_pose = search_pose;
      goal_candidate.goal_pose.orientation = vehicle_front_pose_for_bound.orientation;
      goal_candidate.lateral_offset = dy;
      goal_candidate.id = goal_id;
      goal_id++;
      // use longitudinal_distance as distance_from_original_goal
      goal_candidate.distance_from_original_goal = longitudinal_distance_from_original_goal;
      goal_candidates.push_back(goal_candidate);
    }
  }
  createAreaPolygons(original_search_poses, planner_data);

  return goal_candidates;
}

void GoalSearcher::countObjectsToAvoid(
  GoalCandidates & goal_candidates, const PredictedObjects & objects,
  const std::shared_ptr<const PlannerData> & planner_data, const Pose & reference_goal_pose) const
{
  const auto & route_handler = planner_data->route_handler;
  const double forward_length = parameters_.forward_goal_search_length;
  const double backward_length = parameters_.backward_goal_search_length;

  // calculate search start/end pose in pull over lanes
  const auto search_start_end_poses = std::invoke([&]() -> std::pair<Pose, Pose> {
    const auto pull_over_lanes = goal_planner_utils::getPullOverLanes(
      *route_handler, left_side_parking_, parameters_.backward_goal_search_length,
      parameters_.forward_goal_search_length);
    const auto goal_arc_coords =
      lanelet::utils::getArcCoordinates(pull_over_lanes, reference_goal_pose);
    const double s_start = std::max(0.0, goal_arc_coords.length - backward_length);
    const double s_end = goal_arc_coords.length + forward_length;
    const auto center_line_path = utils::resamplePathWithSpline(
      route_handler->getCenterLinePath(pull_over_lanes, s_start, s_end),
      parameters_.goal_search_interval);
    return std::make_pair(
      center_line_path.points.front().point.pose, center_line_path.points.back().point.pose);
  });
  const auto search_start_pose = std::get<0>(search_start_end_poses);
  const auto search_end_pose = std::get<1>(search_start_end_poses);

  // generate current lane center line path to check collision with objects
  const auto current_lanes = utils::getExtendedCurrentLanes(
    planner_data, parameters_.backward_goal_search_length, parameters_.forward_goal_search_length,
    /*forward_only_in_route*/ false);
  const auto current_center_line_path = std::invoke([&]() -> PathWithLaneId {
    const double s_start =
      lanelet::utils::getArcCoordinates(current_lanes, search_start_pose).length;
    const double s_end = lanelet::utils::getArcCoordinates(current_lanes, search_end_pose).length;
    return utils::resamplePathWithSpline(
      route_handler->getCenterLinePath(current_lanes, s_start, s_end), 1.0);
  });

  // reset num_objects_to_avoid
  for (auto & goal_candidate : goal_candidates) {
    goal_candidate.num_objects_to_avoid = 0;
  }

  // count number of objects to avoid
  for (const auto & object : objects.objects) {
    for (const auto & p : current_center_line_path.points) {
      const auto transformed_vehicle_footprint =
        transformVector(vehicle_footprint_, autoware::universe_utils::pose2transform(p.point.pose));
      const auto obj_polygon = autoware::universe_utils::toPolygon2d(object);
      const double distance = boost::geometry::distance(obj_polygon, transformed_vehicle_footprint);
      if (distance > parameters_.object_recognition_collision_check_hard_margins.back()) {
        continue;
      }
      const Pose & object_pose = object.kinematics.initial_pose_with_covariance.pose;
      const double s_object = lanelet::utils::getArcCoordinates(current_lanes, object_pose).length;
      for (auto & goal_candidate : goal_candidates) {
        const Pose & goal_pose = goal_candidate.goal_pose;
        const double s_goal = lanelet::utils::getArcCoordinates(current_lanes, goal_pose).length;
        if (s_object < s_goal) {
          goal_candidate.num_objects_to_avoid++;
        }
      }
      break;
    }
  }
}

void GoalSearcher::update(
  GoalCandidates & goal_candidates,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
  const std::shared_ptr<const PlannerData> & planner_data, const PredictedObjects & objects) const
{
  const auto refined_goal_opt = goal_planner_utils::calcRefinedGoal(
    planner_data->route_handler->getOriginalGoalPose(), planner_data->route_handler,
    left_side_parking_, planner_data->parameters.vehicle_width,
    planner_data->parameters.base_link2front, planner_data->parameters.base_link2rear, parameters_);

  if (!refined_goal_opt) {
    return;
  }

  const auto & refined_goal = refined_goal_opt.value();
  if (parameters_.prioritize_goals_before_objects) {
    countObjectsToAvoid(goal_candidates, objects, planner_data, refined_goal);
  }

  if (parameters_.goal_priority == "minimum_weighted_distance") {
    std::sort(
      goal_candidates.begin(), goal_candidates.end(),
      SortByWeightedDistance(
        parameters_.minimum_weighted_distance_lateral_weight,
        parameters_.prioritize_goals_before_objects));
  } else if (parameters_.goal_priority == "minimum_longitudinal_distance") {
    std::sort(
      goal_candidates.begin(), goal_candidates.end(),
      SortByLongitudinalDistance(parameters_.prioritize_goals_before_objects));
  }

  // update is_safe
  for (auto & goal_candidate : goal_candidates) {
    const Pose goal_pose = goal_candidate.goal_pose;

    // check collision with footprint
    if (checkCollision(goal_pose, objects, occupancy_grid_map)) {
      goal_candidate.is_safe = false;
      continue;
    }

    // check longitudinal margin with pull over lane objects
    constexpr bool filter_inside = true;
    const auto target_objects = goal_planner_utils::filterObjectsByLateralDistance(
      goal_pose, planner_data->parameters.vehicle_width, objects,
      parameters_.object_recognition_collision_check_hard_margins.back(), filter_inside);
    if (checkCollisionWithLongitudinalDistance(
          goal_pose, target_objects, occupancy_grid_map, planner_data)) {
      goal_candidate.is_safe = false;
      continue;
    }

    goal_candidate.is_safe = true;
  }
}

// Note: this function is not just return goal_candidate.is_safe but check collision with
// current planner_data_ and margin scale factor.
// And is_safe is not updated in this function.
bool GoalSearcher::isSafeGoalWithMarginScaleFactor(
  const GoalCandidate & goal_candidate, const double margin_scale_factor,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
  const std::shared_ptr<const PlannerData> & planner_data, const PredictedObjects & objects) const
{
  if (!parameters_.use_object_recognition) {
    return true;
  }

  const Pose goal_pose = goal_candidate.goal_pose;
  const double margin =
    parameters_.object_recognition_collision_check_hard_margins.back() * margin_scale_factor;

  if (utils::checkCollisionBetweenFootprintAndObjects(
        vehicle_footprint_, goal_pose, objects, margin)) {
    return false;
  }

  // check longitudinal margin with pull over lane objects
  constexpr bool filter_inside = true;
  const auto target_objects = goal_planner_utils::filterObjectsByLateralDistance(
    goal_pose, planner_data->parameters.vehicle_width, objects, margin, filter_inside);
  if (checkCollisionWithLongitudinalDistance(
        goal_pose, target_objects, occupancy_grid_map, planner_data)) {
    return false;
  }

  return true;
}

bool GoalSearcher::checkCollision(
  const Pose & pose, const PredictedObjects & objects,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map) const
{
  if (parameters_.use_occupancy_grid_for_goal_search) {
    const Pose pose_grid_coords = global2local(occupancy_grid_map->getMap(), pose);
    const auto idx = pose2index(
      occupancy_grid_map->getMap(), pose_grid_coords, occupancy_grid_map->getParam().theta_size);
    const bool check_out_of_range = false;
    if (occupancy_grid_map->detectCollision(idx, check_out_of_range)) {
      return true;
    }
  }

  if (parameters_.use_object_recognition) {
    if (utils::checkCollisionBetweenFootprintAndObjects(
          vehicle_footprint_, pose, objects,
          parameters_.object_recognition_collision_check_hard_margins.back())) {
      return true;
    }
  }
  return false;
}

bool GoalSearcher::checkCollisionWithLongitudinalDistance(
  const Pose & ego_pose, const PredictedObjects & objects,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  if (
    parameters_.use_occupancy_grid_for_goal_search &&
    parameters_.use_occupancy_grid_for_goal_longitudinal_margin) {
    constexpr bool check_out_of_range = false;
    const double offset = std::max(
      parameters_.longitudinal_margin - parameters_.occupancy_grid_collision_check_margin, 0.0);

    // check forward collision
    const Pose ego_pose_moved_forward = calcOffsetPose(ego_pose, offset, 0, 0);
    const Pose forward_pose_grid_coords =
      global2local(occupancy_grid_map->getMap(), ego_pose_moved_forward);
    const auto forward_idx = pose2index(
      occupancy_grid_map->getMap(), forward_pose_grid_coords,
      occupancy_grid_map->getParam().theta_size);
    if (occupancy_grid_map->detectCollision(forward_idx, check_out_of_range)) {
      return true;
    }

    // check backward collision
    const Pose ego_pose_moved_backward = calcOffsetPose(ego_pose, -offset, 0, 0);
    const Pose backward_pose_grid_coords =
      global2local(occupancy_grid_map->getMap(), ego_pose_moved_backward);
    const auto backward_idx = pose2index(
      occupancy_grid_map->getMap(), backward_pose_grid_coords,
      occupancy_grid_map->getParam().theta_size);
    if (occupancy_grid_map->detectCollision(backward_idx, check_out_of_range)) {
      return true;
    }
  }

  if (parameters_.use_object_recognition) {
    if (
      utils::calcLongitudinalDistanceFromEgoToObjects(
        ego_pose, planner_data->parameters.base_link2front, planner_data->parameters.base_link2rear,
        objects) < parameters_.longitudinal_margin) {
      return true;
    }
  }
  return false;
}

void GoalSearcher::createAreaPolygons(
  std::vector<Pose> original_search_poses, const std::shared_ptr<const PlannerData> & planner_data)
{
  using autoware::universe_utils::MultiPolygon2d;
  using autoware::universe_utils::Point2d;
  using autoware::universe_utils::Polygon2d;

  const double vehicle_width = planner_data->parameters.vehicle_width;
  const double base_link2front = planner_data->parameters.base_link2front;
  const double base_link2rear = planner_data->parameters.base_link2rear;
  const double max_lateral_offset = parameters_.max_lateral_offset;

  const auto appendPointToPolygon =
    [](Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point) {
      Point2d point{};
      point.x() = geom_point.x;
      point.y() = geom_point.y;
      boost::geometry::append(polygon.outer(), point);
    };

  boost::geometry::clear(area_polygons_);
  for (const auto p : original_search_poses) {
    Polygon2d footprint{};

    const double left_front_offset =
      left_side_parking_ ? vehicle_width / 2 : vehicle_width / 2 + max_lateral_offset;
    const Point p_left_front = calcOffsetPose(p, base_link2front, left_front_offset, 0).position;
    appendPointToPolygon(footprint, p_left_front);

    const double right_front_offset =
      left_side_parking_ ? -vehicle_width / 2 - max_lateral_offset : -vehicle_width / 2;
    const Point p_right_front = calcOffsetPose(p, base_link2front, right_front_offset, 0).position;
    appendPointToPolygon(footprint, p_right_front);

    const double right_back_offset =
      left_side_parking_ ? -vehicle_width / 2 - max_lateral_offset : -vehicle_width / 2;
    const Point p_right_back = calcOffsetPose(p, -base_link2rear, right_back_offset, 0).position;
    appendPointToPolygon(footprint, p_right_back);

    const double left_back_offset =
      left_side_parking_ ? vehicle_width / 2 : vehicle_width / 2 + max_lateral_offset;
    const Point p_left_back = calcOffsetPose(p, -base_link2rear, left_back_offset, 0).position;
    appendPointToPolygon(footprint, p_left_back);

    appendPointToPolygon(footprint, p_left_front);

    MultiPolygon2d current_result{};
    boost::geometry::union_(footprint, area_polygons_, current_result);
    area_polygons_ = current_result;
  }
}

BasicPolygons2d GoalSearcher::getNoParkingAreaPolygons(const lanelet::ConstLanelets & lanes) const
{
  BasicPolygons2d area_polygons{};
  for (const auto & ll : lanes) {
    for (const auto & reg_elem : ll.regulatoryElementsAs<NoParkingArea>()) {
      for (const auto & area : reg_elem->noParkingAreas()) {
        const auto & area_poly = lanelet::utils::to2D(area).basicPolygon();
        area_polygons.push_back(area_poly);
      }
    }
  }
  return area_polygons;
}

BasicPolygons2d GoalSearcher::getNoStoppingAreaPolygons(const lanelet::ConstLanelets & lanes) const
{
  BasicPolygons2d area_polygons{};
  for (const auto & ll : lanes) {
    for (const auto & reg_elem : ll.regulatoryElementsAs<NoStoppingArea>()) {
      for (const auto & area : reg_elem->noStoppingAreas()) {
        const auto & area_poly = lanelet::utils::to2D(area).basicPolygon();
        area_polygons.push_back(area_poly);
      }
    }
  }
  return area_polygons;
}

GoalCandidate GoalSearcher::getClosetGoalCandidateAlongLanes(
  const GoalCandidates & goal_candidates,
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  const auto current_lanes = utils::getExtendedCurrentLanes(
    planner_data, parameters_.backward_goal_search_length, parameters_.forward_goal_search_length,
    /*forward_only_in_route*/ false);

  // Define a lambda function to compute the arc length for a given goal candidate.
  auto getGoalArcLength = [&current_lanes](const auto & candidate) {
    return lanelet::utils::getArcCoordinates(current_lanes, candidate.goal_pose).length;
  };

  // Find the closest goal candidate by comparing the arc lengths of each candidate.
  const auto closest_goal_candidate = std::min_element(
    goal_candidates.begin(), goal_candidates.end(),
    [&getGoalArcLength](const auto & a, const auto & b) {
      return getGoalArcLength(a) < getGoalArcLength(b);
    });

  if (closest_goal_candidate == goal_candidates.end()) {
    return {};  // return empty GoalCandidate in case no valid candidates are found.
  }

  return *closest_goal_candidate;
}

}  // namespace autoware::behavior_path_planner
