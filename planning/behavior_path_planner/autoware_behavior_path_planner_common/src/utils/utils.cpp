// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include "autoware/motion_utils/trajectory/path_with_lane_id.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/geometry/algorithms/is_valid.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace
{
double calcInterpolatedZ(
  const tier4_planning_msgs::msg::PathWithLaneId & input,
  const geometry_msgs::msg::Point target_pos, const size_t seg_idx)
{
  const double closest_to_target_dist = autoware::motion_utils::calcSignedArcLength(
    input.points, input.points.at(seg_idx).point.pose.position,
    target_pos);  // TODO(murooka) implement calcSignedArcLength(points, idx, point)
  const double seg_dist =
    autoware::motion_utils::calcSignedArcLength(input.points, seg_idx, seg_idx + 1);

  const double closest_z = input.points.at(seg_idx).point.pose.position.z;
  const double next_z = input.points.at(seg_idx + 1).point.pose.position.z;
  const double interpolated_z =
    std::abs(seg_dist) < 1e-6
      ? next_z
      : closest_z + (next_z - closest_z) * closest_to_target_dist / seg_dist;
  return interpolated_z;
}

double calcInterpolatedVelocity(
  const tier4_planning_msgs::msg::PathWithLaneId & input, const size_t seg_idx)
{
  const double seg_dist =
    autoware::motion_utils::calcSignedArcLength(input.points, seg_idx, seg_idx + 1);

  const double closest_vel = input.points.at(seg_idx).point.longitudinal_velocity_mps;
  const double next_vel = input.points.at(seg_idx + 1).point.longitudinal_velocity_mps;
  const double interpolated_vel = std::abs(seg_dist) < 1e-06 ? next_vel : closest_vel;
  return interpolated_vel;
}
}  // namespace

namespace autoware::behavior_path_planner::utils
{
using autoware::universe_utils::LineString2d;
using autoware::universe_utils::Point2d;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;
using geometry_msgs::msg::PoseWithCovarianceStamped;

std::optional<lanelet::Polygon3d> getPolygonByPoint(
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstPoint3d & point,
  const std::string & polygon_name)
{
  const auto polygons = route_handler->getLaneletMapPtr()->polygonLayer.findUsages(point);
  for (const auto & polygon : polygons) {
    const std::string type = polygon.attributeOr(lanelet::AttributeName::Type, "none");
    if (type == polygon_name) {
      // NOTE: If there are multiple polygons on a point, only the front one is used.
      return polygon;
    }
  }
  return std::nullopt;
}

double l2Norm(const Vector3 vector)
{
  return std::sqrt(std::pow(vector.x, 2) + std::pow(vector.y, 2) + std::pow(vector.z, 2));
}

bool checkCollisionBetweenPathFootprintsAndObjects(
  const autoware::universe_utils::LinearRing2d & local_vehicle_footprint,
  const PathWithLaneId & ego_path, const PredictedObjects & dynamic_objects, const double margin)
{
  for (const auto & p : ego_path.points) {
    if (checkCollisionBetweenFootprintAndObjects(
          local_vehicle_footprint, p.point.pose, dynamic_objects, margin)) {
      return true;
    }
  }
  return false;
}

bool checkCollisionBetweenFootprintAndObjects(
  const autoware::universe_utils::LinearRing2d & local_vehicle_footprint, const Pose & ego_pose,
  const PredictedObjects & dynamic_objects, const double margin)
{
  const auto vehicle_footprint =
    transformVector(local_vehicle_footprint, autoware::universe_utils::pose2transform(ego_pose));

  for (const auto & object : dynamic_objects.objects) {
    const auto obj_polygon = autoware::universe_utils::toPolygon2d(object);
    const double distance = boost::geometry::distance(obj_polygon, vehicle_footprint);
    if (distance < margin) return true;
  }
  return false;
}

double calcLateralDistanceFromEgoToObject(
  const Pose & ego_pose, const double vehicle_width, const PredictedObject & dynamic_object)
{
  double min_distance = std::numeric_limits<double>::max();
  const auto obj_polygon = autoware::universe_utils::toPolygon2d(dynamic_object);
  const auto vehicle_left_pose =
    autoware::universe_utils::calcOffsetPose(ego_pose, 0, vehicle_width / 2, 0);
  const auto vehicle_right_pose =
    autoware::universe_utils::calcOffsetPose(ego_pose, 0, -vehicle_width / 2, 0);

  for (const auto & p : obj_polygon.outer()) {
    const auto point = autoware::universe_utils::createPoint(p.x(), p.y(), 0.0);
    const double signed_distance_from_left =
      autoware::universe_utils::calcLateralDeviation(vehicle_left_pose, point);
    const double signed_distance_from_right =
      autoware::universe_utils::calcLateralDeviation(vehicle_right_pose, point);

    if (signed_distance_from_left < 0.0 && signed_distance_from_right > 0.0) {
      // point is between left and right
      return 0.0;
    }

    const double distance_from_ego =
      std::min(std::abs(signed_distance_from_left), std::abs(signed_distance_from_right));
    min_distance = std::min(min_distance, distance_from_ego);
  }
  return min_distance;
}

double calc_longitudinal_distance_from_ego_to_object(
  const Pose & ego_pose, const double base_link2front, const double base_link2rear,
  const PredictedObject & dynamic_object)
{
  double min_distance = std::numeric_limits<double>::max();
  const auto obj_polygon = autoware::universe_utils::toPolygon2d(dynamic_object);
  const auto vehicle_front_pose =
    autoware::universe_utils::calcOffsetPose(ego_pose, base_link2front, 0, 0);
  const auto vehicle_rear_pose =
    autoware::universe_utils::calcOffsetPose(ego_pose, base_link2rear, 0, 0);

  for (const auto & p : obj_polygon.outer()) {
    const auto point = autoware::universe_utils::createPoint(p.x(), p.y(), 0.0);

    // forward is positive
    const double signed_distance_from_front =
      autoware::universe_utils::calcLongitudinalDeviation(vehicle_front_pose, point);
    // backward is positive
    const double signed_distance_from_rear =
      -autoware::universe_utils::calcLongitudinalDeviation(vehicle_rear_pose, point);

    if (signed_distance_from_front < 0.0 && signed_distance_from_rear < 0.0) {
      // point is between front and rear
      return 0.0;
    }

    const double distance_from_ego =
      std::min(std::abs(signed_distance_from_front), std::abs(signed_distance_from_rear));
    min_distance = std::min(min_distance, distance_from_ego);
  }
  return min_distance;
}

double calcLongitudinalDistanceFromEgoToObjects(
  const Pose & ego_pose, double base_link2front, double base_link2rear,
  const PredictedObjects & dynamic_objects)
{
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & object : dynamic_objects.objects) {
    min_distance = std::min(
      min_distance, calc_longitudinal_distance_from_ego_to_object(
                      ego_pose, base_link2front, base_link2rear, object));
  }
  return min_distance;
}

std::optional<size_t> findIndexOutOfGoalSearchRange(
  const std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> & points, const Pose & goal,
  const int64_t goal_lane_id, const double max_dist = std::numeric_limits<double>::max())
{
  if (points.empty()) {
    return std::nullopt;
  }

  // find goal index
  size_t min_dist_index;
  {
    bool found = false;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < points.size(); ++i) {
      const auto & lane_ids = points.at(i).lane_ids;

      const double dist_to_goal =
        autoware::universe_utils::calcDistance2d(points.at(i).point.pose, goal);
      const bool is_goal_lane_id_in_point =
        std::find(lane_ids.begin(), lane_ids.end(), goal_lane_id) != lane_ids.end();
      if (dist_to_goal < max_dist && dist_to_goal < min_dist && is_goal_lane_id_in_point) {
        min_dist_index = i;
        min_dist = dist_to_goal;
        found = true;
      }
    }
    if (!found) {
      return std::nullopt;
    }
  }

  // find index out of goal search range
  size_t min_dist_out_of_range_index = min_dist_index;
  for (int i = min_dist_index; 0 <= i; --i) {
    const double dist = autoware::universe_utils::calcDistance2d(points.at(i).point, goal);
    min_dist_out_of_range_index = i;
    if (max_dist < dist) {
      break;
    }
  }

  return min_dist_out_of_range_index;
}

// goal does not have z
bool set_goal(
  const double search_radius_range, [[maybe_unused]] const double search_rad_range,
  const PathWithLaneId & input, const Pose & goal, const int64_t goal_lane_id,
  PathWithLaneId * output_ptr)
{
  try {
    if (input.points.empty()) {
      return false;
    }

    // calculate refined_goal with interpolation
    // NOTE: goal does not have valid z, that will be calculated by interpolation here
    PathPointWithLaneId refined_goal{};
    const size_t closest_seg_idx_for_goal =
      findNearestSegmentIndex(input.points, goal, 3.0, M_PI_4);
    refined_goal.point.pose = goal;
    refined_goal.point.pose.position.z =
      calcInterpolatedZ(input, goal.position, closest_seg_idx_for_goal);
    refined_goal.point.longitudinal_velocity_mps = 0.0;

    // calculate pre_refined_goal with interpolation
    // NOTE: z and velocity are filled
    PathPointWithLaneId pre_refined_goal{};
    constexpr double goal_to_pre_goal_distance = -1.0;
    pre_refined_goal.point.pose =
      autoware::universe_utils::calcOffsetPose(goal, goal_to_pre_goal_distance, 0.0, 0.0);
    const size_t closest_seg_idx_for_pre_goal =
      findNearestSegmentIndex(input.points, pre_refined_goal.point.pose, 3.0, M_PI_4);
    pre_refined_goal.point.pose.position.z =
      calcInterpolatedZ(input, pre_refined_goal.point.pose.position, closest_seg_idx_for_pre_goal);
    pre_refined_goal.point.longitudinal_velocity_mps =
      calcInterpolatedVelocity(input, closest_seg_idx_for_pre_goal);

    // find min_dist_out_of_circle_index whose distance to goal is longer than search_radius_range
    const auto min_dist_out_of_circle_index_opt =
      findIndexOutOfGoalSearchRange(input.points, goal, goal_lane_id, search_radius_range);
    if (!min_dist_out_of_circle_index_opt) {
      return false;
    }
    const size_t min_dist_out_of_circle_index = min_dist_out_of_circle_index_opt.value();

    // create output points
    output_ptr->points.reserve(output_ptr->points.size() + min_dist_out_of_circle_index + 3);
    for (size_t i = 0; i <= min_dist_out_of_circle_index; ++i) {
      output_ptr->points.push_back(input.points.at(i));
    }
    output_ptr->points.push_back(pre_refined_goal);
    output_ptr->points.push_back(refined_goal);

    {  // fill skipped lane ids
      // pre refined goal
      auto & pre_goal = output_ptr->points.at(output_ptr->points.size() - 2);
      for (size_t i = min_dist_out_of_circle_index + 1; i < input.points.size(); ++i) {
        for (const auto target_lane_id : input.points.at(i).lane_ids) {
          const bool is_lane_id_found =
            std::find(pre_goal.lane_ids.begin(), pre_goal.lane_ids.end(), target_lane_id) !=
            pre_goal.lane_ids.end();
          if (!is_lane_id_found) {
            pre_goal.lane_ids.push_back(target_lane_id);
          }
        }
      }

      // goal
      output_ptr->points.back().lane_ids = input.points.back().lane_ids;
    }

    output_ptr->left_bound = input.left_bound;
    output_ptr->right_bound = input.right_bound;
    return true;
  } catch (std::out_of_range & ex) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("utils"),
      "failed to set goal: " << ex.what());
    return false;
  }
}

const Pose refineGoal(const Pose & goal, const lanelet::ConstLanelet & goal_lanelet)
{
  // return goal;
  const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(goal.position);
  const double distance = boost::geometry::distance(
    goal_lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(lanelet_point).basicPoint());
  if (distance < std::numeric_limits<double>::epsilon()) {
    return goal;
  }

  const auto segment = lanelet::utils::getClosestSegment(
    lanelet::utils::to2D(lanelet_point), goal_lanelet.centerline());
  if (segment.empty()) {
    return goal;
  }

  Pose refined_goal;
  {
    // find position
    const auto p1 = segment.front().basicPoint();
    const auto p2 = segment.back().basicPoint();
    const auto direction_vector = (p2 - p1).normalized();
    const auto p1_to_goal = lanelet_point.basicPoint() - p1;
    const double s = direction_vector.dot(p1_to_goal);
    const auto refined_point = p1 + direction_vector * s;

    refined_goal.position.x = refined_point.x();
    refined_goal.position.y = refined_point.y();
    refined_goal.position.z = refined_point.z();

    // find orientation
    const double yaw = std::atan2(direction_vector.y(), direction_vector.x());
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, yaw);
    refined_goal.orientation = tf2::toMsg(tf_quat);
  }
  return refined_goal;
}

PathWithLaneId refinePathForGoal(
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const Pose & goal, const int64_t goal_lane_id)
{
  PathWithLaneId filtered_path = input;
  PathWithLaneId path_with_goal;
  filtered_path.points = autoware::motion_utils::removeOverlapPoints(filtered_path.points);

  // always set zero velocity at the end of path for safety
  if (!filtered_path.points.empty()) {
    filtered_path.points.back().point.longitudinal_velocity_mps = 0.0;
  }

  if (set_goal(
        search_radius_range, search_rad_range, filtered_path, goal, goal_lane_id,
        &path_with_goal)) {
    return path_with_goal;
  }
  return filtered_path;
}

bool isInLanelets(const Pose & pose, const lanelet::ConstLanelets & lanes)
{
  for (const auto & lane : lanes) {
    if (lanelet::utils::isInLanelet(pose, lane)) {
      return true;
    }
  }
  return false;
}

bool isInLaneletWithYawThreshold(
  const Pose & current_pose, const lanelet::ConstLanelet & lanelet, const double yaw_threshold,
  const double radius)
{
  const double pose_yaw = tf2::getYaw(current_pose.orientation);
  const double lanelet_angle = lanelet::utils::getLaneletAngle(lanelet, current_pose.position);
  const double angle_diff =
    std::abs(autoware::universe_utils::normalizeRadian(lanelet_angle - pose_yaw));

  return (angle_diff < std::abs(yaw_threshold)) &&
         lanelet::utils::isInLanelet(current_pose, lanelet, radius);
}

bool isEgoOutOfRoute(
  const Pose & self_pose, const lanelet::ConstLanelet & closest_road_lane,
  const std::optional<PoseWithUuidStamped> & modified_goal,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  const Pose & goal_pose = (modified_goal && modified_goal->uuid == route_handler->getRouteUuid())
                             ? modified_goal->pose
                             : route_handler->getGoalPose();

  lanelet::ConstLanelet goal_lane;
  const auto shoulder_goal_lanes = route_handler->getShoulderLaneletsAtPose(goal_pose);
  if (!shoulder_goal_lanes.empty()) goal_lane = shoulder_goal_lanes.front();
  const auto is_failed_getting_lanelet =
    shoulder_goal_lanes.empty() && !route_handler->getGoalLanelet(&goal_lane);
  if (is_failed_getting_lanelet) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("util"), "cannot find goal lanelet");
    return true;
  }

  // If ego vehicle is over goal on goal lane, return true
  const double yaw_threshold = autoware::universe_utils::deg2rad(90);
  if (
    closest_road_lane.id() == goal_lane.id() &&
    isInLaneletWithYawThreshold(self_pose, goal_lane, yaw_threshold)) {
    constexpr double buffer = 1.0;
    const auto ego_arc_coord = lanelet::utils::getArcCoordinates({goal_lane}, self_pose);
    const auto goal_arc_coord =
      lanelet::utils::getArcCoordinates({goal_lane}, route_handler->getGoalPose());
    if (ego_arc_coord.length > goal_arc_coord.length + buffer) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("util"), "ego pose is beyond goal");
      return true;
    }
    return false;
  }

  // If ego vehicle is out of the closest lanelet, return true
  // Check if ego vehicle is in shoulder lane
  const bool is_in_shoulder_lane = !route_handler->getShoulderLaneletsAtPose(self_pose).empty();
  // Check if ego vehicle is in road lane
  const bool is_in_road_lane = std::invoke([&]() {
    if (lanelet::utils::isInLanelet(self_pose, closest_road_lane)) {
      return true;
    }

    // check previous lanes for backward driving (e.g. pull out)
    const auto prev_lanes = route_handler->getPreviousLanelets(closest_road_lane);
    for (const auto & lane : prev_lanes) {
      if (lanelet::utils::isInLanelet(self_pose, lane)) {
        return true;
      }
    }

    return false;
  });

  return !is_in_shoulder_lane && !is_in_road_lane;
}

bool isEgoWithinOriginalLane(
  const lanelet::ConstLanelets & current_lanes, const Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param, const double outer_margin)
{
  const auto combined_lane = lanelet::utils::combineLaneletsShape(current_lanes);
  const auto lane_polygon = combined_lane.polygon2d().basicPolygon();
  return isEgoWithinOriginalLane(lane_polygon, current_pose, common_param, outer_margin);
}

bool isEgoWithinOriginalLane(
  const lanelet::BasicPolygon2d & lane_polygon, const Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param, const double outer_margin)
{
  const auto vehicle_poly = autoware::universe_utils::toFootprint(
    current_pose, common_param.base_link2front, common_param.base_link2rear,
    common_param.vehicle_width);

  // Check if the ego vehicle is entirely within the lane with a given outer margin.
  for (const auto & p : vehicle_poly.outer()) {
    // When the point is in the polygon, the distance is 0. When it is out of the polygon, return a
    // positive value.
    const auto dist = boost::geometry::distance(p, lane_polygon);
    if (dist > std::max(outer_margin, 0.0)) {
      return false;  // out of polygon
    }
  }

  return true;  // inside polygon
}

lanelet::ConstLanelets transformToLanelets(const DrivableLanes & drivable_lanes)
{
  lanelet::ConstLanelets lanes;

  const auto has_same_lane = [&](const auto & lane) {
    if (lanes.empty()) return false;
    const auto has_same = [&](const auto & ll) { return ll.id() == lane.id(); };
    return std::find_if(lanes.begin(), lanes.end(), has_same) != lanes.end();
  };

  lanes.push_back(drivable_lanes.right_lane);
  if (!has_same_lane(drivable_lanes.left_lane)) {
    lanes.push_back(drivable_lanes.left_lane);
  }

  for (const auto & ml : drivable_lanes.middle_lanes) {
    if (!has_same_lane(ml)) {
      lanes.push_back(ml);
    }
  }

  return lanes;
}

lanelet::ConstLanelets transformToLanelets(const std::vector<DrivableLanes> & drivable_lanes)
{
  lanelet::ConstLanelets lanes;

  for (const auto & drivable_lane : drivable_lanes) {
    const auto transformed_lane = transformToLanelets(drivable_lane);
    lanes.insert(lanes.end(), transformed_lane.begin(), transformed_lane.end());
  }

  return lanes;
}

std::optional<lanelet::ConstLanelet> getRightLanelet(
  const lanelet::ConstLanelet & current_lane, const lanelet::ConstLanelets & shoulder_lanes)
{
  for (const auto & shoulder_lane : shoulder_lanes) {
    if (shoulder_lane.leftBound().id() == current_lane.rightBound().id()) {
      return shoulder_lane;
    }
  }

  return {};
}

std::optional<lanelet::ConstLanelet> getLeftLanelet(
  const lanelet::ConstLanelet & current_lane, const lanelet::ConstLanelets & shoulder_lanes)
{
  for (const auto & shoulder_lane : shoulder_lanes) {
    if (shoulder_lane.rightBound().id() == current_lane.leftBound().id()) {
      return shoulder_lane;
    }
  }

  return {};
}

// generate drivable area by expanding path for freespace
double getDistanceToEndOfLane(const Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);
  const double lanelet_length = lanelet::utils::getLaneletLength3d(lanelets);
  return lanelet_length - arc_coordinates.length;
}

double getDistanceToNextIntersection(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);

  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, current_pose, &current_lanelet)) {
    return std::numeric_limits<double>::max();
  }

  double distance = 0;
  bool is_after_current_lanelet = false;
  for (const auto & llt : lanelets) {
    if (llt == current_lanelet) {
      is_after_current_lanelet = true;
    }
    if (is_after_current_lanelet && llt.hasAttribute("turn_direction")) {
      bool is_lane_change_yes = false;
      const auto right_line = llt.rightBound();
      if (right_line.hasAttribute(lanelet::AttributeNamesString::LaneChange)) {
        const auto attr = right_line.attribute(lanelet::AttributeNamesString::LaneChange);
        if (attr.value() == std::string("yes")) {
          is_lane_change_yes = true;
        }
      }
      const auto left_line = llt.leftBound();
      if (left_line.hasAttribute(lanelet::AttributeNamesString::LaneChange)) {
        const auto attr = left_line.attribute(lanelet::AttributeNamesString::LaneChange);
        if (attr.value() == std::string("yes")) {
          is_lane_change_yes = true;
        }
      }
      if (!is_lane_change_yes) {
        return distance - arc_coordinates.length;
      }
    }
    distance += lanelet::utils::getLaneletLength3d(llt);
  }

  return std::numeric_limits<double>::max();
}

double getDistanceToCrosswalk(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphContainer & overall_graphs)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);

  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, current_pose, &current_lanelet)) {
    return std::numeric_limits<double>::infinity();
  }

  double distance = 0;
  bool is_after_current_lanelet = false;
  for (const auto & llt : lanelets) {
    if (llt == current_lanelet) {
      is_after_current_lanelet = true;
    }

    if (is_after_current_lanelet) {
      const auto conflicting_crosswalks = overall_graphs.conflictingInGraph(llt, 1);
      if (!(conflicting_crosswalks.empty())) {
        // create centerline
        const lanelet::ConstLineString2d lanelet_centerline = llt.centerline2d();
        LineString2d centerline;
        centerline.reserve(lanelet_centerline.size());
        for (const auto & point : lanelet_centerline) {
          boost::geometry::append(centerline, Point2d(point.x(), point.y()));
        }

        // create crosswalk polygon and calculate distance
        double min_distance_to_crosswalk = std::numeric_limits<double>::infinity();
        for (const auto & crosswalk : conflicting_crosswalks) {
          lanelet::CompoundPolygon2d lanelet_crosswalk_polygon = crosswalk.polygon2d();
          Polygon2d polygon;
          polygon.outer().reserve(lanelet_crosswalk_polygon.size() + 1);
          for (const auto & point : lanelet_crosswalk_polygon) {
            polygon.outer().emplace_back(point.x(), point.y());
          }
          polygon.outer().push_back(polygon.outer().front());

          std::vector<Point2d> points_intersection;
          boost::geometry::intersection(centerline, polygon, points_intersection);

          for (const auto & point : points_intersection) {
            Pose pose_point;
            pose_point.position.x = point.x();
            pose_point.position.y = point.y();
            const lanelet::ArcCoordinates & arc_crosswalk =
              lanelet::utils::getArcCoordinates({llt}, pose_point);

            const double distance_to_crosswalk = arc_crosswalk.length;
            if (distance_to_crosswalk < min_distance_to_crosswalk) {
              min_distance_to_crosswalk = distance_to_crosswalk;
            }
          }
        }
        if (distance + min_distance_to_crosswalk > arc_coordinates.length) {
          return distance + min_distance_to_crosswalk - arc_coordinates.length;
        }
      }
    }
    distance += lanelet::utils::getLaneletLength3d(llt);
  }

  return std::numeric_limits<double>::infinity();
}

double getSignedDistance(
  const Pose & current_pose, const Pose & goal_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto arc_current = lanelet::utils::getArcCoordinates(lanelets, current_pose);
  const auto arc_goal = lanelet::utils::getArcCoordinates(lanelets, goal_pose);

  return arc_goal.length - arc_current.length;
}

PathPointWithLaneId insertStopPoint(const double length, PathWithLaneId & path)
{
  const size_t original_size = path.points.size();

  // insert stop point
  const auto insert_idx = autoware::motion_utils::insertStopPoint(length, path.points);
  if (!insert_idx) {
    return PathPointWithLaneId();
  }

  // check if a stop point is inserted
  if (path.points.size() == original_size) {
    return path.points.at(*insert_idx);
  }

  if (*insert_idx == 0 || *insert_idx == original_size - 1) {
    return path.points.at(*insert_idx);
  }

  // check lane ids of the inserted stop point
  path.points.at(*insert_idx).lane_ids = {};
  const auto & prev_lane_ids = path.points.at(*insert_idx - 1).lane_ids;
  const auto & next_lane_ids = path.points.at(*insert_idx + 1).lane_ids;

  for (const auto target_lane_id : prev_lane_ids) {
    if (
      std::find(next_lane_ids.begin(), next_lane_ids.end(), target_lane_id) !=
      next_lane_ids.end()) {
      path.points.at(*insert_idx).lane_ids.push_back(target_lane_id);
    }
  }

  // If there is no lane ids, we are going to insert prev lane ids
  if (path.points.at(*insert_idx).lane_ids.empty()) {
    path.points.at(*insert_idx).lane_ids = prev_lane_ids;
  }

  return path.points.at(*insert_idx);
}

double getSignedDistanceFromLaneBoundary(
  const lanelet::ConstLanelet & lanelet, const Point & position, bool left_side)
{
  const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(position);
  const auto & boundary_line_2d = left_side ? lanelet.leftBound2d() : lanelet.rightBound2d();
  const auto arc_coordinates = lanelet::geometry::toArcCoordinates(
    boundary_line_2d, lanelet::utils::to2D(lanelet_point).basicPoint());
  return arc_coordinates.distance;
}

double getSignedDistanceFromBoundary(
  const lanelet::ConstLanelets & lanelets, const Pose & pose, bool left_side)
{
  lanelet::ConstLanelet closest_lanelet;

  if (lanelet::utils::query::getClosestLanelet(lanelets, pose, &closest_lanelet)) {
    return getSignedDistanceFromLaneBoundary(closest_lanelet, pose.position, left_side);
  }

  RCLCPP_ERROR_STREAM(
    rclcpp::get_logger("behavior_path_planner").get_child("utils"), "closest lanelet not found.");

  return 0.0;
}

std::optional<double> getSignedDistanceFromBoundary(
  const lanelet::ConstLanelets & lanelets, const double vehicle_width, const double base_link2front,
  const double base_link2rear, const Pose & vehicle_pose, const bool left_side)
{
  // Depending on which side is selected, calculate the transformed coordinates of the front and
  // rear vehicle corners
  Point rear_corner_point, front_corner_point;
  if (left_side) {
    Point front_left, rear_left;
    rear_left.x = -base_link2rear;
    rear_left.y = vehicle_width / 2;
    front_left.x = base_link2front;
    front_left.y = vehicle_width / 2;
    rear_corner_point = autoware::universe_utils::transformPoint(rear_left, vehicle_pose);
    front_corner_point = autoware::universe_utils::transformPoint(front_left, vehicle_pose);
  } else {
    Point front_right, rear_right;
    rear_right.x = -base_link2rear;
    rear_right.y = -vehicle_width / 2;
    front_right.x = base_link2front;
    front_right.y = -vehicle_width / 2;
    rear_corner_point = autoware::universe_utils::transformPoint(rear_right, vehicle_pose);
    front_corner_point = autoware::universe_utils::transformPoint(front_right, vehicle_pose);
  }

  const auto combined_lane = lanelet::utils::combineLaneletsShape(lanelets);
  const auto & bound_line_2d = left_side ? lanelet::utils::to2D(combined_lane.leftBound3d())
                                         : lanelet::utils::to2D(combined_lane.rightBound3d());

  // Find the closest bound segment that contains the corner point in the X-direction
  // and calculate the lateral distance from that segment.
  const auto calcLateralDistanceFromBound =
    [&](const Point & vehicle_corner_point) -> std::optional<std::pair<double, size_t>> {
    Pose vehicle_corner_pose{};
    vehicle_corner_pose.position = vehicle_corner_point;
    vehicle_corner_pose.orientation = vehicle_pose.orientation;

    std::optional<std::pair<double, size_t>> lateral_distance_with_idx{};

    // Euclidean distance to find the closest segment containing the corner point.
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < bound_line_2d.size() - 1; i++) {
      const Point p1 = lanelet::utils::conversion::toGeomMsgPt(bound_line_2d[i]);
      const Point p2 = lanelet::utils::conversion::toGeomMsgPt(bound_line_2d[i + 1]);

      const Point inverse_p1 =
        autoware::universe_utils::inverseTransformPoint(p1, vehicle_corner_pose);
      const Point inverse_p2 =
        autoware::universe_utils::inverseTransformPoint(p2, vehicle_corner_pose);
      const double dx_p1 = inverse_p1.x;
      const double dx_p2 = inverse_p2.x;
      const double dy_p1 = inverse_p1.y;
      const double dy_p2 = inverse_p2.y;

      // Calculate the Euclidean distances between vehicle's corner and the current and next points.
      const double distance1 = autoware::universe_utils::calcDistance2d(p1, vehicle_corner_point);
      const double distance2 = autoware::universe_utils::calcDistance2d(p2, vehicle_corner_point);

      // If one of the bound points is behind and the other is in front of the vehicle corner point
      // and any of these points is closer than the current minimum distance,
      // then update minimum distance, lateral distance and the segment index.
      if (dx_p1 < 0 && dx_p2 > 0 && (distance1 < min_distance || distance2 < min_distance)) {
        min_distance = std::min(distance1, distance2);
        // Update lateral distance using the formula derived from similar triangles in the lateral
        // cross-section view.
        lateral_distance_with_idx =
          std::make_pair(-1.0 * (dy_p1 * dx_p2 + dy_p2 * -dx_p1) / (dx_p2 - dx_p1), i);
      }
    }
    if (lateral_distance_with_idx) {
      return lateral_distance_with_idx;
    }
    return std::nullopt;
  };

  // Calculate the lateral distance for both the rear and front corners of the vehicle.
  const std::optional<std::pair<double, size_t>> rear_lateral_distance_with_idx =
    calcLateralDistanceFromBound(rear_corner_point);
  const std::optional<std::pair<double, size_t>> front_lateral_distance_with_idx =
    calcLateralDistanceFromBound(front_corner_point);

  // If no closest bound segment was found for both corners, return an empty optional.
  if (!rear_lateral_distance_with_idx && !front_lateral_distance_with_idx) {
    return {};
  }
  // If only one of them found the closest bound, return the found lateral distance.
  if (!rear_lateral_distance_with_idx) {
    return front_lateral_distance_with_idx.value().first;
  } else if (!front_lateral_distance_with_idx) {
    return rear_lateral_distance_with_idx.value().first;
  }
  // If both corners found their closest bound, return the maximum (for left side) or the minimum
  // (for right side) lateral distance.
  double lateral_distance = left_side ? std::max(
                                          rear_lateral_distance_with_idx.value().first,
                                          front_lateral_distance_with_idx.value().first)
                                      : std::min(
                                          rear_lateral_distance_with_idx.value().first,
                                          front_lateral_distance_with_idx.value().first);

  // Iterate through all segments between the segments closest to the rear and front corners.
  // Update the lateral distance in case any of these inner segments are closer to the vehicle.
  for (size_t i = rear_lateral_distance_with_idx.value().second + 1;
       i < front_lateral_distance_with_idx.value().second; i++) {
    Pose bound_pose;
    bound_pose.position = lanelet::utils::conversion::toGeomMsgPt(bound_line_2d[i]);
    bound_pose.orientation = vehicle_pose.orientation;

    const Point inverse_rear_point =
      autoware::universe_utils::inverseTransformPoint(rear_corner_point, bound_pose);
    const Point inverse_front_point =
      autoware::universe_utils::inverseTransformPoint(front_corner_point, bound_pose);
    const double dx_rear = inverse_rear_point.x;
    const double dx_front = inverse_front_point.x;
    const double dy_rear = inverse_rear_point.y;
    const double dy_front = inverse_front_point.y;

    const double current_lateral_distance =
      (dy_rear * dx_front + dy_front * -dx_rear) / (dx_front - dx_rear);
    lateral_distance = left_side ? std::max(lateral_distance, current_lateral_distance)
                                 : std::min(lateral_distance, current_lateral_distance);
  }

  return lateral_distance;
}

double getArcLengthToTargetLanelet(
  const lanelet::ConstLanelets & lanelet_sequence, const lanelet::ConstLanelet & target_lane,
  const Pose & pose)
{
  const auto arc_pose = lanelet::utils::getArcCoordinates(lanelet_sequence, pose);

  const auto target_center_line = target_lane.centerline().basicLineString();

  const auto front_pose = to_geom_msg_pose(target_center_line.front(), target_lane);
  const auto back_pose = to_geom_msg_pose(target_center_line.back(), target_lane);

  const auto arc_front = lanelet::utils::getArcCoordinates(lanelet_sequence, front_pose);
  const auto arc_back = lanelet::utils::getArcCoordinates(lanelet_sequence, back_pose);

  return std::max(
    std::min(arc_front.length - arc_pose.length, arc_back.length - arc_pose.length), 0.0);
}

Polygon2d toPolygon2d(const lanelet::ConstLanelet & lanelet)
{
  Polygon2d polygon;
  for (const auto & p : lanelet.polygon2d().basicPolygon()) {
    polygon.outer().emplace_back(p.x(), p.y());
  }
  polygon.outer().push_back(polygon.outer().front());

  return autoware::universe_utils::isClockwise(polygon)
           ? polygon
           : autoware::universe_utils::inverseClockwise(polygon);
}

Polygon2d toPolygon2d(const lanelet::BasicPolygon2d & polygon)
{
  Polygon2d ret;
  for (const auto & p : polygon) {
    ret.outer().emplace_back(p.x(), p.y());
  }
  ret.outer().push_back(ret.outer().front());

  return autoware::universe_utils::isClockwise(ret)
           ? ret
           : autoware::universe_utils::inverseClockwise(ret);
}

// TODO(Horibe) There is a similar function in route_handler.
std::shared_ptr<PathWithLaneId> generateCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data)
{
  auto centerline_path = std::make_shared<PathWithLaneId>();

  const auto & p = planner_data->parameters;

  const auto & route_handler = planner_data->route_handler;
  const auto & pose = planner_data->self_odometry->pose.pose;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("utils"),
      "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe) What should be returned?
  }

  // For lanelet_sequence with desired length
  lanelet::ConstLanelets lanelet_sequence = route_handler->getLaneletSequence(
    current_lane, pose, p.backward_path_length, p.forward_path_length);

  std::vector<DrivableLanes> drivable_lanes(lanelet_sequence.size());
  for (size_t i = 0; i < lanelet_sequence.size(); ++i) {
    drivable_lanes.at(i).left_lane = lanelet_sequence.at(i);
    drivable_lanes.at(i).right_lane = lanelet_sequence.at(i);
  }

  *centerline_path = getCenterLinePath(
    *route_handler, lanelet_sequence, pose, p.backward_path_length, p.forward_path_length, p);

  centerline_path->header = route_handler->getRouteHeader();

  return centerline_path;
}

PathWithLaneId getCenterLinePathFromLanelet(
  const lanelet::ConstLanelet & lanelet, const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto & route_handler = planner_data->route_handler;
  const auto & current_pose = planner_data->self_odometry->pose.pose;
  const auto & p = planner_data->parameters;

  const auto reference_lanes = route_handler->getLaneletSequence(
    lanelet, current_pose, p.backward_path_length, p.forward_path_length);

  return getCenterLinePath(
    *route_handler, reference_lanes, current_pose, p.backward_path_length, p.forward_path_length,
    p);
}

PathWithLaneId getCenterLinePath(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & lanelet_sequence,
  const Pose & pose, const double backward_path_length, const double forward_path_length,
  const BehaviorPathPlannerParameters & parameter)
{
  PathWithLaneId reference_path;

  if (lanelet_sequence.empty()) {
    return reference_path;
  }

  const auto arc_coordinates = lanelet::utils::getArcCoordinates(lanelet_sequence, pose);
  const double s = arc_coordinates.length;
  const double s_backward = std::max(0., s - backward_path_length);
  double s_forward = s + forward_path_length;

  if (route_handler.isDeadEndLanelet(lanelet_sequence.back())) {
    const auto lane_length = lanelet::utils::getLaneletLength2d(lanelet_sequence);
    s_forward = std::clamp(s_forward, 0.0, lane_length);
  }

  if (route_handler.isInGoalRouteSection(lanelet_sequence.back())) {
    const auto goal_arc_coordinates =
      lanelet::utils::getArcCoordinates(lanelet_sequence, route_handler.getGoalPose());
    s_forward = std::clamp(s_forward, 0.0, goal_arc_coordinates.length);
  }

  const auto raw_path_with_lane_id =
    route_handler.getCenterLinePath(lanelet_sequence, s_backward, s_forward, true);
  auto resampled_path_with_lane_id = autoware::motion_utils::resamplePath(
    raw_path_with_lane_id, parameter.input_path_interval, parameter.enable_akima_spline_first);

  // convert centerline, which we consider as CoG center,  to rear wheel center
  if (parameter.enable_cog_on_centerline) {
    const double rear_to_cog = parameter.vehicle_length / 2 - parameter.rear_overhang;
    return autoware::motion_utils::convertToRearWheelCenter(
      resampled_path_with_lane_id, rear_to_cog);
  }

  return resampled_path_with_lane_id;
}

std::uint8_t getHighestProbLabel(const std::vector<ObjectClassification> & classification)
{
  std::uint8_t label = ObjectClassification::UNKNOWN;
  float highest_prob = 0.0;
  for (const auto & _class : classification) {
    if (highest_prob < _class.probability) {
      highest_prob = _class.probability;
      label = _class.label;
    }
  }
  return label;
}

lanelet::ConstLanelets getCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_path_length,
  const double forward_path_length)
{
  const auto & route_handler = planner_data->route_handler;
  const auto current_pose = planner_data->self_odometry->pose.pose;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    auto clock{rclcpp::Clock{RCL_ROS_TIME}};
    RCLCPP_ERROR_STREAM_THROTTLE(
      rclcpp::get_logger("behavior_path_planner").get_child("utils"), clock, 1000,
      "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe) what should be returned?
  }

  // For current_lanes with desired length
  return route_handler->getLaneletSequence(
    current_lane, current_pose, backward_path_length, forward_path_length);
}

lanelet::ConstLanelets getCurrentLanes(const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto & common_parameters = planner_data->parameters;
  return getCurrentLanes(
    planner_data, common_parameters.backward_path_length, common_parameters.forward_path_length);
}

lanelet::ConstLanelets getCurrentLanesFromPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data)
{
  if (path.points.empty() || !planner_data) {
    return {};
  }

  const auto & route_handler = planner_data->route_handler;
  const auto & self_odometry = planner_data->self_odometry;

  if (!route_handler || !self_odometry) {
    return {};
  }

  const auto & current_pose = self_odometry->pose.pose;

  const auto & p = planner_data->parameters;

  std::set<lanelet::Id> lane_ids;
  for (const auto & point : path.points) {
    for (const auto & id : point.lane_ids) {
      lane_ids.insert(id);
    }
  }

  lanelet::ConstLanelets reference_lanes{};
  for (const auto & id : lane_ids) {
    reference_lanes.push_back(planner_data->route_handler->getLaneletsFromId(id));
  }

  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(reference_lanes, current_pose, &current_lane);
  auto current_lanes = route_handler->getLaneletSequence(
    current_lane, current_pose, p.backward_path_length, p.forward_path_length);

  // Extend the 'current_lanes' with previous lanes until it contains 'front_lane_ids'
  // if the extended prior lanes is in same lane sequence with current lanes
  const auto front_lane_ids = path.points.front().lane_ids;
  auto have_front_lanes = [front_lane_ids](const auto & lanes) {
    return std::any_of(lanes.begin(), lanes.end(), [&](const auto & lane) {
      return std::find(front_lane_ids.begin(), front_lane_ids.end(), lane.id()) !=
             front_lane_ids.end();
    });
  };
  auto extended_lanes = current_lanes;
  while (rclcpp::ok()) {
    const size_t pre_extension_size = extended_lanes.size();  // Get existing size before extension
    extended_lanes = extendPrevLane(route_handler, extended_lanes, true);
    if (extended_lanes.size() == pre_extension_size) break;
    if (have_front_lanes(extended_lanes)) {
      current_lanes = extended_lanes;
      break;
    }
  }

  return current_lanes;
}

lanelet::ConstLanelets extendNextLane(
  const std::shared_ptr<RouteHandler> route_handler, const lanelet::ConstLanelets & lanes,
  const bool only_in_route)
{
  if (lanes.empty()) return lanes;

  const auto next_lanes = route_handler->getNextLanelets(lanes.back());
  if (next_lanes.empty()) return lanes;

  // Add next lane
  auto extended_lanes = lanes;
  std::optional<lanelet::ConstLanelet> target_next_lane;
  for (const auto & next_lane : next_lanes) {
    // skip overlapping lanes
    if (next_lane.id() == lanes.front().id()) continue;

    // if route lane, set target and break
    if (route_handler->isRouteLanelet(next_lane)) {
      target_next_lane = next_lane;
      break;
    }
    if (!only_in_route && !target_next_lane) target_next_lane = next_lane;
  }
  if (target_next_lane) {
    extended_lanes.push_back(*target_next_lane);
  }
  return extended_lanes;
}

lanelet::ConstLanelets extendPrevLane(
  const std::shared_ptr<RouteHandler> route_handler, const lanelet::ConstLanelets & lanes,
  const bool only_in_route)
{
  if (lanes.empty()) return lanes;

  const auto prev_lanes = route_handler->getPreviousLanelets(lanes.front());
  if (prev_lanes.empty()) return lanes;

  // Add previous lane
  auto extended_lanes = lanes;
  std::optional<lanelet::ConstLanelet> target_prev_lane;
  for (const auto & prev_lane : prev_lanes) {
    // skip overlapping lanes
    if (prev_lane.id() == lanes.back().id()) continue;

    // if route lane, set target and break
    if (route_handler->isRouteLanelet(prev_lane)) {
      target_prev_lane = prev_lane;
      break;
    }
    if (!only_in_route && !target_prev_lane) target_prev_lane = prev_lane;
  }
  if (target_prev_lane) {
    extended_lanes.insert(extended_lanes.begin(), *target_prev_lane);
  }
  return extended_lanes;
}

lanelet::ConstLanelets extendLanes(
  const std::shared_ptr<RouteHandler> route_handler, const lanelet::ConstLanelets & lanes)
{
  auto extended_lanes = extendNextLane(route_handler, lanes);
  extended_lanes = extendPrevLane(route_handler, extended_lanes);

  return extended_lanes;
}

lanelet::ConstLanelets getExtendedCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_length,
  const double forward_length, const bool forward_only_in_route)
{
  auto lanes = getCurrentLanes(planner_data);
  if (lanes.empty()) return lanes;
  const auto start_lane = lanes.front();

  double forward_length_sum = 0.0;
  double backward_length_sum = 0.0;

  while (backward_length_sum < backward_length) {
    auto extended_lanes = extendPrevLane(planner_data->route_handler, lanes);
    if (extended_lanes.empty()) {
      return lanes;
    }
    // loop check
    // if current map lanes is looping and has a very large value for backward_length,
    // the extending process will not finish.
    if (extended_lanes.front().id() == start_lane.id()) {
      return lanes;
    }

    if (extended_lanes.size() > lanes.size()) {
      backward_length_sum += lanelet::utils::getLaneletLength2d(extended_lanes.front());
    } else {
      break;  // no more previous lanes to add
    }
    lanes = extended_lanes;
  }

  while (forward_length_sum < forward_length) {
    auto extended_lanes = extendNextLane(planner_data->route_handler, lanes);
    if (extended_lanes.empty()) {
      return lanes;
    }
    // loop check
    // if current map lanes is looping and has a very large value for forward_length,
    // the extending process will not finish.
    if (extended_lanes.back().id() == start_lane.id()) {
      return lanes;
    }

    if (extended_lanes.size() > lanes.size()) {
      forward_length_sum += lanelet::utils::getLaneletLength2d(extended_lanes.back());
    } else {
      break;  // no more next lanes to add
    }

    // stop extending when the lane outside of the route is reached
    // if forward_length is a very large value, set it to true,
    // as it may continue to extend forever.
    if (forward_only_in_route) {
      if (!planner_data->route_handler->isRouteLanelet(extended_lanes.back())) {
        return lanes;
      }
    }

    lanes = extended_lanes;
  }

  return lanes;
}

lanelet::ConstLanelets getExtendedCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data)
{
  return extendLanes(planner_data->route_handler, getCurrentLanes(planner_data));
}

lanelet::ConstLanelets getExtendedCurrentLanesFromPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data,
  const double backward_length, const double forward_length, const bool forward_only_in_route)
{
  auto lanes = getCurrentLanesFromPath(path, planner_data);

  if (lanes.empty()) return lanes;
  const auto start_lane = lanes.front();
  double forward_length_sum = 0.0;
  double backward_length_sum = 0.0;

  while (backward_length_sum < backward_length) {
    auto extended_lanes = extendPrevLane(planner_data->route_handler, lanes);
    if (extended_lanes.empty()) {
      return lanes;
    }
    // loop check
    // if current map lanes is looping and has a very large value for backward_length,
    // the extending process will not finish.
    if (extended_lanes.front().id() == start_lane.id()) {
      return lanes;
    }

    if (extended_lanes.size() > lanes.size()) {
      backward_length_sum += lanelet::utils::getLaneletLength2d(extended_lanes.front());
    } else {
      break;  // no more previous lanes to add
    }
    lanes = extended_lanes;
  }

  while (forward_length_sum < forward_length) {
    auto extended_lanes = extendNextLane(planner_data->route_handler, lanes);
    if (extended_lanes.empty()) {
      return lanes;
    }
    // loop check
    // if current map lanes is looping and has a very large value for forward_length,
    // the extending process will not finish.
    if (extended_lanes.back().id() == start_lane.id()) {
      return lanes;
    }

    if (extended_lanes.size() > lanes.size()) {
      forward_length_sum += lanelet::utils::getLaneletLength2d(extended_lanes.back());
    } else {
      break;  // no more next lanes to add
    }

    // stop extending when the lane outside of the route is reached
    // if forward_length is a very large value, set it to true,
    // as it may continue to extend forever.
    if (forward_only_in_route) {
      if (!planner_data->route_handler->isRouteLanelet(extended_lanes.back())) {
        return lanes;
      }
    }

    lanes = extended_lanes;
  }

  return lanes;
}

std::vector<lanelet::ConstLanelets> getPrecedingLanelets(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & current_pose, const double backward_length)
{
  if (target_lanes.empty()) {
    return {};
  }

  const auto arc_length = lanelet::utils::getArcCoordinates(target_lanes, current_pose);

  if (arc_length.length >= backward_length) {
    return {};
  }

  const auto & front_lane = target_lanes.front();
  return route_handler.getPrecedingLaneletSequence(
    front_lane, std::abs(backward_length - arc_length.length), {front_lane});
}

lanelet::ConstLanelets getBackwardLanelets(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & current_pose, const double backward_length)
{
  const auto preceding_lanes =
    getPrecedingLanelets(route_handler, target_lanes, current_pose, backward_length);
  const auto calc_sum = [](size_t sum, const auto & lanes) { return sum + lanes.size(); };
  const auto num_of_lanes =
    std::accumulate(preceding_lanes.begin(), preceding_lanes.end(), 0u, calc_sum);

  lanelet::ConstLanelets backward_lanes{};
  backward_lanes.reserve(num_of_lanes);

  for (const auto & lanes : preceding_lanes) {
    backward_lanes.insert(backward_lanes.end(), lanes.begin(), lanes.end());
  }

  return backward_lanes;
}

lanelet::ConstLanelets calcLaneAroundPose(
  const std::shared_ptr<RouteHandler> & route_handler, const Pose & pose,
  const double forward_length, const double backward_length, const double dist_threshold,
  const double yaw_threshold)
{
  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithConstrainsWithinRoute(
        pose, &current_lane, dist_threshold, yaw_threshold)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
      "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe)
  }

  // For current_lanes with desired length
  lanelet::ConstLanelets current_lanes =
    route_handler->getLaneletSequence(current_lane, pose, backward_length, forward_length);

  return current_lanes;
}

bool checkPathRelativeAngle(const PathWithLaneId & path, const double angle_threshold)
{
  // We need at least three points to compute relative angle
  constexpr size_t relative_angle_points_num = 3;
  if (path.points.size() < relative_angle_points_num) {
    return true;
  }

  for (size_t p1_id = 0; p1_id <= path.points.size() - relative_angle_points_num; ++p1_id) {
    // Get Point1
    const auto & p1 = path.points.at(p1_id).point.pose.position;

    // Get Point2
    const auto & p2 = path.points.at(p1_id + 1).point.pose.position;

    // Get Point3
    const auto & p3 = path.points.at(p1_id + 2).point.pose.position;

    // ignore invert driving direction
    if (
      path.points.at(p1_id).point.longitudinal_velocity_mps < 0 ||
      path.points.at(p1_id + 1).point.longitudinal_velocity_mps < 0 ||
      path.points.at(p1_id + 2).point.longitudinal_velocity_mps < 0) {
      continue;
    }

    // convert to p1 coordinate
    const double x3 = p3.x - p1.x;
    const double x2 = p2.x - p1.x;
    const double y3 = p3.y - p1.y;
    const double y2 = p2.y - p1.y;

    // calculate relative angle of vector p3 based on p1p2 vector
    const double th = std::atan2(y2, x2);
    const double th2 =
      std::atan2(-x3 * std::sin(th) + y3 * std::cos(th), x3 * std::cos(th) + y3 * std::sin(th));
    if (std::abs(th2) > angle_threshold) {
      // invalid angle
      return false;
    }
  }

  return true;
}

lanelet::ConstLanelets getLaneletsFromPath(
  const PathWithLaneId & path, const std::shared_ptr<RouteHandler> & route_handler)
{
  std::vector<int64_t> unique_lanelet_ids;
  for (const auto & p : path.points) {
    const auto & lane_ids = p.lane_ids;
    for (const auto & lane_id : lane_ids) {
      if (
        std::find(unique_lanelet_ids.begin(), unique_lanelet_ids.end(), lane_id) ==
        unique_lanelet_ids.end()) {
        unique_lanelet_ids.push_back(lane_id);
      }
    }
  }

  lanelet::ConstLanelets lanelets;
  for (const auto & lane_id : unique_lanelet_ids) {
    lanelets.push_back(route_handler->getLaneletsFromId(lane_id));
  }

  return lanelets;
}

std::string convertToSnakeCase(const std::string & input_str)
{
  std::string output_str = std::string{static_cast<char>(std::tolower(input_str.at(0)))};
  for (size_t i = 1; i < input_str.length(); ++i) {
    const auto input_chr = input_str.at(i);
    if (std::isupper(input_chr)) {
      output_str += "_" + std::string{static_cast<char>(std::tolower(input_chr))};
    } else {
      output_str += input_chr;
    }
  }
  return output_str;
}

bool isAllowedGoalModification(const std::shared_ptr<RouteHandler> & route_handler)
{
  return route_handler->isAllowedGoalModification() || checkOriginalGoalIsInShoulder(route_handler);
}

bool checkOriginalGoalIsInShoulder(const std::shared_ptr<RouteHandler> & route_handler)
{
  const Pose & goal_pose = route_handler->getOriginalGoalPose();
  return !route_handler->getShoulderLaneletsAtPose(goal_pose).empty();
}
}  // namespace autoware::behavior_path_planner::utils
