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

#include "autoware/behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp"

#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/interpolation/linear_interpolation.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"
#include "autoware/universe_utils/ros/uuid_helper.hpp"

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/overlaps.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/strategies/strategies.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::utils::path_safety_checker
{

namespace bg = boost::geometry;

using autoware::motion_utils::calcLongitudinalOffsetPoint;
using autoware::motion_utils::calcLongitudinalOffsetToSegment;
using autoware::motion_utils::findNearestIndex;
using autoware::motion_utils::findNearestSegmentIndex;
using autoware::universe_utils::calcDistance2d;

void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  bg::append(polygon.outer(), point);
}

bool isTargetObjectOncoming(
  const geometry_msgs::msg::Pose & vehicle_pose, const geometry_msgs::msg::Pose & object_pose,
  const double angle_threshold)
{
  return std::abs(calcYawDeviation(vehicle_pose, object_pose)) > angle_threshold;
}

bool isTargetObjectFront(
  const geometry_msgs::msg::Pose & ego_pose, const Polygon2d & obj_polygon,
  const double base_to_front)
{
  const auto ego_offset_pose =
    autoware::universe_utils::calcOffsetPose(ego_pose, base_to_front, 0.0, 0.0);

  // check all edges in the polygon
  const auto & obj_polygon_outer = obj_polygon.outer();
  for (const auto & obj_edge : obj_polygon_outer) {
    const auto obj_point = autoware::universe_utils::createPoint(obj_edge.x(), obj_edge.y(), 0.0);
    if (autoware::universe_utils::calcLongitudinalDeviation(ego_offset_pose, obj_point) > 0.0) {
      return true;
    }
  }

  return false;
}

Polygon2d createExtendedPolygon(
  const Pose & base_link_pose, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double lon_length, const double lat_margin, const bool is_stopped_obj,
  CollisionCheckDebug & debug)
{
  const double & base_to_front = vehicle_info.max_longitudinal_offset_m;
  const double & width = vehicle_info.vehicle_width_m;
  const double & base_to_rear = vehicle_info.rear_overhang_m;

  // if stationary object, extend forward and backward by the half of lon length
  const double forward_lon_offset = base_to_front + (is_stopped_obj ? lon_length / 2 : lon_length);
  const double backward_lon_offset =
    -base_to_rear - (is_stopped_obj ? lon_length / 2 : 0);  // minus value
  const double lat_offset = width / 2.0 + lat_margin;

  {
    debug.forward_lon_offset = forward_lon_offset;
    debug.backward_lon_offset = backward_lon_offset;
    debug.lat_offset = lat_offset;
  }

  const auto p1 =
    autoware::universe_utils::calcOffsetPose(base_link_pose, forward_lon_offset, lat_offset, 0.0);
  const auto p2 =
    autoware::universe_utils::calcOffsetPose(base_link_pose, forward_lon_offset, -lat_offset, 0.0);
  const auto p3 =
    autoware::universe_utils::calcOffsetPose(base_link_pose, backward_lon_offset, -lat_offset, 0.0);
  const auto p4 =
    autoware::universe_utils::calcOffsetPose(base_link_pose, backward_lon_offset, lat_offset, 0.0);

  Polygon2d polygon;
  polygon.outer().reserve(5);
  appendPointToPolygon(polygon, p1.position);
  appendPointToPolygon(polygon, p2.position);
  appendPointToPolygon(polygon, p3.position);
  appendPointToPolygon(polygon, p4.position);
  appendPointToPolygon(polygon, p1.position);
  return autoware::universe_utils::isClockwise(polygon)
           ? polygon
           : autoware::universe_utils::inverseClockwise(polygon);
}

Polygon2d createExtendedPolygon(
  const PoseWithVelocityAndPolygonStamped & obj_pose_with_poly, const double lon_length,
  const double lat_margin, const bool is_stopped_obj, CollisionCheckDebug & debug)
{
  const auto & obj_polygon = obj_pose_with_poly.poly;
  if (obj_polygon.outer().empty()) {
    return obj_polygon;
  }
  const auto & obj_pose = obj_pose_with_poly.pose;

  double max_x = std::numeric_limits<double>::lowest();
  double min_x = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  const auto obj_polygon_outer = obj_polygon.outer();
  for (const auto & polygon_p : obj_polygon_outer) {
    const auto obj_p = autoware::universe_utils::createPoint(polygon_p.x(), polygon_p.y(), 0.0);
    const auto transformed_p = autoware::universe_utils::inverseTransformPoint(obj_p, obj_pose);

    max_x = std::max(transformed_p.x, max_x);
    min_x = std::min(transformed_p.x, min_x);
    max_y = std::max(transformed_p.y, max_y);
    min_y = std::min(transformed_p.y, min_y);
  }

  // if stationary object, extend forward and backward by the half of lon length
  const double forward_lon_offset = max_x + (is_stopped_obj ? lon_length / 2 : lon_length);
  const double backward_lon_offset = min_x - (is_stopped_obj ? lon_length / 2 : 0);  // minus value

  const double left_lat_offset = max_y + lat_margin;
  const double right_lat_offset = min_y - lat_margin;

  {
    debug.forward_lon_offset = forward_lon_offset;
    debug.backward_lon_offset = backward_lon_offset;
    debug.lat_offset = std::max(std::abs(left_lat_offset), std::abs(right_lat_offset));
  }

  const auto p1 =
    autoware::universe_utils::calcOffsetPose(obj_pose, forward_lon_offset, left_lat_offset, 0.0);
  const auto p2 =
    autoware::universe_utils::calcOffsetPose(obj_pose, forward_lon_offset, right_lat_offset, 0.0);
  const auto p3 =
    autoware::universe_utils::calcOffsetPose(obj_pose, backward_lon_offset, right_lat_offset, 0.0);
  const auto p4 =
    autoware::universe_utils::calcOffsetPose(obj_pose, backward_lon_offset, left_lat_offset, 0.0);

  Polygon2d polygon;
  polygon.outer().reserve(5);
  appendPointToPolygon(polygon, p1.position);
  appendPointToPolygon(polygon, p2.position);
  appendPointToPolygon(polygon, p3.position);
  appendPointToPolygon(polygon, p4.position);
  appendPointToPolygon(polygon, p1.position);
  return autoware::universe_utils::isClockwise(polygon)
           ? polygon
           : autoware::universe_utils::inverseClockwise(polygon);
}

Polygon2d create_extended_polygon_along_path(
  const PathWithLaneId & planned_path, const Pose & base_link_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double lon_length,
  const double lat_margin, const bool is_stopped_obj, CollisionCheckDebug & debug)
{
  const double & base_to_front = vehicle_info.max_longitudinal_offset_m;
  const double & width = vehicle_info.vehicle_width_m;
  const double & base_to_rear = vehicle_info.rear_overhang_m;

  // if stationary object, extend forward and backward by the half of lon length
  const double forward_lon_offset = base_to_front + (is_stopped_obj ? lon_length / 2 : lon_length);
  const double backward_lon_offset =
    -base_to_rear - (is_stopped_obj ? lon_length / 2 : 0);  // minus value
  const double lat_offset = width / 2.0 + lat_margin;

  {
    debug.forward_lon_offset = forward_lon_offset;
    debug.backward_lon_offset = backward_lon_offset;
    debug.lat_offset = lat_offset;
  }

  const auto lon_offset_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    planned_path.points, base_link_pose.position, lon_length);
  if (!lon_offset_pose.has_value()) {
    return createExtendedPolygon(
      base_link_pose, vehicle_info, lon_length, lat_margin, is_stopped_obj, debug);
  }

  const size_t start_idx =
    autoware::motion_utils::findNearestSegmentIndex(planned_path.points, base_link_pose.position);
  const size_t end_idx = autoware::motion_utils::findNearestSegmentIndex(
    planned_path.points, lon_offset_pose.value().position);

  Polygon2d polygon;

  {
    const auto p_offset = autoware::universe_utils::calcOffsetPose(
      base_link_pose, backward_lon_offset, lat_offset, 0.0);
    appendPointToPolygon(polygon, p_offset.position);
  }

  for (size_t i = start_idx + 1; i < end_idx + 1; ++i) {
    const auto p = autoware::universe_utils::getPose(planned_path.points.at(i));
    const auto p_offset = autoware::universe_utils::calcOffsetPose(p, 0.0, lat_offset, 0.0);
    appendPointToPolygon(polygon, p_offset.position);
  }

  {
    const auto p_offset = autoware::universe_utils::calcOffsetPose(
      lon_offset_pose.value(), base_to_front, lat_offset, 0.0);
    appendPointToPolygon(polygon, p_offset.position);
  }

  {
    const auto p_offset = autoware::universe_utils::calcOffsetPose(
      lon_offset_pose.value(), base_to_front, -lat_offset, 0.0);
    appendPointToPolygon(polygon, p_offset.position);
  }

  for (size_t i = end_idx; i > start_idx; --i) {
    const auto p = autoware::universe_utils::getPose(planned_path.points.at(i));
    const auto p_offset = autoware::universe_utils::calcOffsetPose(p, 0.0, -lat_offset, 0.0);
    appendPointToPolygon(polygon, p_offset.position);
  }

  {
    const auto p_offset = autoware::universe_utils::calcOffsetPose(
      base_link_pose, backward_lon_offset, -lat_offset, 0.0);
    appendPointToPolygon(polygon, p_offset.position);
  }

  {
    const auto p_offset = autoware::universe_utils::calcOffsetPose(
      base_link_pose, backward_lon_offset, lat_offset, 0.0);
    appendPointToPolygon(polygon, p_offset.position);
  }

  return autoware::universe_utils::isClockwise(polygon)
           ? polygon
           : autoware::universe_utils::inverseClockwise(polygon);
}

std::vector<Polygon2d> createExtendedPolygonsFromPoseWithVelocityStamped(
  const std::vector<PoseWithVelocityStamped> & predicted_path, const VehicleInfo & vehicle_info,
  const double forward_margin, const double backward_margin, const double lat_margin)
{
  std::vector<Polygon2d> polygons{};
  polygons.reserve(predicted_path.size());

  for (const auto & elem : predicted_path) {
    const auto & pose = elem.pose;
    const double base_to_front = vehicle_info.max_longitudinal_offset_m + forward_margin;
    const double base_to_rear = vehicle_info.rear_overhang_m + backward_margin;
    const double width = vehicle_info.vehicle_width_m + lat_margin * 2;

    const auto polygon =
      autoware::universe_utils::toFootprint(pose, base_to_front, base_to_rear, width);
    polygons.push_back(polygon);
  }

  return polygons;
}

PredictedPath convertToPredictedPath(
  const std::vector<PoseWithVelocityStamped> & path, const double time_resolution)
{
  PredictedPath predicted_path;
  predicted_path.time_step = rclcpp::Duration::from_seconds(time_resolution);
  predicted_path.path.resize(path.size());

  for (size_t i = 0; i < path.size(); ++i) {
    predicted_path.path.at(i) = path.at(i).pose;
  }
  return predicted_path;
}

double calcRssDistance(
  const double front_object_velocity, const double rear_object_velocity,
  const RSSparams & rss_params)
{
  const auto stoppingDistance = [](const auto vehicle_velocity, const auto vehicle_accel) {
    // compensate if user accidentally set the deceleration to some positive value
    const auto deceleration = (vehicle_accel < -1e-3) ? vehicle_accel : -1.0;
    return -std::pow(vehicle_velocity, 2) / (2.0 * deceleration);
  };

  const double & reaction_time =
    rss_params.rear_vehicle_reaction_time + rss_params.rear_vehicle_safety_time_margin;

  const double front_object_stop_length =
    stoppingDistance(front_object_velocity, rss_params.front_vehicle_deceleration);
  const double rear_object_stop_length =
    rear_object_velocity * reaction_time +
    stoppingDistance(rear_object_velocity, rss_params.rear_vehicle_deceleration);
  return rear_object_stop_length - front_object_stop_length;
}

double calc_minimum_longitudinal_length(
  const double front_object_velocity, const double rear_object_velocity,
  const RSSparams & rss_params)
{
  const double & lon_threshold = rss_params.longitudinal_distance_min_threshold;
  const auto max_vel = std::max(front_object_velocity, rear_object_velocity);
  return rss_params.longitudinal_velocity_delta_time * std::abs(max_vel) + lon_threshold;
}

std::optional<PoseWithVelocityStamped> calc_interpolated_pose_with_velocity(
  const std::vector<PoseWithVelocityStamped> & path, const double relative_time)
{
  // Check if relative time is in the valid range
  if (path.empty() || relative_time < 0.0) {
    return std::nullopt;
  }

  constexpr double epsilon = 1e-6;
  for (size_t path_idx = 1; path_idx < path.size(); ++path_idx) {
    const auto & pt = path.at(path_idx);
    const auto & prev_pt = path.at(path_idx - 1);
    if (relative_time < pt.time + epsilon) {
      const double offset = relative_time - prev_pt.time;
      const double time_step = pt.time - prev_pt.time;
      const double ratio = std::clamp(offset / time_step, 0.0, 1.0);
      const auto interpolated_pose =
        autoware::universe_utils::calcInterpolatedPose(prev_pt.pose, pt.pose, ratio, false);
      const double interpolated_velocity =
        autoware::interpolation::lerp(prev_pt.velocity, pt.velocity, ratio);
      return PoseWithVelocityStamped{relative_time, interpolated_pose, interpolated_velocity};
    }
  }

  return std::nullopt;
}

std::optional<PoseWithVelocityAndPolygonStamped>
get_interpolated_pose_with_velocity_and_polygon_stamped(
  const std::vector<PoseWithVelocityStamped> & pred_path, const double current_time,
  const VehicleInfo & ego_info)
{
  const auto interpolation_result = calc_interpolated_pose_with_velocity(pred_path, current_time);

  if (!interpolation_result) {
    return {};
  }

  const auto & i = ego_info;
  const auto & base_to_front = i.max_longitudinal_offset_m;
  const auto & base_to_rear = i.rear_overhang_m;
  const auto & width = i.vehicle_width_m;
  const auto & pose = interpolation_result->pose;
  const auto & velocity = interpolation_result->velocity;

  const auto ego_polygon =
    autoware::universe_utils::toFootprint(pose, base_to_front, base_to_rear, width);

  return PoseWithVelocityAndPolygonStamped{current_time, pose, velocity, ego_polygon};
}

std::optional<PoseWithVelocityAndPolygonStamped>
get_interpolated_pose_with_velocity_and_polygon_stamped(
  const std::vector<PoseWithVelocityAndPolygonStamped> & pred_path, const double current_time,
  const Shape & shape)
{
  auto to_pose_with_velocity_stamped_vector = [](const auto & pred_path) {
    std::vector<PoseWithVelocityStamped> path;
    path.reserve(pred_path.size());
    for (const auto & elem : pred_path) {
      path.push_back(PoseWithVelocityStamped{elem.time, elem.pose, elem.velocity});
    }
    return path;
  };

  const auto interpolation_result = calc_interpolated_pose_with_velocity(
    to_pose_with_velocity_stamped_vector(pred_path), current_time);

  if (!interpolation_result) {
    return {};
  }

  const auto & pose = interpolation_result->pose;
  const auto & velocity = interpolation_result->velocity;

  const auto obj_polygon = autoware::universe_utils::toPolygon2d(pose, shape);

  return PoseWithVelocityAndPolygonStamped{current_time, pose, velocity, obj_polygon};
}

template <typename T, typename F>
std::vector<T> filterPredictedPathByTimeHorizon(
  const std::vector<T> & path, const double time_horizon, const F & interpolateFunc)
{
  std::vector<T> filtered_path;

  for (const auto & elem : path) {
    if (elem.time < time_horizon) {
      filtered_path.push_back(elem);
    } else {
      break;
    }
  }

  const auto interpolated_opt = interpolateFunc(path, time_horizon);
  if (interpolated_opt) {
    filtered_path.push_back(*interpolated_opt);
  }

  return filtered_path;
};

std::vector<PoseWithVelocityStamped> filterPredictedPathByTimeHorizon(
  const std::vector<PoseWithVelocityStamped> & path, const double time_horizon)
{
  return filterPredictedPathByTimeHorizon(
    path, time_horizon, [](const auto & path, const auto & time) {
      return calc_interpolated_pose_with_velocity(path, time);
    });
}

ExtendedPredictedObject filterObjectPredictedPathByTimeHorizon(
  const ExtendedPredictedObject & object, const double time_horizon,
  const bool check_all_predicted_path)
{
  auto filtered_object = object;
  auto filtered_predicted_paths = getPredictedPathFromObj(object, check_all_predicted_path);

  for (auto & predicted_path : filtered_predicted_paths) {
    // path is vector of polygon
    const auto filtered_path = filterPredictedPathByTimeHorizon(
      predicted_path.path, time_horizon, [&object](const auto & poses, double t) {
        return get_interpolated_pose_with_velocity_and_polygon_stamped(poses, t, object.shape);
      });
    predicted_path.path = filtered_path;
  }

  filtered_object.predicted_paths = filtered_predicted_paths;
  return filtered_object;
}

ExtendedPredictedObjects filterObjectPredictedPathByTimeHorizon(
  const ExtendedPredictedObjects & objects, const double time_horizon,
  const bool check_all_predicted_path)
{
  ExtendedPredictedObjects filtered_objects;
  filtered_objects.reserve(objects.size());

  for (const auto & object : objects) {
    filtered_objects.push_back(
      filterObjectPredictedPathByTimeHorizon(object, time_horizon, check_all_predicted_path));
  }

  return filtered_objects;
}

std::vector<PoseWithVelocityStamped> filterPredictedPathAfterTargetPose(
  const std::vector<PoseWithVelocityStamped> & path, const Pose & target_pose)
{
  std::vector<PoseWithVelocityStamped> filtered_path;

  const auto target_idx =
    std::min_element(path.begin(), path.end(), [&target_pose](const auto & a, const auto & b) {
      return calcDistance2d(a.pose.position, target_pose.position) <
             calcDistance2d(b.pose.position, target_pose.position);
    });

  std::copy(target_idx, path.end(), std::back_inserter(filtered_path));

  return filtered_path;
};

bool checkSafetyWithRSS(
  const PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path,
  const std::vector<ExtendedPredictedObject> & objects, CollisionCheckDebugMap & debug_map,
  const BehaviorPathPlannerParameters & parameters, const RSSparams & rss_params,
  const bool check_all_predicted_path, const double hysteresis_factor,
  const double yaw_difference_th)
{
  // Check for collisions with each predicted path of the object
  const bool is_safe = !std::any_of(objects.begin(), objects.end(), [&](const auto & object) {
    auto current_debug_data = utils::path_safety_checker::createObjectDebug(object);

    const auto obj_predicted_paths =
      utils::path_safety_checker::getPredictedPathFromObj(object, check_all_predicted_path);

    return std::any_of(
      obj_predicted_paths.begin(), obj_predicted_paths.end(), [&](const auto & obj_path) {
        const bool has_collision = !utils::path_safety_checker::checkCollision(
          planned_path, ego_predicted_path, object, obj_path, parameters, rss_params,
          hysteresis_factor, yaw_difference_th, current_debug_data.second);

        utils::path_safety_checker::updateCollisionCheckDebugMap(
          debug_map, current_debug_data, !has_collision);

        return has_collision;
      });
  });

  return is_safe;
}

bool checkSafetyWithIntegralPredictedPolygon(
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path, const VehicleInfo & vehicle_info,
  const ExtendedPredictedObjects & objects, const bool check_all_predicted_path,
  const IntegralPredictedPolygonParams & params, CollisionCheckDebugMap & debug_map)
{
  const std::vector<PoseWithVelocityStamped> filtered_ego_path = filterPredictedPathByTimeHorizon(
    ego_predicted_path, params.time_horizon);  // path is vector of pose
  const std::vector<Polygon2d> extended_ego_polygons =
    createExtendedPolygonsFromPoseWithVelocityStamped(
      filtered_ego_path, vehicle_info, params.forward_margin, params.backward_margin,
      params.lat_margin);

  const ExtendedPredictedObjects filtered_path_objects = filterObjectPredictedPathByTimeHorizon(
    objects, params.time_horizon, check_all_predicted_path);  // path is vector of polygon

  Polygon2d ego_integral_polygon{};
  for (const auto & ego_polygon : extended_ego_polygons) {
    std::vector<Polygon2d> unions{};
    boost::geometry::union_(ego_integral_polygon, ego_polygon, unions);
    if (!unions.empty()) {
      ego_integral_polygon = unions.front();
      boost::geometry::correct(ego_integral_polygon);
    }
  }

  // check collision
  for (const auto & object : filtered_path_objects) {
    CollisionCheckDebugPair debug_pair = createObjectDebug(object);
    for (const auto & path : object.predicted_paths) {
      for (const auto & pose_with_poly : path.path) {
        if (boost::geometry::intersects(ego_integral_polygon, pose_with_poly.poly)) {
          debug_pair.second.ego_predicted_path = ego_predicted_path;  // raw path
          debug_pair.second.obj_predicted_path = path.path;           // raw path
          debug_pair.second.extended_obj_polygon = pose_with_poly.poly;
          debug_pair.second.extended_ego_polygon =
            ego_integral_polygon;  // time filtered extended polygon
          updateCollisionCheckDebugMap(debug_map, debug_pair, /*is_safe=*/false);
          return false;
        }
      }
    }
    updateCollisionCheckDebugMap(debug_map, debug_pair, /*is_safe=*/true);
  }
  return true;
}

bool checkCollision(
  const PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & predicted_ego_path,
  const ExtendedPredictedObject & target_object,
  const PredictedPathWithPolygon & target_object_path,
  const BehaviorPathPlannerParameters & common_parameters, const RSSparams & rss_parameters,
  const double hysteresis_factor, const double yaw_difference_th, CollisionCheckDebug & debug)
{
  const auto collided_polygons = get_collided_polygons(
    planned_path, predicted_ego_path, target_object, target_object_path,
    common_parameters.vehicle_info, rss_parameters, hysteresis_factor,
    std::numeric_limits<double>::max(), yaw_difference_th, debug);
  return collided_polygons.empty();
}

std::vector<Polygon2d> get_collided_polygons(
  [[maybe_unused]] const PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & predicted_ego_path,
  const ExtendedPredictedObject & target_object,
  const PredictedPathWithPolygon & target_object_path, const VehicleInfo & vehicle_info,
  const RSSparams & rss_parameters, double hysteresis_factor, const double max_velocity_limit,
  const double yaw_difference_th, CollisionCheckDebug & debug)
{
  {
    debug.ego_predicted_path = predicted_ego_path;
    debug.obj_predicted_path = target_object_path.path;
    debug.current_obj_pose = target_object.initial_pose;
  }

  std::vector<Polygon2d> collided_polygons{};
  collided_polygons.reserve(target_object_path.path.size());
  for (const auto & obj_pose_with_poly : target_object_path.path) {
    const auto & current_time = obj_pose_with_poly.time;

    // get object information at current time
    const auto & obj_pose = obj_pose_with_poly.pose;
    const auto & obj_polygon = obj_pose_with_poly.poly;
    const auto object_velocity = obj_pose_with_poly.velocity;

    // get ego information at current time
    // Note: we can create these polygons in advance. However, it can decrease the readability and
    // variability
    const auto & ego_vehicle_info = vehicle_info;
    const auto interpolated_data = get_interpolated_pose_with_velocity_and_polygon_stamped(
      predicted_ego_path, current_time, ego_vehicle_info);
    if (!interpolated_data) {
      continue;
    }
    const auto & ego_pose = interpolated_data->pose;
    const auto & ego_polygon = interpolated_data->poly;
    const auto ego_velocity = std::min(interpolated_data->velocity, max_velocity_limit);

    const double ego_yaw = tf2::getYaw(ego_pose.orientation);
    const double object_yaw = tf2::getYaw(obj_pose.orientation);
    const double yaw_difference = autoware::universe_utils::normalizeRadian(ego_yaw - object_yaw);
    if (std::abs(yaw_difference) > yaw_difference_th) continue;

    // check intersects
    if (boost::geometry::intersects(ego_polygon, obj_polygon)) {
      if (collided_polygons.empty()) {
        debug.unsafe_reason = "overlap_polygon";
        debug.expected_ego_pose = ego_pose;
        debug.expected_obj_pose = obj_pose;
        debug.extended_ego_polygon = ego_polygon;
        debug.extended_obj_polygon = obj_polygon;
      }
      collided_polygons.push_back(obj_polygon);

      continue;
    }

    // compute which one is at the front of the other
    const bool is_object_front =
      isTargetObjectFront(ego_pose, obj_polygon, ego_vehicle_info.max_longitudinal_offset_m);
    const auto & [front_object_velocity, rear_object_velocity] =
      is_object_front ? std::make_pair(object_velocity, ego_velocity)
                      : std::make_pair(ego_velocity, object_velocity);

    // compute rss dist
    const auto rss_dist =
      calcRssDistance(front_object_velocity, rear_object_velocity, rss_parameters);

    // minimum longitudinal length
    const auto min_lon_length =
      calc_minimum_longitudinal_length(front_object_velocity, rear_object_velocity, rss_parameters);

    const auto & lon_offset = std::max(rss_dist, min_lon_length) * hysteresis_factor;
    const auto & lat_margin = rss_parameters.lateral_distance_max_threshold * hysteresis_factor;
    // TODO(watanabe) fix hard coding value
    const bool is_stopped_object = object_velocity < 0.3;
    const auto extended_ego_polygon = [&]() {
      if (!is_object_front) {
        return ego_polygon;
      }

      if (rss_parameters.extended_polygon_policy == "rectangle") {
        return createExtendedPolygon(
          ego_pose, ego_vehicle_info, lon_offset, lat_margin, is_stopped_object, debug);
      }

      if (rss_parameters.extended_polygon_policy == "along_path") {
        return create_extended_polygon_along_path(
          planned_path, ego_pose, ego_vehicle_info, lon_offset, lat_margin, is_stopped_object,
          debug);
      }

      throw std::domain_error("invalid rss parameter. please select 'rectangle' or 'along_path'.");
    }();
    const auto & extended_obj_polygon =
      is_object_front ? obj_polygon
                      : createExtendedPolygon(
                          obj_pose_with_poly, lon_offset, lat_margin, is_stopped_object, debug);

    // check intersects with extended polygon
    if (boost::geometry::intersects(extended_ego_polygon, extended_obj_polygon)) {
      if (collided_polygons.empty()) {
        debug.unsafe_reason = "overlap_extended_polygon";
        debug.rss_longitudinal = rss_dist;
        debug.inter_vehicle_distance = min_lon_length;
        debug.expected_ego_pose = ego_pose;
        debug.expected_obj_pose = obj_pose;
        debug.extended_ego_polygon = extended_ego_polygon;
        debug.extended_obj_polygon = extended_obj_polygon;
        debug.is_front = is_object_front;
      }
      collided_polygons.push_back(obj_polygon);
    }
  }

  return collided_polygons;
}

bool checkPolygonsIntersects(
  const std::vector<Polygon2d> & polys_1, const std::vector<Polygon2d> & polys_2)
{
  for (const auto & poly_1 : polys_1) {
    for (const auto & poly_2 : polys_2) {
      if (boost::geometry::intersects(poly_1, poly_2)) {
        return true;
      }
    }
  }
  return false;
}

CollisionCheckDebugPair createObjectDebug(const ExtendedPredictedObject & obj)
{
  CollisionCheckDebug debug;
  debug.current_obj_pose = obj.initial_pose;
  debug.extended_obj_polygon = autoware::universe_utils::toPolygon2d(obj.initial_pose, obj.shape);
  debug.obj_shape = obj.shape;
  debug.current_twist = obj.initial_twist;
  return {autoware::universe_utils::toBoostUUID(obj.uuid), debug};
}

void updateCollisionCheckDebugMap(
  CollisionCheckDebugMap & debug_map, CollisionCheckDebugPair & object_debug, bool is_safe)
{
  auto & [key, element] = object_debug;
  element.is_safe = is_safe;
  if (debug_map.find(key) != debug_map.end()) {
    debug_map[key] = element;
    return;
  }

  debug_map.insert(object_debug);
}

double calc_obstacle_min_length(const Shape & shape)
{
  if (shape.type == Shape::BOUNDING_BOX) {
    return std::min(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  }
  if (shape.type == Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  }
  if (shape.type == Shape::POLYGON) {
    double min_length_to_point = std::numeric_limits<double>::max();
    for (const auto rel_point : shape.footprint.points) {
      const double length_to_point = std::hypot(rel_point.x, rel_point.y);
      if (min_length_to_point > length_to_point) {
        min_length_to_point = length_to_point;
      }
    }
    return min_length_to_point;
  }

  throw std::logic_error("The shape type is not supported in obstacle_cruise_planner.");
}

double calc_obstacle_max_length(const Shape & shape)
{
  if (shape.type == Shape::BOUNDING_BOX) {
    return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  }
  if (shape.type == Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  }
  if (shape.type == Shape::POLYGON) {
    double max_length_to_point = 0.0;
    for (const auto rel_point : shape.footprint.points) {
      const double length_to_point = std::hypot(rel_point.x, rel_point.y);
      if (max_length_to_point < length_to_point) {
        max_length_to_point = length_to_point;
      }
    }
    return max_length_to_point;
  }

  throw std::logic_error("The shape type is not supported in obstacle_cruise_planner.");
}

std::pair<bool, bool> checkObjectsCollisionRough(
  const PathWithLaneId & path, const PredictedObjects & objects, const double margin,
  const BehaviorPathPlannerParameters & parameters, const bool use_offset_ego_point)
{
  const auto & points = path.points;

  std::pair<bool, bool> has_collision = {false, false};  // {min_distance, max_distance}
  for (const auto & object : objects.objects) {
    // calculate distance between object center and ego base_link
    const Point & object_point = object.kinematics.initial_pose_with_covariance.pose.position;
    const double distance = std::invoke([&]() -> double {
      if (use_offset_ego_point) {
        const size_t nearest_segment_idx = findNearestSegmentIndex(points, object_point);
        const double offset_length =
          calcLongitudinalOffsetToSegment(points, nearest_segment_idx, object_point);
        const auto offset_point =
          calcLongitudinalOffsetPoint(points, nearest_segment_idx, offset_length);
        const Point ego_point =
          offset_point ? offset_point.value()
                       : points.at(findNearestIndex(points, object_point)).point.pose.position;
        return autoware::universe_utils::calcDistance2d(ego_point, object_point);
      }
      const Point ego_point = points.at(findNearestIndex(points, object_point)).point.pose.position;
      return autoware::universe_utils::calcDistance2d(ego_point, object_point);
    });

    // calculate min and max length from object center to edge
    const double object_min_length = calc_obstacle_min_length(object.shape);
    const double object_max_length = calc_obstacle_max_length(object.shape);

    // calculate min and max length from ego base_link to edge
    const auto & p = parameters;
    const double ego_min_length =
      std::min({p.vehicle_width / 2, p.front_overhang / 2, p.rear_overhang / 2});
    const double ego_max_length = std::max(
      std::hypot(p.vehicle_width / 2, p.front_overhang),
      std::hypot(p.vehicle_width / 2, p.rear_overhang));

    // calculate min and max distance between object footprint and ego footprint
    const double min_distance = distance - object_max_length - ego_max_length;
    const double max_distance = distance - object_min_length - ego_min_length;

    if (min_distance < margin) {
      has_collision.first = true;
    }
    if (max_distance < margin) {
      has_collision.second = true;
    }
  }
  return has_collision;
}

double calculateRoughDistanceToObjects(
  const PathWithLaneId & path, const PredictedObjects & objects,
  const BehaviorPathPlannerParameters & parameters, const bool use_offset_ego_point,
  const std::string & distance_type)
{
  const auto & points = path.points;

  const double ego_length = std::invoke([&]() -> double {
    const auto & p = parameters;
    if (distance_type == "min") {
      return std::max(
        std::hypot(p.vehicle_width / 2, p.front_overhang),
        std::hypot(p.vehicle_width / 2, p.rear_overhang));
    }
    if (distance_type == "max") {
      return std::min({p.vehicle_width / 2, p.front_overhang / 2, p.rear_overhang / 2});
    }
    throw std::invalid_argument("Invalid distance type");
  });

  double min_distance = std::numeric_limits<double>::max();
  for (const auto & object : objects.objects) {
    const double object_length = std::invoke([&]() -> double {
      if (distance_type == "min") {
        return calc_obstacle_max_length(object.shape);
      }
      if (distance_type == "max") {
        return calc_obstacle_min_length(object.shape);
      }
      throw std::invalid_argument("Invalid distance type");
    });
    const Point & object_point = object.kinematics.initial_pose_with_covariance.pose.position;
    const double distance = std::invoke([&]() -> double {
      if (use_offset_ego_point) {
        const size_t nearest_segment_idx = findNearestSegmentIndex(points, object_point);
        const double offset_length =
          calcLongitudinalOffsetToSegment(points, nearest_segment_idx, object_point);
        const auto offset_point =
          calcLongitudinalOffsetPoint(points, nearest_segment_idx, offset_length);
        const Point ego_point =
          offset_point ? offset_point.value()
                       : points.at(findNearestIndex(points, object_point)).point.pose.position;
        return std::max(calcDistance2d(ego_point, object_point) - object_length - ego_length, 0.0);
      }
      const Point ego_point = points.at(findNearestIndex(points, object_point)).point.pose.position;
      return std::max(calcDistance2d(ego_point, object_point) - object_length - ego_length, 0.0);
    });
    min_distance = std::min(min_distance, distance);
  }
  return min_distance;
}

}  // namespace autoware::behavior_path_planner::utils::path_safety_checker
