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

#include "autoware/behavior_path_dynamic_obstacle_avoidance_module/scene.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/object_recognition_utils/predicted_path_utils.hpp"
#include "autoware/signal_processing/lowpass_filter_1d.hpp"
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/algorithms/union.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
namespace
{
geometry_msgs::msg::Point toGeometryPoint(const autoware::universe_utils::Point2d & point)
{
  geometry_msgs::msg::Point geom_obj_point;
  geom_obj_point.x = point.x();
  geom_obj_point.y = point.y();
  return geom_obj_point;
}

MinMaxValue getMinMaxValues(const std::vector<double> & vec)
{
  const size_t min_idx = std::distance(vec.begin(), std::min_element(vec.begin(), vec.end()));

  const size_t max_idx = std::distance(vec.begin(), std::max_element(vec.begin(), vec.end()));

  return MinMaxValue{vec.at(min_idx), vec.at(max_idx)};
}

MinMaxValue combineMinMaxValues(const MinMaxValue & r1, const MinMaxValue & r2)
{
  return MinMaxValue{std::min(r1.min_value, r2.min_value), std::max(r1.max_value, r2.max_value)};
}

void appendObjectMarker(MarkerArray & marker_array, const geometry_msgs::msg::Pose & obj_pose)
{
  auto marker = autoware::universe_utils::createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "dynamic_objects_to_avoid",
    marker_array.markers.size(), visualization_msgs::msg::Marker::CUBE,
    autoware::universe_utils::createMarkerScale(3.0, 1.0, 1.0),
    autoware::universe_utils::createMarkerColor(1.0, 0.5, 0.6, 0.8));
  marker.pose = obj_pose;

  marker_array.markers.push_back(marker);
}

void appendExtractedPolygonMarker(
  MarkerArray & marker_array, const autoware::universe_utils::Polygon2d & obj_poly,
  const double obj_z)
{
  auto marker = autoware::universe_utils::createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "extracted_polygons", marker_array.markers.size(),
    visualization_msgs::msg::Marker::LINE_STRIP,
    autoware::universe_utils::createMarkerScale(0.1, 0.0, 0.0),
    autoware::universe_utils::createMarkerColor(1.0, 0.5, 0.6, 0.8));

  // NOTE: obj_poly.outer() has already duplicated points to close the polygon.
  for (size_t i = 0; i < obj_poly.outer().size(); ++i) {
    const auto & bound_point = obj_poly.outer().at(i);

    geometry_msgs::msg::Point bound_geom_point;
    bound_geom_point.x = bound_point.x();
    bound_geom_point.y = bound_point.y();
    bound_geom_point.z = obj_z;
    marker.points.push_back(bound_geom_point);
  }

  marker_array.markers.push_back(marker);
}

bool isEndPointsConnected(
  const lanelet::ConstLanelet & left_lane, const lanelet::ConstLanelet & right_lane)
{
  const auto & left_back_point_2d = right_lane.leftBound2d().back().basicPoint();
  const auto & right_back_point_2d = left_lane.rightBound2d().back().basicPoint();

  constexpr double epsilon = 1e-5;
  return (right_back_point_2d - left_back_point_2d).norm() < epsilon;
}

template <typename T>
void pushUniqueVector(T & base_vector, const T & additional_vector)
{
  base_vector.insert(base_vector.end(), additional_vector.begin(), additional_vector.end());
}

template <typename T>
std::optional<T> getObjectFromUuid(const std::vector<T> & objects, const std::string & target_uuid)
{
  const auto itr = std::find_if(objects.begin(), objects.end(), [&](const auto & object) {
    return object.uuid == target_uuid;
  });

  if (itr == objects.end()) {
    return std::nullopt;
  }
  return *itr;
}

std::pair<double, double> projectObstacleVelocityToTrajectory(
  const std::vector<PathPointWithLaneId> & path_points, const PredictedObject & object,
  const size_t obj_idx)
{
  const auto & obj_pose = object.kinematics.initial_pose_with_covariance.pose;
  const double obj_yaw = tf2::getYaw(obj_pose.orientation);
  const double path_yaw = tf2::getYaw(path_points.at(obj_idx).point.pose.orientation);

  const Eigen::Rotation2Dd R_ego_to_obstacle(obj_yaw - path_yaw);
  const Eigen::Vector2d obstacle_velocity(
    object.kinematics.initial_twist_with_covariance.twist.linear.x,
    object.kinematics.initial_twist_with_covariance.twist.linear.y);
  const Eigen::Vector2d projected_velocity = R_ego_to_obstacle * obstacle_velocity;

  return std::make_pair(projected_velocity[0], projected_velocity[1]);
}

double calcObstacleMaxLength(const autoware_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  } else if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  } else if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    double max_length_to_point = 0.0;
    for (const auto rel_point : shape.footprint.points) {
      const double length_to_point = std::hypot(rel_point.x, rel_point.y);
      if (max_length_to_point < length_to_point) {
        max_length_to_point = length_to_point;
      }
    }
    return max_length_to_point;
  }

  throw std::logic_error("The shape type is not supported in dynamic_avoidance.");
}

double calcObstacleWidth(const autoware_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return shape.dimensions.y;
  } else if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    return shape.dimensions.x;
  } else if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    double max_length_to_point = 0.0;
    for (const auto rel_point : shape.footprint.points) {
      const double length_to_point = std::hypot(rel_point.x, rel_point.y);
      if (max_length_to_point < length_to_point) {
        max_length_to_point = length_to_point;
      }
    }
    return max_length_to_point;
  }
  throw std::logic_error("The shape type is not supported in dynamic_avoidance.");
}

double calcDiffAngleAgainstPath(
  const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::msg::Pose & target_pose, const size_t target_idx)
{
  const double traj_yaw = tf2::getYaw(path_points.at(target_idx).point.pose.orientation);
  const double target_yaw = tf2::getYaw(target_pose.orientation);

  const double diff_yaw = autoware::universe_utils::normalizeRadian(target_yaw - traj_yaw);
  return diff_yaw;
}

double calcDistanceToPath(
  const std::vector<geometry_msgs::msg::Pose> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_idx)
{
  if (target_idx == 0 || target_idx == points.size() - 1) {
    const double target_yaw = tf2::getYaw(points.at(target_idx).orientation);
    const double angle_to_target_pos =
      autoware::universe_utils::calcAzimuthAngle(points.at(target_idx).position, target_pos);
    const double diff_yaw =
      autoware::universe_utils::normalizeRadian(angle_to_target_pos - target_yaw);

    if (
      (target_idx == 0 && (diff_yaw < -M_PI_2 || M_PI_2 < diff_yaw)) ||
      (target_idx == points.size() - 1 && (-M_PI_2 < diff_yaw && diff_yaw < M_PI_2))) {
      return autoware::universe_utils::calcDistance2d(points.at(target_idx), target_pos);
    }
  }

  return std::abs(autoware::motion_utils::calcLateralOffset(points, target_pos));
}

bool isLeft(
  const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_idx)
{
  const double target_yaw = tf2::getYaw(path_points.at(target_idx).point.pose.orientation);
  const double angle_to_target_pos = autoware::universe_utils::calcAzimuthAngle(
    path_points.at(target_idx).point.pose.position, target_pos);
  const double diff_yaw =
    autoware::universe_utils::normalizeRadian(angle_to_target_pos - target_yaw);

  if (0 < diff_yaw) {
    return true;
  }
  return false;
}

template <typename T>
std::optional<T> getObstacleFromUuid(
  const std::vector<T> & obstacles, const std::string & target_uuid)
{
  const auto itr = std::find_if(obstacles.begin(), obstacles.end(), [&](const auto & obstacle) {
    return obstacle.uuid == target_uuid;
  });

  if (itr == obstacles.end()) {
    return std::nullopt;
  }
  return *itr;
}

double calcDistanceToSegment(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2_first,
  const geometry_msgs::msg::Point & p2_second)
{
  const Eigen::Vector2d first_to_target(p1.x - p2_first.x, p1.y - p2_first.y);
  const Eigen::Vector2d second_to_target(p1.x - p2_second.x, p1.y - p2_second.y);
  const Eigen::Vector2d first_to_second(p2_second.x - p2_first.x, p2_second.y - p2_first.y);

  if (first_to_target.dot(first_to_second) < 0) {
    return first_to_target.norm();
  }
  if (second_to_target.dot(-first_to_second) < 0) {
    return second_to_target.norm();
  }

  const Eigen::Vector2d p2_nearest =
    Eigen::Vector2d{p2_first.x, p2_first.y} +
    first_to_second * first_to_target.dot(first_to_second) / std::pow(first_to_second.norm(), 2);
  return (Eigen::Vector2d{p1.x, p1.y} - p2_nearest).norm();
}

std::optional<std::pair<size_t, geometry_msgs::msg::Point>> intersectLines(
  const std::vector<geometry_msgs::msg::Pose> & source_line,
  const std::vector<geometry_msgs::msg::Point> & target_line)
{
  for (int source_seg_idx = 0; source_seg_idx < static_cast<int>(source_line.size()) - 1;
       ++source_seg_idx) {
    for (int target_seg_idx = 0; target_seg_idx < static_cast<int>(target_line.size()) - 1;
         ++target_seg_idx) {
      const auto intersect_point = autoware::universe_utils::intersect(
        source_line.at(source_seg_idx).position, source_line.at(source_seg_idx + 1).position,
        target_line.at(target_seg_idx), target_line.at(target_seg_idx + 1));
      if (intersect_point) {
        return std::make_pair(source_seg_idx, *intersect_point);
      }
    }
  }
  return std::nullopt;
}

std::vector<geometry_msgs::msg::Point> convertToPoints(
  const std::vector<geometry_msgs::msg::Pose> & poses)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & pose : poses) {
    points.push_back(pose.position);
  }
  return points;
}

// NOTE: Giving PathPointWithLaneId to autoware_motion_utils functions
//       cost a lot. Instead, using Pose is much faster (around x10).
std::vector<geometry_msgs::msg::Pose> toGeometryPoints(
  const std::vector<PathPointWithLaneId> & path_points)
{
  std::vector<geometry_msgs::msg::Pose> geom_points;
  for (const auto & path_point : path_points) {
    geom_points.push_back(path_point.point.pose);
  }
  return geom_points;
}

size_t getNearestIndexFromSegmentIndex(
  const std::vector<geometry_msgs::msg::Pose> & points, const size_t seg_idx,
  const geometry_msgs::msg::Point & target_pos)
{
  const double first_dist =
    autoware::universe_utils::calcDistance2d(points.at(seg_idx), target_pos);
  const double second_dist =
    autoware::universe_utils::calcDistance2d(points.at(seg_idx + 1), target_pos);
  if (first_dist < second_dist) {
    return seg_idx;
  }
  return seg_idx + 1;
}
}  // namespace

DynamicObstacleAvoidanceModule::DynamicObstacleAvoidanceModule(
  const std::string & name, rclcpp::Node & node,
  std::shared_ptr<DynamicAvoidanceParameters> parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> planning_factor_interface)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map, planning_factor_interface},  // NOLINT
  parameters_{std::move(parameters)},
  target_objects_manager_{TargetObjectsManager(
    parameters_->successive_num_to_entry_dynamic_avoidance_condition,
    parameters_->successive_num_to_exit_dynamic_avoidance_condition)}
{
}

bool DynamicObstacleAvoidanceModule::isExecutionRequested() const
{
  RCLCPP_DEBUG(getLogger(), "DYNAMIC AVOIDANCE isExecutionRequested.");

  const auto input_path = getPreviousModuleOutput().path;
  if (input_path.points.size() < 2) {
    return false;
  }

  // check if the ego is driving forward
  const auto is_driving_forward = autoware::motion_utils::isDrivingForward(input_path.points);
  if (!is_driving_forward || !(*is_driving_forward)) {
    return false;
  }

  // check if the planner is already running
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    return true;
  }

  // check if there is target objects to avoid
  return !target_objects_.empty();
}

bool DynamicObstacleAvoidanceModule::isExecutionReady() const
{
  RCLCPP_DEBUG(getLogger(), "DYNAMIC AVOIDANCE isExecutionReady.");
  return true;
}

void DynamicObstacleAvoidanceModule::updateData()
{
  universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  info_marker_.markers.clear();
  debug_marker_.markers.clear();

  const auto prev_objects = target_objects_manager_.getValidObjects();
  target_objects_manager_.initialize();

  // 1. Rough filtering of target objects with small computing cost
  registerRegulatedObjects(prev_objects);
  registerUnregulatedObjects(prev_objects);

  const auto & ego_lat_feasible_paths = generateLateralFeasiblePaths(getEgoPose(), getEgoSpeed());
  target_objects_manager_.finalize(ego_lat_feasible_paths);

  // 2. Precise filtering of target objects and check if they should be avoided
  determineWhetherToAvoidAgainstRegulatedObjects(prev_objects);
  determineWhetherToAvoidAgainstUnregulatedObjects(prev_objects);

  const auto target_objects_candidate = target_objects_manager_.getValidObjects();
  target_objects_.clear();
  for (const auto & target_object_candidate : target_objects_candidate) {
    if (target_object_candidate.should_be_avoided) {
      target_objects_.push_back(target_object_candidate);
    }
  }
}

bool DynamicObstacleAvoidanceModule::canTransitSuccessState()
{
  return planner_data_->dynamic_object->objects.empty();
}

BehaviorModuleOutput DynamicObstacleAvoidanceModule::plan()
{
  const auto & input_path = getPreviousModuleOutput().path;
  if (input_path.points.empty()) {
    throw std::runtime_error("input path is empty");
  }

  const auto ego_path_reserve_poly = calcEgoPathReservePoly(input_path);

  // create obstacles to avoid (= extract from the drivable area)
  std::vector<DrivableAreaInfo::Obstacle> obstacles_for_drivable_area;
  for (const auto & object : target_objects_) {
    const auto obstacle_poly = [&]() {
      if (getObjectType(object.label) == ObjectType::UNREGULATED) {
        return calcPredictedPathBasedDynamicObstaclePolygon(object, ego_path_reserve_poly);
      }

      if (parameters_->polygon_generation_method == PolygonGenerationMethod::EGO_PATH_BASE) {
        return calcEgoPathBasedDynamicObstaclePolygon(object);
      }
      if (parameters_->polygon_generation_method == PolygonGenerationMethod::OBJECT_PATH_BASE) {
        return calcObjectPathBasedDynamicObstaclePolygon(object);
      }
      throw std::logic_error("The polygon_generation_method's string is invalid.");
    }();
    if (obstacle_poly) {
      obstacles_for_drivable_area.push_back(
        {object.pose, obstacle_poly.value(), object.is_collision_left});

      appendObjectMarker(info_marker_, object.pose);
      appendExtractedPolygonMarker(debug_marker_, obstacle_poly.value(), object.pose.position.z);
    }
  }
  // generate drivable lanes
  DrivableAreaInfo current_drivable_area_info;
  if (parameters_->expand_drivable_area) {
    auto current_lanelets =
      getCurrentLanesFromPath(getPreviousModuleOutput().reference_path, planner_data_);
    std::for_each(current_lanelets.begin(), current_lanelets.end(), [&](const auto & lanelet) {
      current_drivable_area_info.drivable_lanes.push_back(
        generateExpandedDrivableLanes(lanelet, planner_data_, parameters_));
    });
  } else {
    current_drivable_area_info.drivable_lanes =
      getPreviousModuleOutput().drivable_area_info.drivable_lanes;
  }
  current_drivable_area_info.obstacles = obstacles_for_drivable_area;
  current_drivable_area_info.enable_expanding_hatched_road_markings =
    parameters_->use_hatched_road_markings;

  BehaviorModuleOutput output;
  output.path = input_path;
  output.drivable_area_info = utils::combineDrivableAreaInfo(
    current_drivable_area_info, getPreviousModuleOutput().drivable_area_info);
  output.reference_path = getPreviousModuleOutput().reference_path;
  output.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
  output.modified_goal = getPreviousModuleOutput().modified_goal;

  return output;
}

CandidateOutput DynamicObstacleAvoidanceModule::planCandidate() const
{
  auto candidate_path = utils::generateCenterLinePath(planner_data_);
  return CandidateOutput(*candidate_path);
}

BehaviorModuleOutput DynamicObstacleAvoidanceModule::planWaitingApproval()
{
  BehaviorModuleOutput out = plan();
  return out;
}

ObjectType DynamicObstacleAvoidanceModule::getObjectType(const uint8_t label) const
{
  using autoware_perception_msgs::msg::ObjectClassification;

  if (label == ObjectClassification::CAR && parameters_->avoid_car) {
    return ObjectType::REGULATED;
  }
  if (label == ObjectClassification::TRUCK && parameters_->avoid_truck) {
    return ObjectType::REGULATED;
  }
  if (label == ObjectClassification::BUS && parameters_->avoid_bus) {
    return ObjectType::REGULATED;
  }
  if (label == ObjectClassification::TRAILER && parameters_->avoid_trailer) {
    return ObjectType::REGULATED;
  }
  if (label == ObjectClassification::UNKNOWN && parameters_->avoid_unknown) {
    return ObjectType::UNREGULATED;
  }
  if (label == ObjectClassification::BICYCLE && parameters_->avoid_bicycle) {
    return ObjectType::UNREGULATED;
  }
  if (label == ObjectClassification::MOTORCYCLE && parameters_->avoid_motorcycle) {
    return ObjectType::REGULATED;
  }
  if (label == ObjectClassification::PEDESTRIAN && parameters_->avoid_pedestrian) {
    return ObjectType::UNREGULATED;
  }
  return ObjectType::OUT_OF_SCOPE;
}

void DynamicObstacleAvoidanceModule::registerRegulatedObjects(
  const std::vector<DynamicAvoidanceObject> & prev_objects)
{
  universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto input_path = getPreviousModuleOutput().path;
  const auto input_points = toGeometryPoints(input_path.points);  // for efficient computation
  const auto & predicted_objects = planner_data_->dynamic_object->objects;

  for (const auto & predicted_object : predicted_objects) {
    const auto obj_uuid = autoware::universe_utils::toHexString(predicted_object.object_id);
    const auto & obj_pose = predicted_object.kinematics.initial_pose_with_covariance.pose;
    const double obj_vel_norm = std::hypot(
      predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x,
      predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y);
    const auto prev_object = getObstacleFromUuid(prev_objects, obj_uuid);
    const auto obj_path = *std::max_element(
      predicted_object.kinematics.predicted_paths.begin(),
      predicted_object.kinematics.predicted_paths.end(),
      [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });
    const size_t obj_seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(input_points, obj_pose.position);
    const size_t obj_idx =
      getNearestIndexFromSegmentIndex(input_points, obj_seg_idx, obj_pose.position);

    // 1.a. check label
    if (getObjectType(predicted_object.classification.front().label) != ObjectType::REGULATED) {
      continue;
    }

    // 1.b. check obstacle velocity
    const auto [obj_tangent_vel, obj_normal_vel] =
      projectObstacleVelocityToTrajectory(input_path.points, predicted_object, obj_idx);
    if (
      std::abs(obj_tangent_vel) < parameters_->min_obstacle_vel ||
      parameters_->max_obstacle_vel < std::abs(obj_tangent_vel)) {
      continue;
    }

    // 1.c. check if object is not crossing ego's path
    const double obj_angle = calcDiffAngleAgainstPath(input_path.points, obj_pose, obj_idx);
    const double max_crossing_object_angle = 0.0 <= obj_tangent_vel
                                               ? parameters_->max_overtaking_crossing_object_angle
                                               : parameters_->max_oncoming_crossing_object_angle;
    const bool is_obstacle_crossing_path = max_crossing_object_angle < std::abs(obj_angle) &&
                                           max_crossing_object_angle < M_PI - std::abs(obj_angle);
    const double min_crossing_object_vel = 0.0 <= obj_tangent_vel
                                             ? parameters_->min_overtaking_crossing_object_vel
                                             : parameters_->min_oncoming_crossing_object_vel;
    const bool is_crossing_object_to_ignore =
      min_crossing_object_vel < obj_vel_norm && is_obstacle_crossing_path;
    if (is_crossing_object_to_ignore) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since it crosses the ego's path.",
        obj_uuid.c_str());
      continue;
    }

    // 1.e. check if object lateral offset to ego's path is small enough
    const double obj_dist_to_path = calcDistanceToPath(input_points, obj_pose.position, obj_idx);
    const bool is_object_far_from_path = isObjectFarFromPath(predicted_object, obj_dist_to_path);
    if (is_object_far_from_path) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since lateral offset is large.", obj_uuid.c_str());
      continue;
    }

    // 1.f. calculate the object is on ego's path or not
    const bool is_object_on_ego_path =
      obj_dist_to_path <
      planner_data_->parameters.vehicle_width / 2.0 + parameters_->min_obj_lat_offset_to_ego_path;

    // 1.g. calculate latest time inside ego's path
    const auto latest_time_inside_ego_path = [&]() -> std::optional<rclcpp::Time> {
      if (!prev_object || !prev_object->latest_time_inside_ego_path) {
        if (is_object_on_ego_path) {
          return clock_->now();
        }
        return std::nullopt;
      }
      if (is_object_on_ego_path) {
        return clock_->now();
      }
      return *prev_object->latest_time_inside_ego_path;
    }();

    const auto target_object = DynamicAvoidanceObject(
      predicted_object, obj_tangent_vel, obj_normal_vel, is_object_on_ego_path,
      latest_time_inside_ego_path);
    target_objects_manager_.updateObject(obj_uuid, target_object);
  }
}

void DynamicObstacleAvoidanceModule::registerUnregulatedObjects(
  const std::vector<DynamicAvoidanceObject> & prev_objects)
{
  universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto input_path = getPreviousModuleOutput().path;
  const auto input_points = toGeometryPoints(input_path.points);  // for efficient computation
  const auto & predicted_objects = planner_data_->dynamic_object->objects;

  for (const auto & predicted_object : predicted_objects) {
    const auto obj_uuid = autoware::universe_utils::toHexString(predicted_object.object_id);
    const auto & obj_pose = predicted_object.kinematics.initial_pose_with_covariance.pose;
    const double obj_vel_norm = std::hypot(
      predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x,
      predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y);
    const auto prev_object = getObstacleFromUuid(prev_objects, obj_uuid);
    const auto obj_path = *std::max_element(
      predicted_object.kinematics.predicted_paths.begin(),
      predicted_object.kinematics.predicted_paths.end(),
      [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });
    const size_t obj_seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(input_points, obj_pose.position);
    const size_t obj_idx =
      getNearestIndexFromSegmentIndex(input_points, obj_seg_idx, obj_pose.position);

    // 1.a. Check if the obstacle is labeled as pedestrians, bicycle or similar.
    if (getObjectType(predicted_object.classification.front().label) != ObjectType::UNREGULATED) {
      continue;
    }

    // 1.b. Check if the object's velocity is within the module's coverage range.
    const auto [obj_tangent_vel, obj_normal_vel] =
      projectObstacleVelocityToTrajectory(input_path.points, predicted_object, obj_idx);
    if (
      obj_vel_norm < parameters_->min_obstacle_vel ||
      obj_vel_norm > parameters_->max_obstacle_vel) {
      continue;
    }

    // 1.c. Check if object' lateral velocity is small enough to be avoid.
    if (std::abs(obj_normal_vel) > parameters_->max_pedestrian_crossing_vel) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since it crosses the ego's path with its normal "
        "vel (%5.2f) m/s.",
        obj_uuid.c_str(), obj_normal_vel);
      continue;
    }

    // Blocks for compatibility with existing code
    //  1.e. check if object lateral distance to ego's path is small enough
    //  1.f. calculate the object is on ego's path or not

    const double dist_obj_center_to_path =
      std::abs(autoware::motion_utils::calcLateralOffset(input_points, obj_pose.position));
    const bool is_object_on_ego_path =
      dist_obj_center_to_path <
      planner_data_->parameters.vehicle_width / 2.0 + parameters_->min_obj_lat_offset_to_ego_path;

    // 1.g. calculate last time inside ego's path
    const auto latest_time_inside_ego_path = [&]() -> std::optional<rclcpp::Time> {
      if (!prev_object || !prev_object->latest_time_inside_ego_path) {
        if (is_object_on_ego_path) {
          return clock_->now();
        }
        return std::nullopt;
      }
      if (is_object_on_ego_path) {
        return clock_->now();
      }
      return *prev_object->latest_time_inside_ego_path;
    }();

    // register the object
    const auto target_object = DynamicAvoidanceObject(
      predicted_object, obj_tangent_vel, obj_normal_vel, is_object_on_ego_path,
      latest_time_inside_ego_path);
    target_objects_manager_.updateObject(obj_uuid, target_object);
  }
}

void DynamicObstacleAvoidanceModule::determineWhetherToAvoidAgainstRegulatedObjects(
  const std::vector<DynamicAvoidanceObject> & prev_objects)
{
  universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & input_path = getPreviousModuleOutput().path;
  const auto input_points = toGeometryPoints(input_path.points);  // for efficient computation

  for (const auto & object : target_objects_manager_.getValidObjects()) {
    if (getObjectType(object.label) != ObjectType::REGULATED) {
      continue;
    }

    const auto obj_uuid = object.uuid;
    const auto prev_object = getObstacleFromUuid(prev_objects, obj_uuid);
    const auto obj_path = *std::max_element(
      object.predicted_paths.begin(), object.predicted_paths.end(),
      [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });
    const size_t obj_seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(input_points, object.pose.position);
    const size_t obj_idx =
      getNearestIndexFromSegmentIndex(input_points, obj_seg_idx, object.pose.position);

    const auto & ref_points_for_obj_poly = input_points;

    // 2.a. check if object is not to be followed by ego
    const double obj_angle = calcDiffAngleAgainstPath(input_path.points, object.pose, obj_idx);
    const bool is_object_aligned_to_path =
      std::abs(obj_angle) < parameters_->max_front_object_angle ||
      M_PI - parameters_->max_front_object_angle < std::abs(obj_angle);
    if (
      object.is_object_on_ego_path && is_object_aligned_to_path &&
      parameters_->min_front_object_vel < object.vel) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since it is to be followed.", obj_uuid.c_str());
      continue;
    }

    // 2.b. calculate which side object exists against ego's path
    const bool is_object_left = isLeft(input_path.points, object.pose.position, obj_idx);
    const auto lat_lon_offset =
      getLateralLongitudinalOffset(input_points, object.pose, obj_seg_idx, object.shape);

    // 2.c. check if object will not cut in
    const bool will_object_cut_in =
      willObjectCutIn(input_path.points, obj_path, object.vel, lat_lon_offset);
    if (will_object_cut_in) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since it will cut in.", obj_uuid.c_str());
      continue;
    }

    // 2.d. check if object will not cut out
    const auto will_object_cut_out =
      willObjectCutOut(object.vel, object.lat_vel, is_object_left, prev_object);
    if (will_object_cut_out.decision) {
      printIgnoreReason(obj_uuid.c_str(), will_object_cut_out.reason);
      continue;
    }

    // 2.e. check if the ego will change the lane and the object will be outside the ego's path
    // const auto will_object_be_outside_ego_changing_path =
    //   willObjectBeOutsideEgoChangingPath(object.pose, object.shape, object.vel);
    // if (will_object_be_outside_ego_changing_path) {
    //   RCLCPP_INFO_EXPRESSION(
    //     getLogger(), parameters_->enable_debug_info,
    //     "[DynamicAvoidance] Ignore obstacle (%s) since the object will be outside ego's changing
    //     path", obj_uuid.c_str());
    //   continue;
    // }

    // 2.e. check time to collision
    const auto time_while_collision =
      calcTimeWhileCollision(input_path.points, object.vel, lat_lon_offset);
    const double time_to_collision = time_while_collision.time_to_start_collision;
    if (parameters_->max_stopped_object_vel < std::hypot(object.vel, object.lat_vel)) {
      // NOTE: Only not stopped object is filtered by time to collision.
      if (
        (0 <= object.vel &&
         parameters_->max_time_to_collision_overtaking_object < time_to_collision) ||
        (object.vel <= 0 &&
         parameters_->max_time_to_collision_oncoming_object < time_to_collision)) {
        const auto time_to_collision_str = time_to_collision == std::numeric_limits<double>::max()
                                             ? "infinity"
                                             : std::to_string(time_to_collision);
        RCLCPP_INFO_EXPRESSION(
          getLogger(), parameters_->enable_debug_info,
          "[DynamicAvoidance] Ignore obstacle (%s) since time to collision (%s) is large.",
          obj_uuid.c_str(), time_to_collision_str.c_str());
        continue;
      }
      if (time_to_collision < -parameters_->duration_to_hold_avoidance_overtaking_object) {
        const auto time_to_collision_str = time_to_collision == std::numeric_limits<double>::max()
                                             ? "infinity"
                                             : std::to_string(time_to_collision);
        RCLCPP_INFO_EXPRESSION(
          getLogger(), parameters_->enable_debug_info,
          "[DynamicAvoidance] Ignore obstacle (%s) since time to collision (%s) is a small "
          "negative value.",
          obj_uuid.c_str(), time_to_collision_str.c_str());
        continue;
      }
    }

    // 2.f. calculate which side object will be against ego's path
    const bool is_collision_left = [&]() {
      if (0.0 < object.vel) {
        return is_object_left;
      }
      const auto future_obj_pose =
        autoware::object_recognition_utils::calcInterpolatedPose(obj_path, time_to_collision);
      const size_t future_obj_idx =
        autoware::motion_utils::findNearestIndex(input_path.points, future_obj_pose->position);

      return future_obj_pose ? isLeft(input_path.points, future_obj_pose->position, future_obj_idx)
                             : is_object_left;
    }();

    // 2.g. check if the ego is not ahead of the object.
    const double signed_dist_ego_to_obj = [&]() {
      const size_t ego_seg_idx = planner_data_->findEgoSegmentIndex(input_path.points);
      const double lon_offset_ego_to_obj = autoware::motion_utils::calcSignedArcLength(
        input_path.points, getEgoPose().position, ego_seg_idx, lat_lon_offset.nearest_idx);
      if (0 < lon_offset_ego_to_obj) {
        return std::max(
          0.0, lon_offset_ego_to_obj - planner_data_->parameters.front_overhang +
                 lat_lon_offset.min_lon_offset);
      }
      return std::min(
        0.0, lon_offset_ego_to_obj + planner_data_->parameters.rear_overhang +
               lat_lon_offset.max_lon_offset);
    }();
    if (signed_dist_ego_to_obj < 0) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since distance from ego to object (%f) is less "
        "than 0.",
        obj_uuid.c_str(), signed_dist_ego_to_obj);
      continue;
    }

    // 2.h. calculate longitudinal and lateral offset to avoid to generate object polygon by
    // "ego_path_base"
    const auto obj_points = autoware::universe_utils::toPolygon2d(object.pose, object.shape);
    const auto lon_offset_to_avoid = calcMinMaxLongitudinalOffsetToAvoid(
      ref_points_for_obj_poly, object.pose, obj_points, object.vel, obj_path, object.shape,
      time_while_collision);
    const auto lat_offset_to_avoid = calcMinMaxLateralOffsetToAvoidRegulatedObject(
      ref_points_for_obj_poly, obj_points, object.pose.position, object.vel, is_collision_left,
      object.lat_vel, prev_object);

    if (!lat_offset_to_avoid) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since the object laterally covers the ego's path "
        "enough",
        obj_uuid.c_str());
      continue;
    }

    const bool should_be_avoided = true;
    target_objects_manager_.updateObjectVariables(
      obj_uuid, lon_offset_to_avoid, *lat_offset_to_avoid, is_collision_left, should_be_avoided,
      ref_points_for_obj_poly);
  }
  // prev_input_ref_path_points_ = input_ref_path_points;
}

void DynamicObstacleAvoidanceModule::determineWhetherToAvoidAgainstUnregulatedObjects(
  const std::vector<DynamicAvoidanceObject> & prev_objects)
{
  universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & input_path = getPreviousModuleOutput().path;
  const auto input_points = toGeometryPoints(input_path.points);  // for efficient computation

  const size_t ego_seg_idx = planner_data_->findEgoSegmentIndex(input_path.points);
  for (const auto & object : target_objects_manager_.getValidObjects()) {
    if (getObjectType(object.label) != ObjectType::UNREGULATED) {
      continue;
    }
    const size_t obj_seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(input_points, object.pose.position);

    const auto obj_uuid = object.uuid;
    const auto & ref_points_for_obj_poly = input_points;

    // 2.g. check if the ego is not ahead of the object.
    time_keeper_->start_track("getLateralLongitudinalOffset");
    const auto lat_lon_offset =
      getLateralLongitudinalOffset(input_points, object.pose, obj_seg_idx, object.shape);
    time_keeper_->end_track("getLateralLongitudinalOffset");

    const double signed_dist_ego_to_obj = [&]() {
      const double lon_offset_ego_to_obj = autoware::motion_utils::calcSignedArcLength(
        input_points, getEgoPose().position, ego_seg_idx, lat_lon_offset.nearest_idx);
      if (0 < lon_offset_ego_to_obj) {
        return std::max(
          0.0, lon_offset_ego_to_obj - planner_data_->parameters.front_overhang +
                 lat_lon_offset.min_lon_offset);
      }
      return std::min(
        0.0, lon_offset_ego_to_obj + planner_data_->parameters.rear_overhang +
               lat_lon_offset.max_lon_offset);
    }();
    if (signed_dist_ego_to_obj < 0) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since distance from ego to object (%f) is less "
        "than 0.",
        obj_uuid.c_str(), signed_dist_ego_to_obj);
      continue;
    }

    // 2.h. calculate longitudinal and lateral offset to avoid to generate object polygon by
    // "ego_path_base"
    const auto lat_offset_to_avoid = calcMinMaxLateralOffsetToAvoidUnregulatedObject(
      ref_points_for_obj_poly, getObstacleFromUuid(prev_objects, obj_uuid), object);
    if (!lat_offset_to_avoid) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since the object will intersects the ego's path "
        "enough",
        obj_uuid.c_str());
      continue;
    }

    const bool is_collision_left = (lat_offset_to_avoid.value().max_value > 0.0);
    const auto lon_offset_to_avoid = MinMaxValue{0.0, 1.0};  // not used. dummy value

    const bool should_be_avoided = true;
    target_objects_manager_.updateObjectVariables(
      obj_uuid, lon_offset_to_avoid, *lat_offset_to_avoid, is_collision_left, should_be_avoided,
      ref_points_for_obj_poly);
  }
}

LatFeasiblePaths DynamicObstacleAvoidanceModule::generateLateralFeasiblePaths(
  const geometry_msgs::msg::Pose & ego_pose, const double ego_vel) const
{
  const double lat_acc = parameters_->max_ego_lat_acc;
  const double lat_jerk = parameters_->max_ego_lat_jerk;
  const double delay_time = parameters_->delay_time_ego_shift;

  LatFeasiblePaths ego_lat_feasible_paths;
  for (double t = 0; t < 10.0; t += 1.0) {
    // maybe this equation does not have physical meaning (takagi)
    const double feasible_lat_offset = lat_acc * std::pow(std::max(t - delay_time, 0.0), 2) / 2.0 +
                                       lat_jerk * std::pow(std::max(t - delay_time, 0.0), 3) / 6.0 -
                                       planner_data_->parameters.vehicle_width / 2.0;
    const double x = t * ego_vel;
    const double y = feasible_lat_offset;

    const auto feasible_left_bound_point =
      autoware::universe_utils::calcOffsetPose(ego_pose, x, -y, 0.0).position;
    ego_lat_feasible_paths.left_path.push_back(feasible_left_bound_point);

    const auto feasible_right_bound_point =
      autoware::universe_utils::calcOffsetPose(ego_pose, x, y, 0.0).position;
    ego_lat_feasible_paths.right_path.push_back(feasible_right_bound_point);
  }

  autoware::universe_utils::appendMarkerArray(
    marker_utils::createPointsMarkerArray(
      ego_lat_feasible_paths.left_path, "ego_lat_feasible_left_path", 0, 0.6, 0.9, 0.9),
    &debug_marker_);
  autoware::universe_utils::appendMarkerArray(
    marker_utils::createPointsMarkerArray(
      ego_lat_feasible_paths.right_path, "ego_lat_feasible_right_path", 0, 0.6, 0.9, 0.9),
    &debug_marker_);

  return ego_lat_feasible_paths;
}

[[maybe_unused]] void DynamicObstacleAvoidanceModule::updateRefPathBeforeLaneChange(
  const std::vector<PathPointWithLaneId> & ego_ref_path_points)
{
  if (ref_path_before_lane_change_) {
    // check if the ego is close enough to the current ref path, meaning that lane change ends.
    const auto ego_pos = getEgoPose().position;
    const double dist_to_ref_path =
      std::abs(autoware::motion_utils::calcLateralOffset(ego_ref_path_points, ego_pos));

    constexpr double epsilon_dist_to_ref_path = 0.5;
    if (dist_to_ref_path < epsilon_dist_to_ref_path) {
      ref_path_before_lane_change_ = std::nullopt;
    }
  } else {
    // check if the ego is during lane change.
    if (prev_input_ref_path_points_ && !prev_input_ref_path_points_->empty()) {
      const double dist_ref_paths = std::abs(autoware::motion_utils::calcLateralOffset(
        ego_ref_path_points, prev_input_ref_path_points_->front().point.pose.position));
      constexpr double epsilon_ref_paths_diff = 1.0;
      if (epsilon_ref_paths_diff < dist_ref_paths) {
        ref_path_before_lane_change_ = *prev_input_ref_path_points_;
      }
    }
  }
}

[[maybe_unused]] std::optional<std::pair<size_t, size_t>>
DynamicObstacleAvoidanceModule::calcCollisionSection(
  const std::vector<PathPointWithLaneId> & ego_path, const PredictedPath & obj_path) const
{
  const size_t ego_idx = planner_data_->findEgoIndex(ego_path);
  const double ego_vel = getEgoSpeed();

  std::optional<size_t> collision_start_idx{std::nullopt};
  double lon_dist = 0.0;
  for (size_t i = ego_idx; i < ego_path.size() - 1; ++i) {
    lon_dist += autoware::universe_utils::calcDistance2d(ego_path.at(i), ego_path.at(i + 1));
    const double elapsed_time = lon_dist / ego_vel;

    const auto future_ego_pose = ego_path.at(i);
    const auto future_obj_pose =
      autoware::object_recognition_utils::calcInterpolatedPose(obj_path, elapsed_time);

    if (future_obj_pose) {
      const double dist_ego_to_obj =
        autoware::universe_utils::calcDistance2d(future_ego_pose, *future_obj_pose);
      if (dist_ego_to_obj < 1.0) {
        if (!collision_start_idx) {
          collision_start_idx = i;
        }
        continue;
      }
    } else {
      if (!collision_start_idx) {
        continue;
      }
    }

    return std::make_pair(*collision_start_idx, i - 1);
  }

  return std::make_pair(*collision_start_idx, ego_path.size() - 1);
}

TimeWhileCollision DynamicObstacleAvoidanceModule::calcTimeWhileCollision(
  const std::vector<PathPointWithLaneId> & ego_path, const double obj_tangent_vel,
  const LatLonOffset & lat_lon_offset) const
{
  // Set maximum time-to-collision 0 if the object longitudinally overlaps ego.
  // NOTE: This is to avoid objects running right beside ego even if time-to-collision is negative.
  const size_t ego_seg_idx = planner_data_->findEgoSegmentIndex(ego_path);
  const double lon_offset_ego_to_obj_idx = autoware::motion_utils::calcSignedArcLength(
    ego_path, getEgoPose().position, ego_seg_idx, lat_lon_offset.nearest_idx);
  const double relative_velocity = getEgoSpeed() - obj_tangent_vel;

  const double signed_time_to_start_collision = [&]() {
    const double lon_offset_ego_front_to_obj_back =
      lon_offset_ego_to_obj_idx + lat_lon_offset.min_lon_offset -
      (planner_data_->parameters.wheel_base + planner_data_->parameters.front_overhang);
    const double lon_offset_obj_front_to_ego_back = -lon_offset_ego_to_obj_idx -
                                                    lat_lon_offset.max_lon_offset -
                                                    planner_data_->parameters.rear_overhang;
    if (0.0 < lon_offset_ego_front_to_obj_back) {  // The object is ahead of the ego.
      return lon_offset_ego_front_to_obj_back / relative_velocity;
    } else if (0.0 < lon_offset_obj_front_to_ego_back) {  // The ego is ahead of the object.
      return lon_offset_obj_front_to_ego_back / -relative_velocity;
    }
    // The ego and object are colliding.
    return 0.0;
  }();
  const double signed_time_to_end_collision = [&]() {
    const double lon_offset_ego_back_to_obj_front = lon_offset_ego_to_obj_idx +
                                                    lat_lon_offset.max_lon_offset +
                                                    planner_data_->parameters.rear_overhang;
    const double lon_offset_obj_back_to_ego_front = -lon_offset_ego_to_obj_idx -
                                                    lat_lon_offset.min_lon_offset +
                                                    planner_data_->parameters.front_overhang;
    if (0.0 < relative_velocity) {
      return lon_offset_ego_back_to_obj_front / relative_velocity;
    }
    return lon_offset_obj_back_to_ego_front / -relative_velocity;
  }();

  // NOTE: In order to make time_to_start_collision continuous around the relative_velocity is zero.
  const double time_to_start_collision = [&]() {
    if (signed_time_to_start_collision < 0.0) {
      return std::numeric_limits<double>::max();
    }
    return signed_time_to_start_collision;
  }();
  const double time_to_end_collision = [&]() {
    if (signed_time_to_end_collision < 0.0) {
      return std::numeric_limits<double>::max();
    }
    return signed_time_to_end_collision;
  }();

  return TimeWhileCollision{time_to_start_collision, time_to_end_collision};
}

bool DynamicObstacleAvoidanceModule::isObjectFarFromPath(
  const PredictedObject & predicted_object, const double obj_dist_to_path) const
{
  const double obj_max_length = calcObstacleMaxLength(predicted_object.shape);
  const double min_obj_dist_to_path = std::max(
    0.0, obj_dist_to_path - planner_data_->parameters.vehicle_width / 2.0 - obj_max_length);

  return parameters_->max_obj_lat_offset_to_ego_path < min_obj_dist_to_path;
}

bool DynamicObstacleAvoidanceModule::willObjectCutIn(
  const std::vector<PathPointWithLaneId> & ego_path, const PredictedPath & predicted_path,
  const double obj_tangent_vel, const LatLonOffset & lat_lon_offset) const
{
  // Ignore oncoming object
  if (obj_tangent_vel < parameters_->min_cut_in_object_vel) {
    return false;
  }

  // Check if ego's path and object's path are close.
  const bool will_object_cut_in = [&]() {
    for (const auto & predicted_path_point : predicted_path.path) {
      const double paths_lat_diff =
        autoware::motion_utils::calcLateralOffset(ego_path, predicted_path_point.position);
      if (std::abs(paths_lat_diff) < planner_data_->parameters.vehicle_width / 2.0) {
        return true;
      }
    }
    return false;
  }();
  if (!will_object_cut_in) {
    // The object's path will not cut in
    return false;
  }

  // Ignore object longitudinally close to the ego
  const size_t ego_seg_idx = planner_data_->findEgoSegmentIndex(ego_path);
  const double relative_velocity = getEgoSpeed() - obj_tangent_vel;
  const double lon_offset_ego_to_obj =
    autoware::motion_utils::calcSignedArcLength(
      ego_path, getEgoPose().position, ego_seg_idx, lat_lon_offset.nearest_idx) +
    lat_lon_offset.min_lon_offset;
  if (
    lon_offset_ego_to_obj < std::max(
                              parameters_->min_lon_offset_ego_to_cut_in_object,
                              relative_velocity * parameters_->min_time_to_start_cut_in)) {
    return false;
  }

  return true;
}

DynamicObstacleAvoidanceModule::DecisionWithReason DynamicObstacleAvoidanceModule::willObjectCutOut(
  const double obj_tangent_vel, const double obj_normal_vel, const bool is_object_left,
  const std::optional<DynamicAvoidanceObject> & prev_object) const
{
  // Ignore oncoming object
  if (obj_tangent_vel < parameters_->min_cut_out_object_vel) {
    return DecisionWithReason{false};
  }

  // Check if previous object is memorized
  if (!prev_object || !prev_object->latest_time_inside_ego_path) {
    return DecisionWithReason{false};
  }
  if (
    parameters_->max_time_from_outside_ego_path_for_cut_out <
    (clock_->now() - *prev_object->latest_time_inside_ego_path).seconds()) {
    return DecisionWithReason{false};
  }

  // Check object's lateral velocity
  std::stringstream reason;
  reason << "since latest time inside ego's path is small enough ("
         << (clock_->now() - *prev_object->latest_time_inside_ego_path).seconds() << "<"
         << parameters_->max_time_from_outside_ego_path_for_cut_out << ")";
  if (is_object_left) {
    if (parameters_->min_cut_out_object_lat_vel < obj_normal_vel) {
      reason << ", and lateral velocity is large enough ("
             << parameters_->min_cut_out_object_lat_vel << "<abs(" << obj_normal_vel << ")";
      return DecisionWithReason{true, reason.str()};
    }
  } else {
    if (obj_normal_vel < -parameters_->min_cut_out_object_lat_vel) {
      reason << ", and lateral velocity is large enough ("
             << parameters_->min_cut_out_object_lat_vel << "<abs(" << obj_normal_vel << ")";
      return DecisionWithReason{true, reason.str()};
    }
  }

  return DecisionWithReason{false};
}

[[maybe_unused]] bool DynamicObstacleAvoidanceModule::willObjectBeOutsideEgoChangingPath(
  const geometry_msgs::msg::Pose & obj_pose, const autoware_perception_msgs::msg::Shape & obj_shape,
  const double obj_vel) const
{
  if (!ref_path_before_lane_change_ || obj_vel < 0.0) {
    return false;
  }

  // Check if object is in the lane before ego's lane change.
  const double dist_to_ref_path_before_lane_change = std::abs(
    autoware::motion_utils::calcLateralOffset(*ref_path_before_lane_change_, obj_pose.position));
  const double epsilon_dist_checking_in_lane = calcObstacleWidth(obj_shape);
  if (epsilon_dist_checking_in_lane < dist_to_ref_path_before_lane_change) {
    return false;
  }

  return true;
}

std::pair<lanelet::ConstLanelets, lanelet::ConstLanelets>
DynamicObstacleAvoidanceModule::getAdjacentLanes(
  const double forward_distance, const double backward_distance) const
{
  const auto & rh = planner_data_->route_handler;

  lanelet::ConstLanelet current_lane;
  if (!rh->getClosestLaneletWithinRoute(getEgoPose(), &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("dynamic_avoidance"),
      "failed to find closest lanelet within route!!!");
    return {};
  }

  const auto ego_succeeding_lanes =
    rh->getLaneletSequence(current_lane, getEgoPose(), backward_distance, forward_distance);

  lanelet::ConstLanelets right_lanes;
  lanelet::ConstLanelets left_lanes;
  for (const auto & lane : ego_succeeding_lanes) {
    // left lane
    const auto opt_left_lane = rh->getLeftLanelet(lane);
    if (opt_left_lane) {
      left_lanes.push_back(opt_left_lane.value());
    }

    // right lane
    const auto opt_right_lane = rh->getRightLanelet(lane);
    if (opt_right_lane) {
      right_lanes.push_back(opt_right_lane.value());
    }

    const auto right_opposite_lanes = rh->getRightOppositeLanelets(lane);
    if (!right_opposite_lanes.empty()) {
      right_lanes.push_back(right_opposite_lanes.front());
    }
  }

  return std::make_pair(right_lanes, left_lanes);
}

DynamicObstacleAvoidanceModule::LatLonOffset
DynamicObstacleAvoidanceModule::getLateralLongitudinalOffset(
  const std::vector<geometry_msgs::msg::Pose> & ego_points,
  const geometry_msgs::msg::Pose & obj_pose, const size_t obj_seg_idx,
  const autoware_perception_msgs::msg::Shape & obj_shape) const
{
  // TODO(murooka) calculation is not so accurate.
  std::vector<double> obj_lat_offset_vec;
  std::vector<double> obj_lon_offset_vec;
  if (obj_shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    // NOTE: efficient calculation for the CYLINDER object.
    const double radius = obj_shape.dimensions.x / 2.0;

    // calculate lateral offset
    const double obj_lat_offset =
      autoware::motion_utils::calcLateralOffset(ego_points, obj_pose.position, obj_seg_idx);
    double obj_max_lat_offset = obj_lat_offset + radius;
    if (obj_max_lat_offset * obj_lat_offset < 0) {
      obj_max_lat_offset = 0.0;
    }
    double obj_min_lat_offset = obj_lat_offset - radius;
    if (obj_min_lat_offset * obj_lat_offset < 0) {
      obj_min_lat_offset = 0.0;
    }

    obj_lat_offset_vec.push_back(obj_max_lat_offset);
    obj_lat_offset_vec.push_back(obj_min_lat_offset);

    // calculate longitudinal offset
    const double obj_lon_offset = autoware::motion_utils::calcLongitudinalOffsetToSegment(
      ego_points, obj_seg_idx, obj_pose.position);
    obj_lon_offset_vec.push_back(obj_lon_offset - radius);
    obj_lon_offset_vec.push_back(obj_lon_offset + radius);
  } else {
    const auto obj_points = autoware::universe_utils::toPolygon2d(obj_pose, obj_shape);
    for (size_t i = 0; i < obj_points.outer().size(); ++i) {
      const auto geom_obj_point = toGeometryPoint(obj_points.outer().at(i));
      const size_t obj_point_seg_idx =
        autoware::motion_utils::findNearestSegmentIndex(ego_points, geom_obj_point);

      // calculate lateral offset
      const double obj_point_lat_offset =
        autoware::motion_utils::calcLateralOffset(ego_points, geom_obj_point, obj_point_seg_idx);
      obj_lat_offset_vec.push_back(obj_point_lat_offset);

      // calculate longitudinal offset
      const double lon_offset = autoware::motion_utils::calcLongitudinalOffsetToSegment(
        ego_points, obj_seg_idx, geom_obj_point);
      obj_lon_offset_vec.push_back(lon_offset);
    }
  }

  const auto obj_lat_min_max_offset = getMinMaxValues(obj_lat_offset_vec);
  const auto obj_lon_min_max_offset = getMinMaxValues(obj_lon_offset_vec);

  return LatLonOffset{
    obj_seg_idx, obj_lat_min_max_offset.max_value, obj_lat_min_max_offset.min_value,
    obj_lon_min_max_offset.max_value, obj_lon_min_max_offset.min_value};
}

MinMaxValue DynamicObstacleAvoidanceModule::calcMinMaxLongitudinalOffsetToAvoid(
  const std::vector<geometry_msgs::msg::Pose> & ref_points_for_obj_poly,
  const geometry_msgs::msg::Pose & obj_pose, const Polygon2d & obj_points, const double obj_vel,
  const PredictedPath & obj_path, const autoware_perception_msgs::msg::Shape & obj_shape,
  const TimeWhileCollision & time_while_collision) const
{
  const size_t obj_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(ref_points_for_obj_poly, obj_pose.position);

  // calculate min/max longitudinal offset from object to path
  const auto obj_lon_offset = [&]() {
    std::vector<double> obj_lon_offset_vec;
    for (size_t i = 0; i < obj_points.outer().size(); ++i) {
      const auto geom_obj_point = toGeometryPoint(obj_points.outer().at(i));
      const double lon_offset = autoware::motion_utils::calcLongitudinalOffsetToSegment(
        ref_points_for_obj_poly, obj_seg_idx, geom_obj_point);
      obj_lon_offset_vec.push_back(lon_offset);
    }

    return getMinMaxValues(obj_lon_offset_vec);
  }();

  const double relative_velocity = getEgoSpeed() - obj_vel;

  // calculate bound start and end length
  const double start_length_to_avoid = [&]() {
    if (obj_vel < parameters_->max_stopped_object_vel) {
      // The ego and object are the opposite directional or the object is parked.
      return std::min(time_while_collision.time_to_start_collision, 3.5) * std::abs(obj_vel) +
             std::abs(relative_velocity) * parameters_->start_duration_to_avoid_oncoming_object;
    }
    // The ego and object are the same directional.
    const double obj_acc = -0.5;
    const double decel_time = 1.0;
    const double obj_moving_dist =
      (std::pow(std::max(obj_vel + obj_acc * decel_time, 0.0), 2) - std::pow(obj_vel, 2)) / 2 /
      obj_acc;
    const double ego_moving_dist = getEgoSpeed() * decel_time;
    return std::max(0.0, ego_moving_dist - obj_moving_dist) +
           std::abs(relative_velocity) * parameters_->start_duration_to_avoid_overtaking_object;
  }();
  const double end_length_to_avoid = [&]() {
    if (obj_vel < parameters_->max_stopped_object_vel) {
      // The ego and object are the opposite directional or the object is parked.
      return std::abs(relative_velocity) * parameters_->end_duration_to_avoid_oncoming_object;
    }
    // The ego and object are the same directional.
    return std::min(time_while_collision.time_to_end_collision, 3.0) * obj_vel +
           std::abs(relative_velocity) * parameters_->end_duration_to_avoid_overtaking_object;
  }();

  // calculate valid path for the forked object's path from the ego's path
  if (obj_vel < -parameters_->max_stopped_object_vel) {
    const bool is_object_same_direction = false;
    const double valid_length_to_avoid =
      calcValidLengthToAvoid(obj_path, obj_pose, obj_shape, is_object_same_direction);
    return MinMaxValue{
      obj_lon_offset.min_value + std::max(-start_length_to_avoid, -valid_length_to_avoid),
      obj_lon_offset.max_value + end_length_to_avoid};
  }
  if (parameters_->max_stopped_object_vel < obj_vel) {
    const bool is_object_same_direction = true;
    const double valid_length_to_avoid =
      calcValidLengthToAvoid(obj_path, obj_pose, obj_shape, is_object_same_direction);
    return MinMaxValue{
      obj_lon_offset.min_value - start_length_to_avoid,
      obj_lon_offset.max_value + std::min(end_length_to_avoid, valid_length_to_avoid)};
  }
  return MinMaxValue{
    obj_lon_offset.min_value - start_length_to_avoid,
    obj_lon_offset.max_value + end_length_to_avoid};
}

double DynamicObstacleAvoidanceModule::calcValidLengthToAvoid(
  const PredictedPath & obj_path, const geometry_msgs::msg::Pose & obj_pose,
  const autoware_perception_msgs::msg::Shape & obj_shape, const bool is_object_same_direction) const
{
  const auto & input_path_points = getPreviousModuleOutput().path.points;
  const size_t obj_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(input_path_points, obj_pose.position);

  constexpr double dist_threshold_additional_margin = 0.5;
  const double dist_threshold_paths =
    planner_data_->parameters.vehicle_width / 2.0 + parameters_->lat_offset_from_obstacle +
    calcObstacleWidth(obj_shape) / 2.0 + dist_threshold_additional_margin;

  // crop the ego's path by object position
  std::vector<PathPointWithLaneId> cropped_ego_path_points;
  if (is_object_same_direction) {
    cropped_ego_path_points = std::vector<PathPointWithLaneId>{
      input_path_points.begin() + obj_seg_idx, input_path_points.end()};
  } else {
    cropped_ego_path_points = std::vector<PathPointWithLaneId>{
      input_path_points.begin(), input_path_points.begin() + obj_seg_idx + 1 + 1};
    std::reverse(cropped_ego_path_points.begin(), cropped_ego_path_points.end());
  }
  if (cropped_ego_path_points.size() < 2) {
    return autoware::motion_utils::calcArcLength(obj_path.path);
  }

  // calculate where the object's path will be forked from (= far from) the ego's path.
  std::optional<size_t> last_nearest_ego_path_seg_idx{std::nullopt};
  const size_t valid_obj_path_end_idx = [&]() {
    size_t ego_path_seg_idx = 0;
    for (size_t obj_path_idx = 0; obj_path_idx < obj_path.path.size(); ++obj_path_idx) {
      bool are_paths_close{false};
      for (; ego_path_seg_idx < cropped_ego_path_points.size() - 1; ++ego_path_seg_idx) {
        const double dist_to_segment = calcDistanceToSegment(
          obj_path.path.at(obj_path_idx).position,
          cropped_ego_path_points.at(ego_path_seg_idx).point.pose.position,
          cropped_ego_path_points.at(ego_path_seg_idx + 1).point.pose.position);
        if (dist_to_segment < dist_threshold_paths) {
          last_nearest_ego_path_seg_idx = ego_path_seg_idx;
          are_paths_close = true;
          break;
        }
      }

      if (!are_paths_close) {
        return obj_path_idx;
      }
    }
    return obj_path.path.size() - 1;
  }();

  // calculate valid length to avoid
  if (last_nearest_ego_path_seg_idx && valid_obj_path_end_idx != obj_path.path.size() - 1) {
    const auto calc_min_dist = [&](const size_t arg_obj_path_idx) -> std::optional<double> {
      std::optional<double> min_dist{std::nullopt};
      for (size_t ego_path_seg_idx = *last_nearest_ego_path_seg_idx;
           ego_path_seg_idx < cropped_ego_path_points.size() - 1; ++ego_path_seg_idx) {
        const double dist_to_segment = calcDistanceToSegment(
          obj_path.path.at(arg_obj_path_idx).position,
          cropped_ego_path_points.at(ego_path_seg_idx).point.pose.position,
          cropped_ego_path_points.at(ego_path_seg_idx + 1).point.pose.position);
        if (!min_dist || dist_to_segment < *min_dist) {
          min_dist = dist_to_segment;
        }
        if (min_dist && *min_dist < dist_to_segment) {
          return *min_dist;
        }
      }
      return min_dist;
    };
    const size_t prev_valid_obj_path_end_idx =
      (valid_obj_path_end_idx == 0) ? valid_obj_path_end_idx : valid_obj_path_end_idx - 1;
    const size_t next_valid_obj_path_end_idx =
      (valid_obj_path_end_idx == 0) ? valid_obj_path_end_idx + 1 : valid_obj_path_end_idx;
    const auto prev_min_dist = calc_min_dist(prev_valid_obj_path_end_idx);
    const auto next_min_dist = calc_min_dist(next_valid_obj_path_end_idx);
    if (prev_min_dist && next_min_dist) {
      const double segment_length = autoware::universe_utils::calcDistance2d(
        obj_path.path.at(prev_valid_obj_path_end_idx),
        obj_path.path.at(next_valid_obj_path_end_idx));
      const double partial_segment_length = segment_length *
                                            (dist_threshold_paths - *prev_min_dist) /
                                            (*next_min_dist - *prev_min_dist);
      return autoware::motion_utils::calcSignedArcLength(
               obj_path.path, 0, prev_valid_obj_path_end_idx) +
             partial_segment_length;
    }
  }
  return autoware::motion_utils::calcSignedArcLength(obj_path.path, 0, valid_obj_path_end_idx);
}

// min value denotes near side, max value denotes far side
std::optional<MinMaxValue>
DynamicObstacleAvoidanceModule::calcMinMaxLateralOffsetToAvoidRegulatedObject(
  const std::vector<geometry_msgs::msg::Pose> & ref_points_for_obj_poly,
  const Polygon2d & obj_points, const geometry_msgs::msg::Point & obj_pos, const double obj_vel,
  const bool is_collision_left, const double obj_normal_vel,
  const std::optional<DynamicAvoidanceObject> & prev_object) const
{
  const bool enable_lowpass_filter = [&]() {
    if (
      !prev_object || prev_object->ref_points_for_obj_poly.size() < 2 ||
      ref_points_for_obj_poly.size() < 2) {
      return true;
    }
    const size_t obj_point_idx =
      autoware::motion_utils::findNearestIndex(ref_points_for_obj_poly, obj_pos);
    const double paths_lat_diff = std::abs(autoware::motion_utils::calcLateralOffset(
      prev_object->ref_points_for_obj_poly, ref_points_for_obj_poly.at(obj_point_idx).position));

    constexpr double min_paths_lat_diff = 0.3;
    if (paths_lat_diff < min_paths_lat_diff) {
      return true;
    }
    // NOTE: When the input reference path laterally changes, the low-pass filter is disabled not to
    // shift the obstacle polygon suddenly.
    return false;
  }();

  // calculate min/max lateral offset from object to path
  const auto obj_lat_abs_offset = [&]() {
    std::vector<double> obj_lat_abs_offset_vec;
    for (size_t i = 0; i < obj_points.outer().size(); ++i) {
      const auto geom_obj_point = toGeometryPoint(obj_points.outer().at(i));
      const size_t obj_point_seg_idx =
        autoware::motion_utils::findNearestSegmentIndex(ref_points_for_obj_poly, geom_obj_point);
      const double obj_point_lat_offset = autoware::motion_utils::calcLateralOffset(
        ref_points_for_obj_poly, geom_obj_point, obj_point_seg_idx);
      obj_lat_abs_offset_vec.push_back(obj_point_lat_offset);
    }
    return getMinMaxValues(obj_lat_abs_offset_vec);
  }();
  const double min_obj_lat_abs_offset = obj_lat_abs_offset.min_value;
  const double max_obj_lat_abs_offset = obj_lat_abs_offset.max_value;

  if (parameters_->min_front_object_vel < obj_vel) {
    const double obj_width_on_ego_path =
      std::min(max_obj_lat_abs_offset, planner_data_->parameters.vehicle_width / 2.0) -
      std::max(min_obj_lat_abs_offset, -planner_data_->parameters.vehicle_width / 2.0);
    if (
      planner_data_->parameters.vehicle_width *
        parameters_->max_front_object_ego_path_lat_cover_ratio <
      obj_width_on_ego_path) {
      return std::nullopt;
    }
  }

  // calculate bound min and max lateral offset
  const double min_bound_lat_offset = [&]() {
    const double lat_abs_offset_to_shift =
      std::max(0.0, obj_normal_vel * (is_collision_left ? -1.0 : 1.0)) *
      parameters_->max_time_for_lat_shift;
    const double raw_min_bound_lat_offset =
      (is_collision_left ? min_obj_lat_abs_offset : max_obj_lat_abs_offset) -
      (parameters_->lat_offset_from_obstacle + lat_abs_offset_to_shift) *
        (is_collision_left ? 1.0 : -1.0);
    const double min_bound_lat_abs_offset_limit =
      planner_data_->parameters.vehicle_width / 2.0 - parameters_->max_lat_offset_to_avoid;

    if (is_collision_left) {
      return std::max(raw_min_bound_lat_offset, min_bound_lat_abs_offset_limit);
    }
    return std::min(raw_min_bound_lat_offset, -min_bound_lat_abs_offset_limit);
  }();
  const double max_bound_lat_offset =
    (is_collision_left ? max_obj_lat_abs_offset : min_obj_lat_abs_offset) +
    (is_collision_left ? 1.0 : -1.0) * parameters_->lat_offset_from_obstacle;

  // filter min_bound_lat_offset
  const auto prev_min_lat_avoid_to_offset = [&]() -> std::optional<double> {
    if (!prev_object || !prev_object->lat_offset_to_avoid) {
      return std::nullopt;
    }
    return prev_object->lat_offset_to_avoid->min_value;
  }();
  const double filtered_min_bound_lat_offset =
    (prev_min_lat_avoid_to_offset.has_value() && enable_lowpass_filter)
      ? signal_processing::lowpassFilter(
          min_bound_lat_offset, *prev_min_lat_avoid_to_offset,
          parameters_->lpf_gain_for_lat_avoid_to_offset)
      : min_bound_lat_offset;

  return MinMaxValue{filtered_min_bound_lat_offset, max_bound_lat_offset};
}

// min value denotes near side, max value denotes far side
std::optional<MinMaxValue>
DynamicObstacleAvoidanceModule::calcMinMaxLateralOffsetToAvoidUnregulatedObject(
  const std::vector<geometry_msgs::msg::Pose> & ref_points_for_obj_poly,
  const std::optional<DynamicAvoidanceObject> & prev_object,
  const DynamicAvoidanceObject & object) const
{
  universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  time_keeper_->start_track("findNearestSegmentIndex of object position");
  const size_t obj_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(ref_points_for_obj_poly, object.pose.position);
  time_keeper_->end_track("findNearestSegmentIndex of object position");
  const size_t obj_point_idx =
    getNearestIndexFromSegmentIndex(ref_points_for_obj_poly, obj_seg_idx, object.pose.position);

  const bool enable_lowpass_filter = [&]() {
    universe_utils::ScopedTimeTrack st("calc enable_lowpass_filter", *time_keeper_);
    if (
      !prev_object || prev_object->ref_points_for_obj_poly.size() < 2 ||
      ref_points_for_obj_poly.size() < 2) {
      return true;
    }
    const double paths_lat_diff = std::abs(autoware::motion_utils::calcLateralOffset(
      prev_object->ref_points_for_obj_poly, ref_points_for_obj_poly.at(obj_point_idx).position));

    constexpr double min_paths_lat_diff = 0.3;
    if (paths_lat_diff < min_paths_lat_diff) {
      return true;
    }
    // NOTE: When the input reference path laterally changes, the low-pass filter is disabled not to
    // shift the obstacle polygon suddenly.
    return false;
  }();

  const auto obj_occupancy_region = [&]() {
    universe_utils::ScopedTimeTrack st("calc obj_occupancy_region", *time_keeper_);
    std::vector<double> lat_pos_vec;
    if (object.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
      // NOTE: efficient calculation for the CYLINDER object.
      const double radius = object.shape.dimensions.x / 2.0;

      const double obj_lat_offset = autoware::motion_utils::calcLateralOffset(
        ref_points_for_obj_poly, object.pose.position, obj_seg_idx);
      const double obj_min_lat_offset = [&]() {
        if (std::abs(obj_lat_offset) < radius) {
          return 0.0;
        }
        if (0.0 < obj_lat_offset) {
          return obj_lat_offset - radius;
        }
        return obj_lat_offset + radius;
      }();
      lat_pos_vec.push_back(obj_min_lat_offset);
    } else {
      const auto obj_points = autoware::universe_utils::toPolygon2d(object.pose, object.shape);
      for (size_t i = 0; i < obj_points.outer().size(); ++i) {
        const auto geom_obj_point = toGeometryPoint(obj_points.outer().at(i));
        const double obj_point_lat_offset = autoware::motion_utils::calcLateralOffset(
          ref_points_for_obj_poly, geom_obj_point,
          autoware::motion_utils::findNearestSegmentIndex(ref_points_for_obj_poly, geom_obj_point));
        lat_pos_vec.push_back(obj_point_lat_offset);
      }
    }
    const auto current_pos_area = getMinMaxValues(lat_pos_vec);
    return combineMinMaxValues(current_pos_area, current_pos_area + object.lat_vel * 3.0);
  }();

  if (
    obj_occupancy_region.min_value * obj_occupancy_region.max_value < 0.0 ||
    obj_occupancy_region.max_value - obj_occupancy_region.min_value < 1e-3) {
    return std::nullopt;
  }

  // calculate bound pos
  const auto bound_pos = [&]() {
    auto temp_bound_pos = obj_occupancy_region;
    temp_bound_pos.max_value += parameters_->lat_offset_from_obstacle;
    temp_bound_pos.min_value -= parameters_->lat_offset_from_obstacle;
    if (std::abs(temp_bound_pos.max_value) < std::abs(temp_bound_pos.min_value)) {
      temp_bound_pos.swap();  // From here, min denotes near bound, max denotes far bound.
    }

    const double near_bound_limit =
      planner_data_->parameters.vehicle_width / 2.0 - parameters_->max_lat_offset_to_avoid;
    if (temp_bound_pos.max_value > 0.0) {
      temp_bound_pos.min_value = std::max(temp_bound_pos.min_value, near_bound_limit);
    } else {
      temp_bound_pos.min_value = std::min(temp_bound_pos.min_value, -near_bound_limit);
    }
    return temp_bound_pos;
  }();

  // low pass filter for min_bound
  const auto prev_min_lat_avoid_to_offset = [&]() -> std::optional<double> {
    if (!prev_object || !prev_object->lat_offset_to_avoid) {
      return std::nullopt;
    }
    return prev_object->lat_offset_to_avoid->min_value;
  }();
  const double filtered_min_bound_pos =
    (prev_min_lat_avoid_to_offset.has_value() && enable_lowpass_filter)
      ? signal_processing::lowpassFilter(
          bound_pos.min_value, *prev_min_lat_avoid_to_offset,
          parameters_->lpf_gain_for_lat_avoid_to_offset)
      : bound_pos.min_value;

  return MinMaxValue{filtered_min_bound_pos, bound_pos.max_value};
}

// NOTE: object does not have const only to update min_bound_lat_offset.
std::optional<autoware::universe_utils::Polygon2d>
DynamicObstacleAvoidanceModule::calcEgoPathBasedDynamicObstaclePolygon(
  const DynamicAvoidanceObject & object) const
{
  if (!object.lon_offset_to_avoid || !object.lat_offset_to_avoid) {
    return std::nullopt;
  }

  auto ref_points_for_obj_poly = object.ref_points_for_obj_poly;

  const size_t obj_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(ref_points_for_obj_poly, object.pose.position);
  // const auto obj_points = autoware::universe_utils::toPolygon2d(object.pose, object.shape);

  const auto lon_bound_start_idx_opt = autoware::motion_utils::insertTargetPoint(
    obj_seg_idx, object.lon_offset_to_avoid->min_value, ref_points_for_obj_poly);
  const size_t updated_obj_seg_idx =
    (lon_bound_start_idx_opt && lon_bound_start_idx_opt.value() <= obj_seg_idx) ? obj_seg_idx + 1
                                                                                : obj_seg_idx;
  const auto lon_bound_end_idx_opt = autoware::motion_utils::insertTargetPoint(
    updated_obj_seg_idx, object.lon_offset_to_avoid->max_value, ref_points_for_obj_poly);

  if (!lon_bound_start_idx_opt && !lon_bound_end_idx_opt) {
    // NOTE: The obstacle is longitudinally out of the ego's trajectory.
    return std::nullopt;
  }
  const size_t lon_bound_start_idx =
    lon_bound_start_idx_opt ? lon_bound_start_idx_opt.value() : static_cast<size_t>(0);
  const size_t lon_bound_end_idx = lon_bound_end_idx_opt
                                     ? lon_bound_end_idx_opt.value()
                                     : static_cast<size_t>(ref_points_for_obj_poly.size() - 1);

  // create inner bound points
  std::vector<geometry_msgs::msg::Pose> obj_inner_bound_poses;
  for (size_t i = lon_bound_start_idx; i <= lon_bound_end_idx; ++i) {
    // NOTE: object.lat_offset_to_avoid->min_value is not the minimum value but the inner value.
    obj_inner_bound_poses.push_back(autoware::universe_utils::calcOffsetPose(
      ref_points_for_obj_poly.at(i), 0.0, object.lat_offset_to_avoid->min_value, 0.0));
  }

  // calculate start index laterally feasible for ego to shift considering maximum lateral jerk and
  // acceleration
  const auto obj_inner_bound_start_idx = [&]() -> std::optional<size_t> {
    const auto & ego_lat_feasible_path = object.is_collision_left
                                           ? object.ego_lat_feasible_paths.left_path
                                           : object.ego_lat_feasible_paths.right_path;
    const auto intersect_result = intersectLines(obj_inner_bound_poses, ego_lat_feasible_path);

    // Check if the object polygon intersects with the ego_lat_feasible_path.
    if (intersect_result) {
      const auto & [bound_seg_idx, intersect_point] = *intersect_result;
      const double lon_offset = autoware::universe_utils::calcDistance2d(
        obj_inner_bound_poses.at(bound_seg_idx), intersect_point);

      const auto obj_inner_bound_start_idx_opt =
        autoware::motion_utils::insertTargetPoint(bound_seg_idx, lon_offset, obj_inner_bound_poses);
      if (obj_inner_bound_start_idx_opt) {
        return *obj_inner_bound_start_idx_opt;
      }
    }

    // Check if the object polygon is fully outside the ego_lat_feasible_path.
    const double obj_poly_lat_offset = autoware::motion_utils::calcLateralOffset(
      ego_lat_feasible_path, obj_inner_bound_poses.front().position);
    if (
      (!object.is_collision_left && 0 < obj_poly_lat_offset) ||
      (object.is_collision_left && obj_poly_lat_offset < 0)) {
      return std::nullopt;
    }

    return 0;
  }();
  if (!obj_inner_bound_start_idx) {
    return std::nullopt;
  }

  // calculate feasible inner/outer bound points
  const auto feasible_obj_inner_bound_poses = std::vector<geometry_msgs::msg::Pose>(
    obj_inner_bound_poses.begin() + *obj_inner_bound_start_idx, obj_inner_bound_poses.end());
  const auto feasible_obj_inner_bound_points = convertToPoints(feasible_obj_inner_bound_poses);
  std::vector<geometry_msgs::msg::Point> feasible_obj_outer_bound_points;
  for (const auto & feasible_obj_inner_bound_pose : feasible_obj_inner_bound_poses) {
    feasible_obj_outer_bound_points.push_back(
      autoware::universe_utils::calcOffsetPose(
        feasible_obj_inner_bound_pose, 0.0,
        object.lat_offset_to_avoid->max_value - object.lat_offset_to_avoid->min_value, 0.0)
        .position);
  }

  // create obj_polygon from inner/outer bound points
  autoware::universe_utils::Polygon2d obj_poly;
  const auto add_points_to_obj_poly = [&](const auto & bound_points) {
    for (const auto & bound_point : bound_points) {
      obj_poly.outer().push_back(autoware::universe_utils::Point2d(bound_point.x, bound_point.y));
    }
  };
  add_points_to_obj_poly(feasible_obj_inner_bound_points);
  std::reverse(feasible_obj_outer_bound_points.begin(), feasible_obj_outer_bound_points.end());
  add_points_to_obj_poly(feasible_obj_outer_bound_points);

  boost::geometry::correct(obj_poly);
  return obj_poly;
}

// should be replace by the function calcPredictedPathBasedDynamicObstaclePolygon() (takagi)
std::optional<autoware::universe_utils::Polygon2d>
DynamicObstacleAvoidanceModule::calcObjectPathBasedDynamicObstaclePolygon(
  const DynamicAvoidanceObject & object) const
{
  const auto obj_path = *std::max_element(
    object.predicted_paths.begin(), object.predicted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

  // calculate left and right bound
  std::vector<geometry_msgs::msg::Point> obj_left_bound_points;
  std::vector<geometry_msgs::msg::Point> obj_right_bound_points;
  const double obj_path_length = autoware::motion_utils::calcArcLength(obj_path.path);
  for (size_t i = 0; i < obj_path.path.size(); ++i) {
    const double lon_offset = [&]() {
      if (i == 0)
        return -object.shape.dimensions.x / 2.0 -
               std::max(
                 parameters_->min_obj_path_based_lon_polygon_margin,
                 parameters_->lat_offset_from_obstacle);
      if (i == obj_path.path.size() - 1)
        return object.shape.dimensions.x / 2.0 +
               std::max(
                 parameters_->min_obj_path_based_lon_polygon_margin - obj_path_length,
                 parameters_->lat_offset_from_obstacle);
      return 0.0;
    }();

    const auto & obj_pose = obj_path.path.at(i);
    obj_left_bound_points.push_back(
      autoware::universe_utils::calcOffsetPose(
        obj_pose, lon_offset,
        object.shape.dimensions.y / 2.0 + parameters_->lat_offset_from_obstacle, 0.0)
        .position);
    obj_right_bound_points.push_back(
      autoware::universe_utils::calcOffsetPose(
        obj_pose, lon_offset,
        -object.shape.dimensions.y / 2.0 - parameters_->lat_offset_from_obstacle, 0.0)
        .position);
  }

  // create obj_polygon from inner/outer bound points
  autoware::universe_utils::Polygon2d obj_poly;
  for (const auto & bound_point : obj_right_bound_points) {
    const auto obj_poly_point = autoware::universe_utils::Point2d(bound_point.x, bound_point.y);
    obj_poly.outer().push_back(obj_poly_point);
  }
  std::reverse(obj_left_bound_points.begin(), obj_left_bound_points.end());
  for (const auto & bound_point : obj_left_bound_points) {
    const auto obj_poly_point = autoware::universe_utils::Point2d(bound_point.x, bound_point.y);
    obj_poly.outer().push_back(obj_poly_point);
  }

  boost::geometry::correct(obj_poly);
  return obj_poly;
}

// Calculate polygons according to predicted_path with certain confidence,
// except for the area required for ego safety.
// input: an object, the minimum area required for ego safety, and some global params
std::optional<autoware::universe_utils::Polygon2d>
DynamicObstacleAvoidanceModule::calcPredictedPathBasedDynamicObstaclePolygon(
  const DynamicAvoidanceObject & object, const EgoPathReservePoly & ego_path_poly) const
{
  std::vector<geometry_msgs::msg::Pose> obj_poses;
  for (const auto & predicted_path : object.predicted_paths) {
    if (predicted_path.confidence < parameters_->threshold_confidence) continue;

    double time_count = 0.0;
    for (const auto & pose : predicted_path.path) {
      obj_poses.push_back(pose);
      time_count += predicted_path.time_step.sec + predicted_path.time_step.nanosec * 1e-9;
      if (time_count > parameters_->end_time_to_consider + 1e-3) break;
    }
  }

  autoware::universe_utils::Polygon2d obj_points_as_poly;
  for (const auto pose : obj_poses) {
    boost::geometry::append(
      obj_points_as_poly, autoware::universe_utils::toFootprint(
                            pose, object.shape.dimensions.x * 0.5, object.shape.dimensions.x * 0.5,
                            object.shape.dimensions.y * 0.5)
                            .outer());
  }
  boost::geometry::correct(obj_points_as_poly);
  Polygon2d obj_poly;
  boost::geometry::convex_hull(obj_points_as_poly, obj_poly);

  autoware::universe_utils::MultiPolygon2d expanded_poly;
  namespace strategy = boost::geometry::strategy::buffer;
  boost::geometry::buffer(
    obj_poly, expanded_poly,
    strategy::distance_symmetric<double>(parameters_->margin_distance_around_pedestrian),
    strategy::side_straight(), strategy::join_round(), strategy::end_flat(),
    strategy::point_circle());
  if (expanded_poly.empty()) return {};

  autoware::universe_utils::MultiPolygon2d output_poly;
  boost::geometry::difference(
    expanded_poly[0],
    object.is_collision_left ? ego_path_poly.right_avoid : ego_path_poly.left_avoid, output_poly);

  if (output_poly.empty()) {
    RCLCPP_INFO_EXPRESSION(
      getLogger(), parameters_->enable_debug_info,
      "[DynamicAvoidance] Ignore obstacle (%s) because it stay inside the ego's path.",
      object.uuid.c_str());
    return {};
  } else if (output_poly.size() >= 2) {
    RCLCPP_INFO_EXPRESSION(
      getLogger(), parameters_->enable_debug_info,
      "[DynamicAvoidance] Ignore obstacle (%s) because it covers the ego's path.",
      object.uuid.c_str());
    return {};
  }

  return output_poly[0];
}

// Calculate the driving area required to ensure the safety of the own vehicle.
// It is assumed that this area will not be clipped.
// input: ego's reference path, ego's pose, ego's speed, and some global params
DynamicObstacleAvoidanceModule::EgoPathReservePoly
DynamicObstacleAvoidanceModule::calcEgoPathReservePoly(const PathWithLaneId & ego_path) const
{
  // This function require almost 0.5 ms. Should be refactored in the future
  namespace strategy = boost::geometry::strategy::buffer;

  assert(!ego_path.points.empty());

  autoware::universe_utils::LineString2d ego_path_lines;
  for (const auto & path_point : ego_path.points) {
    ego_path_lines.push_back(
      autoware::universe_utils::fromMsg(path_point.point.pose.position).to_2d());
  }

  auto calcReservePoly = [&ego_path_lines](
                           const strategy::distance_asymmetric<double> path_expand_strategy,
                           const strategy::distance_asymmetric<double> steer_expand_strategy,
                           const std::vector<geometry_msgs::msg::Point> & outer_body_path)
    -> autoware::universe_utils::Polygon2d {
    // reserve area based on the reference path
    autoware::universe_utils::MultiPolygon2d path_poly;
    boost::geometry::buffer(
      ego_path_lines, path_poly, path_expand_strategy, strategy::side_straight(),
      strategy::join_round(), strategy::end_flat(), strategy::point_circle());

    // reserve area steer to the avoidance path
    autoware::universe_utils::LineString2d steer_lines;
    for (const auto & point : outer_body_path) {
      const auto bg_point = autoware::universe_utils::fromMsg(point).to_2d();
      if (boost::geometry::within(bg_point, path_poly)) {
        if (steer_lines.size() != 0) {
          ;
          // boost::geometry::append(steer_lines, bg_point);
        }
        break;
      }
      // boost::geometry::append(steer_lines, bg_point);
    }
    autoware::universe_utils::MultiPolygon2d steer_poly;
    boost::geometry::buffer(
      steer_lines, steer_poly, steer_expand_strategy, strategy::side_straight(),
      strategy::join_round(), strategy::end_flat(), strategy::point_circle());

    autoware::universe_utils::MultiPolygon2d output_poly;
    boost::geometry::union_(path_poly, steer_poly, output_poly);
    if (output_poly.size() != 1) {
      assert(false);
    }
    return output_poly[0];
  };

  const auto motion_saturated_outer_paths =
    generateLateralFeasiblePaths(getEgoPose(), getEgoSpeed());

  const double vehicle_half_width = planner_data_->parameters.vehicle_width * 0.5;
  const double reserve_width_obj_side = vehicle_half_width - parameters_->max_lat_offset_to_avoid;

  const autoware::universe_utils::Polygon2d left_avoid_poly = calcReservePoly(
    strategy::distance_asymmetric<double>(vehicle_half_width, reserve_width_obj_side),
    strategy::distance_asymmetric<double>(vehicle_half_width, 0.0),
    motion_saturated_outer_paths.right_path);
  const autoware::universe_utils::Polygon2d right_avoid_poly = calcReservePoly(
    strategy::distance_asymmetric<double>(reserve_width_obj_side, vehicle_half_width),
    strategy::distance_asymmetric<double>(0.0, vehicle_half_width),
    motion_saturated_outer_paths.left_path);

  return {left_avoid_poly, right_avoid_poly};
}

lanelet::ConstLanelets DynamicObstacleAvoidanceModule::getCurrentLanesFromPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data)
{
  if (path.points.empty()) {
    throw std::logic_error("empty path.");
  }

  const auto idx = planner_data->findEgoIndex(path.points);

  if (path.points.at(idx).lane_ids.empty()) {
    throw std::logic_error("empty lane ids.");
  }

  const auto start_id = path.points.at(idx).lane_ids.front();
  const auto start_lane = planner_data->route_handler->getLaneletsFromId(start_id);
  const auto & p = planner_data->parameters;

  return planner_data->route_handler->getLaneletSequence(
    start_lane, p.backward_path_length, p.forward_path_length);
}

DrivableLanes DynamicObstacleAvoidanceModule::generateExpandedDrivableLanes(
  const lanelet::ConstLanelet & lanelet, const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<DynamicAvoidanceParameters> & parameters)
{
  const auto & route_handler = planner_data->route_handler;

  DrivableLanes current_drivable_lanes;
  current_drivable_lanes.left_lane = lanelet;
  current_drivable_lanes.right_lane = lanelet;

  if (parameters->use_lane_type == "current_lane") {
    return current_drivable_lanes;
  }

  const auto use_opposite_lane = parameters->use_lane_type == "opposite_direction_lane";

  // 1. get left/right side lanes
  const auto update_left_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
    const auto all_left_lanelets =
      route_handler->getAllLeftSharedLinestringLanelets(target_lane, use_opposite_lane, true);
    if (!all_left_lanelets.empty()) {
      current_drivable_lanes.left_lane = all_left_lanelets.back();  // leftmost lanelet
      pushUniqueVector(
        current_drivable_lanes.middle_lanes,
        lanelet::ConstLanelets(all_left_lanelets.begin(), all_left_lanelets.end() - 1));
    }
  };
  const auto update_right_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
    const auto all_right_lanelets =
      route_handler->getAllRightSharedLinestringLanelets(target_lane, use_opposite_lane, true);
    if (!all_right_lanelets.empty()) {
      current_drivable_lanes.right_lane = all_right_lanelets.back();  // rightmost lanelet
      pushUniqueVector(
        current_drivable_lanes.middle_lanes,
        lanelet::ConstLanelets(all_right_lanelets.begin(), all_right_lanelets.end() - 1));
    }
  };

  update_left_lanelets(lanelet);
  update_right_lanelets(lanelet);

  // 2.1 when there are multiple lanes whose previous lanelet is the same
  const auto get_next_lanes_from_same_previous_lane =
    [&route_handler](const lanelet::ConstLanelet & lane) {
      // get previous lane, and return false if previous lane does not exist
      lanelet::ConstLanelets prev_lanes;
      if (!route_handler->getPreviousLaneletsWithinRoute(lane, &prev_lanes)) {
        return lanelet::ConstLanelets{};
      }

      lanelet::ConstLanelets next_lanes;
      for (const auto & prev_lane : prev_lanes) {
        const auto next_lanes_from_prev = route_handler->getNextLanelets(prev_lane);
        pushUniqueVector(next_lanes, next_lanes_from_prev);
      }
      return next_lanes;
    };

  const auto next_lanes_for_right =
    get_next_lanes_from_same_previous_lane(current_drivable_lanes.right_lane);
  const auto next_lanes_for_left =
    get_next_lanes_from_same_previous_lane(current_drivable_lanes.left_lane);

  // 2.2 look for neighbor lane recursively, where end line of the lane is connected to end line
  // of the original lane
  const auto update_drivable_lanes =
    [&](const lanelet::ConstLanelets & next_lanes, const bool is_left) {
      for (const auto & next_lane : next_lanes) {
        const auto & edge_lane =
          is_left ? current_drivable_lanes.left_lane : current_drivable_lanes.right_lane;
        if (next_lane.id() == edge_lane.id()) {
          continue;
        }

        const auto & left_lane = is_left ? next_lane : edge_lane;
        const auto & right_lane = is_left ? edge_lane : next_lane;
        if (!isEndPointsConnected(left_lane, right_lane)) {
          continue;
        }

        if (is_left) {
          current_drivable_lanes.left_lane = next_lane;
        } else {
          current_drivable_lanes.right_lane = next_lane;
        }

        const auto & middle_lanes = current_drivable_lanes.middle_lanes;
        const auto has_same_lane = std::any_of(
          middle_lanes.begin(), middle_lanes.end(),
          [&edge_lane](const auto & lane) { return lane.id() == edge_lane.id(); });

        if (!has_same_lane) {
          if (is_left) {
            if (current_drivable_lanes.right_lane.id() != edge_lane.id()) {
              current_drivable_lanes.middle_lanes.push_back(edge_lane);
            }
          } else {
            if (current_drivable_lanes.left_lane.id() != edge_lane.id()) {
              current_drivable_lanes.middle_lanes.push_back(edge_lane);
            }
          }
        }

        return true;
      }
      return false;
    };

  const auto expand_drivable_area_recursively =
    [&](const lanelet::ConstLanelets & next_lanes, const bool is_left) {
      // NOTE: set max search num to avoid infinity loop for drivable area expansion
      constexpr size_t max_recursive_search_num = 3;
      for (size_t i = 0; i < max_recursive_search_num; ++i) {
        const bool is_update_kept = update_drivable_lanes(next_lanes, is_left);
        if (!is_update_kept) {
          break;
        }
        if (i == max_recursive_search_num - 1) {
          RCLCPP_DEBUG(
            rclcpp::get_logger(logger_namespace), "Drivable area expansion reaches max iteration.");
        }
      }
    };
  expand_drivable_area_recursively(next_lanes_for_right, false);
  expand_drivable_area_recursively(next_lanes_for_left, true);

  // 3. update again for new left/right lanes
  update_left_lanelets(current_drivable_lanes.left_lane);
  update_right_lanelets(current_drivable_lanes.right_lane);

  // 4. compensate that current_lane is in either of left_lane, right_lane or middle_lanes.
  if (
    current_drivable_lanes.left_lane.id() != lanelet.id() &&
    current_drivable_lanes.right_lane.id() != lanelet.id()) {
    current_drivable_lanes.middle_lanes.push_back(lanelet);
  }

  return current_drivable_lanes;
}
}  // namespace autoware::behavior_path_planner
