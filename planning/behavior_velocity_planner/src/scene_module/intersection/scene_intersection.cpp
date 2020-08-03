/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <scene_module/intersection/scene_intersection.h>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_extension/regulatory_elements/road_marking.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>

#include "scene_module/intersection/util.h"
#include "utilization/boost_geometry_helper.h"
#include "utilization/interpolate.h"
#include "utilization/util.h"

namespace bg = boost::geometry;

IntersectionModule::IntersectionModule(
  const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param)
: SceneModuleInterface(module_id), lane_id_(lane_id)
{
  planner_param_ = planner_param;
  const auto & assigned_lanelet = planner_data->lanelet_map->laneletLayer.get(lane_id);
  turn_direction_ = assigned_lanelet.attributeOr("turn_direction", "else");
  has_traffic_light_ =
    !(assigned_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>().empty());
}

bool IntersectionModule::modifyPathVelocity(
  autoware_planning_msgs::PathWithLaneId * path, autoware_planning_msgs::StopReason * stop_reason)
{
  debug_data_ = {};
  *stop_reason =
    planning_utils::initializeStopReason(autoware_planning_msgs::StopReason::INTERSECTION);

  const auto input_path = *path;
  debug_data_.path_raw = input_path;

  State current_state = state_machine_.getState();
  ROS_DEBUG("[Intersection] lane_id = %ld, state = %d", lane_id_, static_cast<int>(current_state));

  /* get current pose */
  geometry_msgs::PoseStamped current_pose = planner_data_->current_pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  /* get detection area */
  std::vector<lanelet::CompoundPolygon3d> detection_areas;
  util::getObjectivePolygons(
    lanelet_map_ptr, routing_graph_ptr, lane_id_, planner_param_, &detection_areas);
  if (detection_areas.empty()) {
    ROS_DEBUG("[Intersection] no detection area. skip computation.");
    return true;
  }
  debug_data_.detection_area = detection_areas;

  /* set stop-line and stop-judgement-line for base_link */
  int stop_line_idx = -1;
  int pass_judge_line_idx = -1;
  int first_idx_inside_lane = -1;
  if (!util::generateStopLine(
        lane_id_, detection_areas, planner_data_, planner_param_, path, &stop_line_idx,
        &pass_judge_line_idx, &first_idx_inside_lane)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[IntersectionModule::run] setStopLineIdx fail");
    return false;
  }

  if (stop_line_idx <= 0 || pass_judge_line_idx <= 0) {
    ROS_DEBUG("[Intersection] stop line or pass judge line is at path[0], ignore planning.");
    return true;
  }

  /* calc closest index */
  int closest_idx = -1;
  if (!planning_utils::calcClosestIndex(input_path, current_pose.pose, closest_idx)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[Intersection] calcClosestIndex fail");
    return false;
  }

  debug_data_.virtual_wall_pose =
    util::getAheadPose(stop_line_idx, planner_data_->base_link2front, *path);
  debug_data_.stop_point_pose = path->points.at(stop_line_idx).point.pose;
  debug_data_.judge_point_pose = path->points.at(pass_judge_line_idx).point.pose;

  /* if current_state = GO, and current_pose is in front of stop_line, ignore planning. */
  bool is_over_pass_judge_line = static_cast<bool>(closest_idx > pass_judge_line_idx);
  if (closest_idx == pass_judge_line_idx) {
    geometry_msgs::Pose pass_judge_line = path->points.at(pass_judge_line_idx).point.pose;
    is_over_pass_judge_line = util::isAheadOf(current_pose.pose, pass_judge_line);
  }
  if (current_state == State::GO && is_over_pass_judge_line) {
    ROS_DEBUG("[Intersection] over the pass judge line. no plan needed.");
    return true;  // no plan needed.
  }

  /* get dynamic object */
  const auto objects_ptr = planner_data_->dynamic_objects;

  /* calculate dynamic collision around detection area */
  bool has_collision = checkCollision(*path, detection_areas, objects_ptr, closest_idx);
  bool is_stuck = checkStuckVehicleInIntersection(*path, closest_idx, objects_ptr);
  bool is_entry_prohibited = (has_collision || is_stuck);
  state_machine_.setStateWithMarginTime(is_entry_prohibited ? State::STOP : State::GO);

  /* set stop speed : TODO behavior on straight lane should be improved*/
  if (state_machine_.getState() == State::STOP) {
    constexpr double stop_vel = 0.0;
    const double decel_vel = planner_param_.decel_velocoity;
    double v =
      (!is_stuck && has_traffic_light_ && turn_direction_ == "straight") ? decel_vel : stop_vel;
    util::setVelocityFrom(stop_line_idx, v, path);

    /* get stop point and stop factor */
    autoware_planning_msgs::StopFactor stop_factor;
    stop_factor.stop_pose = debug_data_.stop_point_pose;
    const auto stop_factor_conflict = planning_utils::toRosPoints(debug_data_.conflicting_targets);
    const auto stop_factor_stuck = planning_utils::toRosPoints(debug_data_.stuck_targets);
    stop_factor.stop_factor_points =
      planning_utils::concatVector(stop_factor_conflict, stop_factor_stuck);
    planning_utils::appendStopReason(stop_factor, stop_reason);
  }

  return true;
}

void IntersectionModule::cutPredictPathWithDuration(
  autoware_perception_msgs::DynamicObjectArray * objects_ptr, const double time_thr) const
{
  const ros::Time current_time = ros::Time::now();
  for (auto & object : objects_ptr->objects) {                    // each objects
    for (auto & predicted_path : object.state.predicted_paths) {  // each predicted paths
      std::vector<geometry_msgs::PoseWithCovarianceStamped> vp;
      for (auto & predicted_pose : predicted_path.path) {  // each path points
        if ((predicted_pose.header.stamp - current_time).toSec() < time_thr) {
          vp.push_back(predicted_pose);
        }
      }
      predicted_path.path = vp;
    }
  }
}

bool IntersectionModule::checkCollision(
  const autoware_planning_msgs::PathWithLaneId & path,
  const std::vector<lanelet::CompoundPolygon3d> & detection_areas,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr, const int closest_idx)
{
  /* generate ego-lane polygon */
  const Polygon2d ego_poly =
    generateEgoIntersectionLanePolygon(path, closest_idx, 0.0);  // TODO use Lanelet
  debug_data_.ego_lane_polygon = toGeomMsg(ego_poly);

  /* extruct target objects */
  autoware_perception_msgs::DynamicObjectArray target_objects;
  for (const auto & object : objects_ptr->objects) {
    // ignore non-vehicle type objects, such as pedestrian.
    if (!isTargetVehicleType(object)) continue;

    // ignore vehicle in ego-lane. (TODO update check algorithm)
    const auto object_pose = object.state.pose_covariance.pose;
    const bool is_in_ego_lane = bg::within(to_bg2d(object_pose.position), ego_poly);
    if (is_in_ego_lane) {
      continue;  // TODO(Kenji Miyake): check direction?
    }

    // keep vehicle in detection_area
    Polygon2d obj_poly;
    if (object.shape.type == autoware_perception_msgs::Shape::POLYGON) {
      obj_poly = toBoostPoly(object.shape.footprint);
    } else {
      // cylinder type is treated as square-polygon
      obj_poly = obj2polygon(object_pose, object.shape.dimensions);
    }

    for (const auto & detection_area : detection_areas) {
      const auto detection_poly = lanelet::utils::to2D(detection_area).basicPolygon();
      const bool is_in_objective_lanelet =
        !boost::geometry::disjoint(obj_poly, toBoostPoly(detection_poly));
      if (is_in_objective_lanelet) {
        target_objects.objects.push_back(object);
        break;
      }
    }
  }

  /* check collision between target_objects predicted path and ego lane */

  // cut the predicted path at passing_time
  const double passing_time = calcIntersectionPassingTime(path, closest_idx, lane_id_);
  cutPredictPathWithDuration(&target_objects, passing_time);

  // check collision between predicted_path and ego_area
  bool collision_detected = false;
  for (const auto & object : target_objects.objects) {
    bool has_collision = false;
    for (const auto & predicted_path : object.state.predicted_paths) {
      has_collision = bg::intersects(ego_poly, to_bg2d(predicted_path.path));
      if (has_collision) {
        collision_detected = true;
        debug_data_.conflicting_targets.objects.push_back(object);
        break;
      }
    }
  }

  return collision_detected;
}

Polygon2d IntersectionModule::generateEgoIntersectionLanePolygon(
  const autoware_planning_msgs::PathWithLaneId & path, const int closest_idx,
  const double extra_dist) const
{
  size_t assigned_lane_end_idx = 0;
  bool has_assigned_lane_id_prev = false;
  for (size_t i = 0; i < path.points.size(); ++i) {
    bool has_assigned_lane_id = util::hasLaneId(path.points.at(i), lane_id_);
    if (has_assigned_lane_id_prev && !has_assigned_lane_id) {
      assigned_lane_end_idx = i;
      break;
    }
    has_assigned_lane_id_prev = has_assigned_lane_id;
  }

  size_t ego_area_end_idx = assigned_lane_end_idx;
  double dist_sum = 0.0;
  for (size_t i = assigned_lane_end_idx + 1; i < path.points.size(); ++i) {
    dist_sum += planning_utils::calcDist2d(path.points.at(i), path.points.at(i - 1));
    if (dist_sum > extra_dist) break;
    ++ego_area_end_idx;
  }

  Polygon2d ego_area;  // open polygon
  const auto width = planner_param_.path_expand_width;
  for (int i = closest_idx; i <= ego_area_end_idx; ++i) {
    double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    double x = path.points.at(i).point.pose.position.x + width * std::sin(yaw);
    double y = path.points.at(i).point.pose.position.y - width * std::cos(yaw);
    ego_area.outer().push_back(Point2d(x, y));
  }
  for (int i = ego_area_end_idx; i >= closest_idx; --i) {
    double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    double x = path.points.at(i).point.pose.position.x - width * std::sin(yaw);
    double y = path.points.at(i).point.pose.position.y + width * std::cos(yaw);
    ego_area.outer().push_back(Point2d(x, y));
  }

  return ego_area;
}

double IntersectionModule::calcIntersectionPassingTime(
  const autoware_planning_msgs::PathWithLaneId & path, const int closest_idx,
  const int objective_lane_id) const
{
  double dist_sum = 0.0;
  int assigned_lane_found = false;

  for (int i = closest_idx + 1; i < path.points.size(); ++i) {
    dist_sum += planning_utils::calcDist2d(path.points.at(i - 1), path.points.at(i));
    bool has_objective_lane_id = util::hasLaneId(path.points.at(i), objective_lane_id);

    if (assigned_lane_found && !has_objective_lane_id) {
      break;
    }
    assigned_lane_found = has_objective_lane_id;
  }
  if (!assigned_lane_found) return 0.0;  // has already passed the intersection.

  // TODO set to be reasonable
  const double passing_time = dist_sum / planner_param_.intersection_velocity;

  ROS_DEBUG("[intersection] intersection dist = %f, passing_time = %f", dist_sum, passing_time);

  return passing_time;
}

bool IntersectionModule::checkStuckVehicleInIntersection(
  const autoware_planning_msgs::PathWithLaneId & path, const int closest_idx,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr) const
{
  const Polygon2d stuck_vehicle_detect_area =
    generateEgoIntersectionLanePolygon(path, closest_idx, planner_param_.stuck_vehicle_detect_dist);
  debug_data_.stuck_vehicle_detect_area = toGeomMsg(stuck_vehicle_detect_area);

  for (const auto & object : objects_ptr->objects) {
    if (!isTargetVehicleType(object)) {
      continue;  // not target vehicle type
    }
    const auto obj_v = std::fabs(object.state.twist_covariance.twist.linear.x);
    if (obj_v > planner_param_.stuck_vehicle_vel_thr) {
      continue;  // not stop vehicle
    }
    const auto object_pos = object.state.pose_covariance.pose.position;
    if (bg::within(to_bg2d(object_pos), stuck_vehicle_detect_area)) {
      ROS_DEBUG("[intersection] stuck vehicle found.");
      debug_data_.stuck_targets.objects.push_back(object);
      return true;
    }
  }
  return false;
}

bool IntersectionModule::isTargetVehicleType(
  const autoware_perception_msgs::DynamicObject & object) const
{
  if (
    object.semantic.type == autoware_perception_msgs::Semantic::CAR ||
    object.semantic.type == autoware_perception_msgs::Semantic::BUS ||
    object.semantic.type == autoware_perception_msgs::Semantic::TRUCK ||
    object.semantic.type == autoware_perception_msgs::Semantic::MOTORBIKE ||
    object.semantic.type == autoware_perception_msgs::Semantic::BICYCLE) {
    return true;
  }
  return false;
}

void IntersectionModule::StateMachine::setStateWithMarginTime(State state)
{
  /* same state request */
  if (state_ == state) {
    start_time_ = nullptr;  // reset timer
    return;
  }

  /* GO -> STOP */
  if (state == State::STOP) {
    state_ = State::STOP;
    start_time_ = nullptr;  // reset timer
    return;
  }

  /* STOP -> GO */
  if (state == State::GO) {
    if (start_time_ == nullptr) {
      start_time_ = std::make_shared<ros::Time>(ros::Time::now());
    } else {
      const double duration = (ros::Time::now() - *start_time_).toSec();
      if (duration > margin_time_) {
        state_ = State::GO;
        start_time_ = nullptr;  // reset timer
      }
    }
    return;
  }

  ROS_ERROR("[StateMachine] : Unsuitable state. ignore request.");
  return;
}

void IntersectionModule::StateMachine::setState(State state) { state_ = state; }

void IntersectionModule::StateMachine::setMarginTime(const double t) { margin_time_ = t; }

IntersectionModule::State IntersectionModule::StateMachine::getState() { return state_; }
