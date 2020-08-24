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
#include <scene_module/blind_spot/scene.h>
#include <boost/geometry/algorithms/distance.hpp>

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

BlindSpotModule::BlindSpotModule(
  const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param)
: SceneModuleInterface(module_id), lane_id_(lane_id), turn_direction_(TurnDirection::INVALID)
{
  planner_param_ = planner_param;
  const auto & assigned_lanelet = planner_data->lanelet_map->laneletLayer.get(lane_id);
  const std::string turn_direction = assigned_lanelet.attributeOr("turn_direction", "else");
  if (!turn_direction.compare("left")) {
    turn_direction_ = TurnDirection::LEFT;
  } else if (!turn_direction.compare("right")) {
    turn_direction_ = TurnDirection::RIGHT;
  };
  has_traffic_light_ =
    !(assigned_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>().empty());
}

bool BlindSpotModule::modifyPathVelocity(
  autoware_planning_msgs::PathWithLaneId * path, autoware_planning_msgs::StopReason * stop_reason)
{
  debug_data_ = {};
  *stop_reason =
    planning_utils::initializeStopReason(autoware_planning_msgs::StopReason::BLIND_SPOT);

  const auto input_path = *path;
  debug_data_.path_raw = input_path;

  State current_state = state_machine_.getState();
  ROS_DEBUG("[Blind Spot] lane_id = %ld, state = %s", lane_id_, toString(current_state).c_str());

  /* get current pose */
  geometry_msgs::PoseStamped current_pose = planner_data_->current_pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  /* set stop-line and stop-judgement-line for base_link */
  int stop_line_idx = -1;
  int pass_judge_line_idx = -1;
  const auto straight_lanelets = getStraightLanelets(lanelet_map_ptr, routing_graph_ptr, lane_id_);
  if (!generateStopLine(straight_lanelets, path, &stop_line_idx, &pass_judge_line_idx)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[BlindSpotModule::run] setStopLineIdx fail");
    return false;
  }

  if (stop_line_idx <= 0 || pass_judge_line_idx <= 0) {
    ROS_DEBUG("[Blind Spot] stop line or pass judge line is at path[0], ignore planning.");
    return true;
  }

  /* calc closest index */
  int closest_idx = -1;
  if (!planning_utils::calcClosestIndex(input_path, current_pose.pose, closest_idx)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[Blind Spot] calcClosestIndex fail");
    return false;
  }

  /* get debug info */
  debug_data_.virtual_wall_pose =
    util::getAheadPose(stop_line_idx, planner_data_->base_link2front, *path);
  debug_data_.stop_point_pose = path->points.at(stop_line_idx).point.pose;
  debug_data_.judge_point_pose = path->points.at(pass_judge_line_idx).point.pose;

  /* if current_state = GO, and current_pose is over judge_line, ignore planning. */
  bool is_over_pass_judge_line = static_cast<bool>(closest_idx > pass_judge_line_idx);
  if (closest_idx == pass_judge_line_idx) {
    geometry_msgs::Pose pass_judge_line = path->points.at(pass_judge_line_idx).point.pose;
    is_over_pass_judge_line = util::isAheadOf(current_pose.pose, pass_judge_line);
  }
  if (current_state == State::GO && is_over_pass_judge_line) {
    ROS_DEBUG("[Blind Spot] over the pass judge line. no plan needed.");
    return true;  // no plan needed.
  }

  /* get dynamic object */
  const auto objects_ptr = planner_data_->dynamic_objects;

  /* calculate dynamic collision around detection area */
  bool has_obstacle =
    checkObstacleInBlindSpot(lanelet_map_ptr, routing_graph_ptr, *path, objects_ptr, closest_idx);
  state_machine_.setStateWithMarginTime(has_obstacle ? State::STOP : State::GO);

  /* set stop speed */
  if (state_machine_.getState() == State::STOP) {
    constexpr double stop_vel = 0.0;
    util::setVelocityFrom(stop_line_idx, stop_vel, path);

    /* get stop point and stop factor */
    autoware_planning_msgs::StopFactor stop_factor;
    stop_factor.stop_pose = debug_data_.stop_point_pose;
    stop_factor.stop_factor_points = planning_utils::toRosPoints(debug_data_.conflicting_targets);
    planning_utils::appendStopReason(stop_factor, stop_reason);
  }

  return true;
}

boost::optional<int> BlindSpotModule::getFirstPointConflictingLanelets(
  const autoware_planning_msgs::PathWithLaneId & path,
  const lanelet::ConstLanelets & lanelets) const
{
  using namespace lanelet::utils;
  int first_idx_conflicting_lanelets = path.points.size() - 1;
  bool is_conflict = false;
  for (const auto & ll : lanelets) {
    const auto line = (turn_direction_ == TurnDirection::LEFT) ? ll.leftBound() : ll.rightBound();
    for (size_t i = 0; i < path.points.size(); ++i) {
      const auto vehicle_edge = getVehicleEdge(
        path.points.at(i).point.pose, planner_data_->vehicle_width, planner_data_->base_link2front);
      if (bg::intersects(toHybrid(to2D(line)), toHybrid(vehicle_edge))) {
        first_idx_conflicting_lanelets =
          std::min(first_idx_conflicting_lanelets, static_cast<int>(i));
        is_conflict = true;
        break;
      }
    }
  }
  if (is_conflict) {
    return first_idx_conflicting_lanelets;
  } else {
    return boost::none;
  }
}

bool BlindSpotModule::generateStopLine(
  const lanelet::ConstLanelets straight_lanelets, autoware_planning_msgs::PathWithLaneId * path,
  int * stop_line_idx, int * pass_judge_line_idx) const
{
  /* set judge line dist */
  const double current_vel = planner_data_->current_velocity->twist.linear.x;
  const double max_acc = planner_data_->max_stop_acceleration_threshold_;
  const double pass_judge_line_dist = planning_utils::calcJudgeLineDist(current_vel);

  /* set parameters */
  constexpr double interval = 0.2;
  const int margin_idx_dist = std::ceil(planner_param_.stop_line_margin / interval);
  const int base2front_idx_dist = std::ceil(planner_data_->base_link2front / interval);
  const int pass_judge_idx_dist = std::ceil(pass_judge_line_dist / interval);

  /* spline interpolation */
  autoware_planning_msgs::PathWithLaneId path_ip;
  if (!util::splineInterpolate(*path, interval, &path_ip)) return false;
  debug_data_.spline_path = path_ip;

  /* generate stop point */
  int stop_idx_ip;  // stop point index for interpolated path.
  if (straight_lanelets.size() > 0) {
    boost::optional<int> first_idx_conflicting_lane_opt =
      getFirstPointConflictingLanelets(path_ip, straight_lanelets);
    if (!first_idx_conflicting_lane_opt) {
      ROS_DEBUG("No conflicting line found.");
      return false;
    }
    stop_idx_ip =
      std::max(first_idx_conflicting_lane_opt.get() - 1 - margin_idx_dist - base2front_idx_dist, 0);
  } else {
    boost::optional<geometry_msgs::Point> intersection_enter_point_opt =
      getStartPointFromLaneLet(lane_id_);
    if (!intersection_enter_point_opt) {
      ROS_DEBUG("No intersection enter point found.");
      return false;
    }
    planning_utils::calcClosestIndex(
      path_ip, intersection_enter_point_opt.get(), stop_idx_ip, 10.0);
    stop_idx_ip = std::max(stop_idx_ip - base2front_idx_dist, 0);
  }

  /* insert stop_point */
  const auto inserted_stop_point = path_ip.points.at(stop_idx_ip).point.pose;
  *stop_line_idx = util::insertPoint(inserted_stop_point, path);

  /* if another stop point exist before intersection stop_line, disable judge_line. */
  bool has_prior_stopline = false;
  for (int i = 0; i < *stop_line_idx; ++i) {
    if (std::fabs(path->points.at(i).point.twist.linear.x) < 0.1) {
      has_prior_stopline = true;
      break;
    }
  }

  /* insert judge point */
  const int pass_judge_idx_ip = std::max(stop_idx_ip - pass_judge_idx_dist, 0);
  if (has_prior_stopline || stop_idx_ip == pass_judge_idx_ip) {
    *pass_judge_line_idx = *stop_line_idx;
  } else {
    const auto inserted_pass_judge_point = path_ip.points.at(pass_judge_idx_ip).point.pose;
    *pass_judge_line_idx = util::insertPoint(inserted_pass_judge_point, path);
    ++(*stop_line_idx);  // stop index is incremented by judge line insertion
  }

  ROS_DEBUG(
    "[Blind Spot] generateStopLine() : stop_idx = %d, pass_judge_idx = %d, stop_idx_ip = %d, "
    "pass_judge_idx_ip = %d, has_prior_stopline = %d",
    *stop_line_idx, *pass_judge_line_idx, stop_idx_ip, pass_judge_idx_ip, has_prior_stopline);

  return true;
}

void BlindSpotModule::cutPredictPathWithDuration(
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

bool BlindSpotModule::checkObstacleInBlindSpot(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const autoware_planning_msgs::PathWithLaneId & path,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr,
  const int closest_idx) const
{
  /* get detection area */
  if (turn_direction_ == TurnDirection::INVALID) {
    ROS_WARN("blind spot detector is running, turn_direction_ = not right or left.");
    return false;
  }

  const auto areas =
    generateBlindSpotPolygons(lanelet_map_ptr, routing_graph_ptr, path, closest_idx);
  debug_data_.detection_area_for_blind_spot = areas.detection_area;
  debug_data_.confict_area_for_blind_spot = areas.conflict_area;

  autoware_perception_msgs::DynamicObjectArray objects = *objects_ptr;
  cutPredictPathWithDuration(&objects, planner_param_.max_future_movement_time);

  // check objects in blind spot areas
  bool obstacle_detected = false;
  for (const auto & object : objects.objects) {
    if (!isTargetObjectType(object)) continue;

    bool exist_in_detection_area = bg::within(
      to_bg2d(object.state.pose_covariance.pose.position),
      lanelet::utils::to2D(areas.detection_area));
    bool exist_in_conflict_area = isPredictedPathInArea(object, areas.conflict_area);
    if (exist_in_detection_area && exist_in_conflict_area) {
      obstacle_detected = true;
      debug_data_.conflicting_targets.objects.push_back(object);
    }
  }
  return obstacle_detected;
}

bool BlindSpotModule::isPredictedPathInArea(
  const autoware_perception_msgs::DynamicObject & object,
  const lanelet::CompoundPolygon3d & area) const
{
  bool exist_in_conflict_area = false;
  for (const auto & predicted_path : object.state.predicted_paths) {
    for (const auto & predicted_point : predicted_path.path) {
      exist_in_conflict_area =
        bg::within(to_bg2d(predicted_point.pose.pose.position), lanelet::utils::to2D(area));
      if (exist_in_conflict_area) return true;
    }
  }
  return false;
}

lanelet::ConstLanelet BlindSpotModule::generateHalfLanelet(
  const lanelet::ConstLanelet lanelet) const
{
  lanelet::Points3d lefts, rights;

  const auto original_left_bound =
    (turn_direction_ == TurnDirection::LEFT) ? lanelet.leftBound() : lanelet.centerline();
  const auto original_right_bound =
    (turn_direction_ == TurnDirection::LEFT) ? lanelet.centerline() : lanelet.rightBound();

  for (const auto & pt : original_left_bound) {
    lefts.push_back(lanelet::Point3d(pt));
  }
  for (const auto & pt : original_right_bound) {
    rights.push_back(lanelet::Point3d(pt));
  }
  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, lefts);
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, rights);
  auto half_lanelet = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  const auto centerline = lanelet::utils::generateFineCenterline(half_lanelet, 5.0);
  half_lanelet.setCenterline(centerline);
  return half_lanelet;
}

BlindSpotPolygons BlindSpotModule::generateBlindSpotPolygons(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const autoware_planning_msgs::PathWithLaneId & path, const int closest_idx) const
{
  std::vector<int64_t> lane_ids;
  lanelet::ConstLanelets blind_spot_lanelets;
  /* get lane ids until intersection */
  for (const auto & point : path.points) {
    lane_ids.push_back(point.lane_ids.front());
    if (point.lane_ids.front() == lane_id_) break;
  }
  /* remove adjacent duplicates */
  lane_ids.erase(std::unique(lane_ids.begin(), lane_ids.end()), lane_ids.end());

  /* reverse lane ids */
  std::reverse(lane_ids.begin(), lane_ids.end());

  /* add intersection lanelet */
  const auto first_lanelet = lanelet_map_ptr->laneletLayer.get(lane_ids.front());
  const auto first_half_lanelet = generateHalfLanelet(first_lanelet);
  blind_spot_lanelets.push_back(first_half_lanelet);

  if (lane_ids.size() > 1) {
    for (size_t i = 0; i < lane_ids.size() - 1; ++i) {
      const auto prev_lanelet = lanelet_map_ptr->laneletLayer.get(lane_ids.at(i));
      const auto next_lanelet = lanelet_map_ptr->laneletLayer.get(lane_ids.at(i + 1));
      /* end if next lanelet does not follow prev lanelet */
      if (!lanelet::geometry::follows(prev_lanelet.invert(), next_lanelet.invert())) break;
      const auto half_lanelet = generateHalfLanelet(next_lanelet);
      blind_spot_lanelets.push_back(half_lanelet);
    }
    /* reset order of lanelets */
    std::reverse(blind_spot_lanelets.begin(), blind_spot_lanelets.end());
  }

  const auto current_arc =
    lanelet::utils::getArcCoordinates(blind_spot_lanelets, path.points[closest_idx].point.pose);
  const auto total_length = lanelet::utils::getLaneletLength3d(blind_spot_lanelets);
  const auto intersection_length =
    lanelet::utils::getLaneletLength3d(lanelet_map_ptr->laneletLayer.get(lane_id_));
  const auto detection_area_start_length =
    total_length - intersection_length - planner_param_.backward_length;
  const auto conflict_area_start_length = std::max(detection_area_start_length, current_arc.length);
  const auto conflict_area = lanelet::utils::getPolygonFromArcLength(
    blind_spot_lanelets, conflict_area_start_length, total_length);
  const auto detection_area = lanelet::utils::getPolygonFromArcLength(
    blind_spot_lanelets, detection_area_start_length, total_length);

  BlindSpotPolygons blind_spot_polygons;
  blind_spot_polygons.conflict_area = conflict_area;
  blind_spot_polygons.detection_area = detection_area;

  return blind_spot_polygons;
}

lanelet::LineString2d BlindSpotModule::getVehicleEdge(
  const geometry_msgs::Pose & vehicle_pose, const double vehicle_width,
  const double base_link2front) const
{
  lanelet::LineString2d vehicle_edge;
  tf2::Vector3 front_left, front_right, rear_left, rear_right;
  front_left.setValue(base_link2front, vehicle_width / 2, 0);
  front_right.setValue(base_link2front, -vehicle_width / 2, 0);
  rear_left.setValue(0, vehicle_width / 2, 0);
  rear_right.setValue(0, -vehicle_width / 2, 0);

  tf2::Transform tf;
  tf2::fromMsg(vehicle_pose, tf);
  const auto front_left_transformed = tf * front_left;
  const auto front_right_transformed = tf * front_right;
  const auto rear_left_transformed = tf * rear_left;
  const auto rear_right_transformed = tf * rear_right;

  if (turn_direction_ == TurnDirection::LEFT) {
    vehicle_edge.push_back(
      lanelet::Point2d(0, front_left_transformed.x(), front_left_transformed.y()));
    vehicle_edge.push_back(
      lanelet::Point2d(0, rear_left_transformed.x(), rear_left_transformed.y()));
  } else if (turn_direction_ == TurnDirection::RIGHT) {
    vehicle_edge.push_back(
      lanelet::Point2d(0, front_right_transformed.x(), front_right_transformed.y()));
    vehicle_edge.push_back(
      lanelet::Point2d(0, rear_right_transformed.x(), rear_right_transformed.y()));
  }
  return vehicle_edge;
}

bool BlindSpotModule::isTargetObjectType(
  const autoware_perception_msgs::DynamicObject & object) const
{
  if (
    object.semantic.type == autoware_perception_msgs::Semantic::BICYCLE ||
    object.semantic.type == autoware_perception_msgs::Semantic::PEDESTRIAN ||
    object.semantic.type == autoware_perception_msgs::Semantic::MOTORBIKE) {
    return true;
  }
  return false;
}

boost::optional<geometry_msgs::Point> BlindSpotModule::getStartPointFromLaneLet(
  const int lane_id) const
{
  lanelet::ConstLanelet lanelet = planner_data_->lanelet_map->laneletLayer.get(lane_id);
  if (lanelet.centerline().empty()) return boost::none;
  const auto p = lanelet.centerline().front();
  geometry_msgs::Point start_point;
  start_point.x = p.x();
  start_point.y = p.y();
  start_point.z = p.z();

  return start_point;
}

lanelet::ConstLanelets BlindSpotModule::getStraightLanelets(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const int lane_id)
{
  lanelet::ConstLanelets straight_lanelets;
  const auto intersection_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id);
  const auto prev_intersection_lanelets = routing_graph_ptr->previous(intersection_lanelet);
  if (prev_intersection_lanelets.empty()) return straight_lanelets;

  const auto next_lanelets = routing_graph_ptr->following(prev_intersection_lanelets.front());
  for (const auto & ll : next_lanelets) {
    const std::string turn_direction = ll.attributeOr("turn_direction", "else");
    if (!turn_direction.compare("straight")) {
      straight_lanelets.push_back(ll);
    }
  }
  return straight_lanelets;
}

void BlindSpotModule::StateMachine::setStateWithMarginTime(State state)
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

void BlindSpotModule::StateMachine::setState(State state) { state_ = state; }

void BlindSpotModule::StateMachine::setMarginTime(const double t) { margin_time_ = t; }

BlindSpotModule::State BlindSpotModule::StateMachine::getState() { return state_; }
