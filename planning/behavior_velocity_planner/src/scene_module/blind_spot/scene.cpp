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
: SceneModuleInterface(module_id), lane_id_(lane_id)
{
  planner_param_ = planner_param;
  const auto & assigned_lanelet = planner_data->lanelet_map->laneletLayer.get(lane_id);
  turn_direction_ = assigned_lanelet.attributeOr("turn_direction", "else");
  has_traffic_light_ =
    !(assigned_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>().empty());
}

bool BlindSpotModule::modifyPathVelocity(
  autoware_planning_msgs::PathWithLaneId * path,
  autoware_planning_msgs::StopReason * stop_reason)
{
  debug_data_ = {};
  *stop_reason =
    planning_utils::initializeStopReason(autoware_planning_msgs::StopReason::BLIND_SPOT);

  const auto input_path = *path;
  debug_data_.path_raw = input_path;

  State current_state = state_machine_.getState();
  ROS_DEBUG("[Blind Spot] lane_id = %ld, state = %d", lane_id_, static_cast<int>(current_state));

  /* get current pose */
  geometry_msgs::PoseStamped current_pose = planner_data_->current_pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  /* get detection area */
  std::vector<lanelet::CompoundPolygon3d> detection_areas;
  getObjectivePolygons(lanelet_map_ptr, routing_graph_ptr, lane_id_, &detection_areas);
  if (detection_areas.empty()) {
    ROS_DEBUG("[Blind Spot] no detection area. skip computation.");
    return true;
  }

  /* set stop-line and stop-judgement-line for base_link */
  int stop_line_idx = -1;
  int pass_judge_line_idx = -1;
  if (!generateStopLine(detection_areas, path, &stop_line_idx, &pass_judge_line_idx)) {
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
  bool has_obstacle = checkObstacleInBlindSpot(*path, objects_ptr, closest_idx, stop_line_idx);
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

int BlindSpotModule::getFirstPointInsidePolygons(
  const autoware_planning_msgs::PathWithLaneId & path,
  const std::vector<lanelet::CompoundPolygon3d> & polygons) const
{
  int first_idx_inside_lanelet = -1;
  for (size_t i = 0; i < path.points.size(); ++i) {
    bool is_in_lanelet = false;
    auto p = path.points.at(i).point.pose.position;
    for (const auto & polygon : polygons) {
      const auto polygon_2d = lanelet::utils::to2D(polygon);
      is_in_lanelet = bg::within(to_bg2d(p), polygon_2d);
      if (is_in_lanelet) {
        first_idx_inside_lanelet = static_cast<int>(i);
        break;
      }
    }
    if (is_in_lanelet) break;
  }
  return first_idx_inside_lanelet;
}

bool BlindSpotModule::generateStopLine(
  const std::vector<lanelet::CompoundPolygon3d> detection_areas,
  autoware_planning_msgs::PathWithLaneId * path, int * stop_line_idx,
  int * pass_judge_line_idx) const
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
  // If a stop_line is defined in lanelet_map, use it.
  // else, generates a local stop_line with considering the lane conflictions.
  int stop_idx_ip;  // stop point index for interpolated path.
  geometry_msgs::Point stop_point_from_map;
  if (getStopPoseFromMap(lane_id_, &stop_point_from_map)) {
    planning_utils::calcClosestIndex(path_ip, stop_point_from_map, stop_idx_ip, 10.0);
    stop_idx_ip = std::max(stop_idx_ip - base2front_idx_dist, 0);
  } else {
    int first_idx_inside_lane = getFirstPointInsidePolygons(path_ip, detection_areas);
    if (first_idx_inside_lane == -1) {
      ROS_DEBUG("[intersection] generate stopline, but no intersect line found.");
      return false;
    }
    stop_idx_ip = std::max(first_idx_inside_lane - 1 - margin_idx_dist - base2front_idx_dist, 0);
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
    "[intersection] generateStopLine() : stop_idx = %d, pass_judge_idx = %d, stop_idx_ip = %d, "
    "pass_judge_idx_ip = %d, has_prior_stopline = %d",
    *stop_line_idx, *pass_judge_line_idx, stop_idx_ip, pass_judge_idx_ip, has_prior_stopline);

  return true;
}

bool BlindSpotModule::getObjectivePolygons(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const int lane_id, std::vector<lanelet::CompoundPolygon3d> * polygons)
{
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id);

  lanelet::ConstLanelets exclude_lanelets;

  // for non-priority roads.
  const auto right_of_ways = assigned_lanelet.regulatoryElementsAs<lanelet::RightOfWay>();
  for (const auto & right_of_way : right_of_ways) {
    for (const auto & yield_lanelets : right_of_way->yieldLanelets()) {
      exclude_lanelets.push_back(yield_lanelets);
      for (const auto & previous_lanelet : routing_graph_ptr->previous(yield_lanelets)) {
        exclude_lanelets.push_back(previous_lanelet);
      }
    }
  }

  // for the behind ego-car lane.
  for (const auto & previous_lanelet : routing_graph_ptr->previous(assigned_lanelet)) {
    exclude_lanelets.push_back(previous_lanelet);
    for (const auto & following_lanelet : routing_graph_ptr->following(previous_lanelet)) {
      if (lanelet::utils::contains(exclude_lanelets, following_lanelet)) {
        continue;
      }
      exclude_lanelets.push_back(following_lanelet);
    }
  }

  // get conflicting lanes on assigned lanelet
  const auto & conflicting_lanelets =
    lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lanelet);

  lanelet::ConstLanelets objective_lanelets;  // final objective lanelets

  // remove exclude_lanelets from candidates
  for (const auto & conflicting_lanelet : conflicting_lanelets) {
    if (lanelet::utils::contains(exclude_lanelets, conflicting_lanelet)) {
      continue;
    }
    objective_lanelets.push_back(conflicting_lanelet);
  }

  // get possible lanelet path that reaches conflicting_lane longer than given length
  double length = 100;
  std::vector<lanelet::ConstLanelets> objective_lanelets_sequences;
  for (const auto & ll : objective_lanelets) {
    const auto & lanelet_sequences =
      lanelet::utils::query::getPreceedingLaneletSequences(routing_graph_ptr, ll, length);
    for (const auto & l : lanelet_sequences) {
      objective_lanelets_sequences.push_back(l);
    }
  }

  // get exact polygon of interest with exact length
  polygons->clear();
  for (const auto & ll : objective_lanelets_sequences) {
    const double path_length = lanelet::utils::getLaneletLength3d(ll);
    const auto polygon3d =
      lanelet::utils::getPolygonFromArcLength(ll, path_length - length, path_length);
    polygons->push_back(polygon3d);
  }

  debug_data_.detection_area = *polygons;

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
  const autoware_planning_msgs::PathWithLaneId & path,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr, const int closest_idx,
  const int stop_idx) const
{
  /* get detection area */
  if (turn_direction_.compare("right") != 0 && turn_direction_.compare("left") != 0) {
    ROS_WARN(
      "blind spot detector is running, turn_direction_ = not right or left. (%s)",
      turn_direction_.c_str());
    return false;
  }
  const auto areas = generateBlindSpotPolygons(path, closest_idx, stop_idx);
  debug_data_.detection_area_for_blind_spot = toGeomMsg(areas.detection_area);
  debug_data_.confict_area_for_blind_spot = toGeomMsg(areas.conflict_area);

  autoware_perception_msgs::DynamicObjectArray objects = *objects_ptr;
  cutPredictPathWithDuration(&objects, planner_param_.max_future_movement_time);

  // check objects in blind spot areas
  bool obstacle_detected = false;
  for (const auto & object : objects.objects) {
    if (!isTargetObjectType(object)) continue;

    bool exist_in_broad =
      bg::within(to_bg2d(object.state.pose_covariance.pose.position), areas.detection_area);
    bool exist_in_narrow = isPredictedPathInArea(object, areas.conflict_area);
    if (exist_in_narrow && exist_in_broad) {
      obstacle_detected = true;
      debug_data_.conflicting_targets.objects.push_back(object);
    }
  }
  return obstacle_detected;
}

double BlindSpotModule::getLaneletWidth(lanelet::ConstLanelet & lanelet) const
{
  const auto left2d = lanelet::utils::to2D(lanelet.leftBound().front()).basicPoint();
  const auto right2d = lanelet::utils::to2D(lanelet.rightBound().front()).basicPoint();
  return boost::geometry::distance(left2d, right2d);
}

bool BlindSpotModule::isPredictedPathInArea(
  const autoware_perception_msgs::DynamicObject & object, const Polygon2d & area) const
{
  bool exist_in_narrow = false;
  for (const auto & predicted_path : object.state.predicted_paths) {
    for (const auto & predicted_point : predicted_path.path) {
      exist_in_narrow = bg::within(to_bg2d(predicted_point.pose.pose.position), area);
      if (exist_in_narrow) return true;
    }
  }
  return false;
}

BlindSpotPolygons BlindSpotModule::generateBlindSpotPolygons(
  const autoware_planning_msgs::PathWithLaneId & path, const int closest_idx,
  const int stop_line_idx) const
{
  // make backward point along with ceter line for broad_area
  Polygon2d broad_area;
  geometry_msgs::Pose rel_point_along_center;
  rel_point_along_center.position.x = -1 * planner_param_.backward_length;
  const auto abs_point_along_center = planning_utils::transformAbsCoordinate2D(
    rel_point_along_center, path.points[closest_idx].point.pose);
  broad_area.outer().push_back(
    Point2d(abs_point_along_center.position.x, abs_point_along_center.position.y));

  Polygon2d narrow_area;
  for (int i = closest_idx; i <= stop_line_idx; ++i) {
    double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    double x = path.points.at(i).point.pose.position.x;
    double y = path.points.at(i).point.pose.position.y;
    narrow_area.outer().push_back(Point2d(x, y));
    broad_area.outer().push_back(Point2d(x, y));
  }

  // expand polygn to turning direction
  const double direction = (turn_direction_ == "left") ? 1.0 : -1.0;
  lanelet::ConstLanelet lanelet =
    planner_data_->lanelet_map->laneletLayer.get(path.points[closest_idx].lane_ids.front());
  const double expand_width = 0.5 * getLaneletWidth(lanelet);
  for (int i = stop_line_idx; i >= closest_idx; --i) {
    double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    double x = path.points.at(i).point.pose.position.x - direction * expand_width * std::sin(yaw);
    double y = path.points.at(i).point.pose.position.y + direction * expand_width * std::cos(yaw);
    narrow_area.outer().push_back(Point2d(x, y));
    broad_area.outer().push_back(Point2d(x, y));
  }

  // make backward point along with the bound for broad_area
  geometry_msgs::Pose rel_point_along_bound;
  rel_point_along_bound.position.x = -1 * planner_param_.backward_length;
  rel_point_along_bound.position.y = direction * expand_width;
  const auto abs_point_along_bound = planning_utils::transformAbsCoordinate2D(
    rel_point_along_bound, path.points[closest_idx].point.pose);
  broad_area.outer().push_back(
    Point2d(abs_point_along_bound.position.x, abs_point_along_bound.position.y));

  BlindSpotPolygons blind_spot_polygons;
  blind_spot_polygons.conflict_area = narrow_area;
  blind_spot_polygons.detection_area = broad_area;

  return blind_spot_polygons;
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

bool BlindSpotModule::getStopPoseFromMap(const int lane_id, geometry_msgs::Point * stop_point) const
{
  lanelet::ConstLanelet lanelet = planner_data_->lanelet_map->laneletLayer.get(lane_id);
  const auto road_markings = lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();
  lanelet::ConstLineStrings3d stop_line;
  for (const auto & road_marking : road_markings) {
    const std::string type =
      road_marking->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");
    if (type == lanelet::AttributeValueString::StopLine) {
      stop_line.push_back(road_marking->roadMarking());
      break;  // only one stop_line exists.
    }
  }
  if (stop_line.empty()) return false;

  const auto p_start = stop_line.front().front();
  const auto p_end = stop_line.front().back();
  stop_point->x = 0.5 * (p_start.x() + p_end.x());
  stop_point->y = 0.5 * (p_start.y() + p_end.y());
  stop_point->z = 0.5 * (p_start.z() + p_end.z());

  return true;
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
