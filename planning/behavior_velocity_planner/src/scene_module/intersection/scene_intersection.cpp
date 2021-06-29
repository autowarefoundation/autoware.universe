// Copyright 2020 Tier IV, Inc.
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

#include "scene_module/intersection/scene_intersection.hpp"

#include <algorithm>
#include <memory>
#include <vector>

#include "lanelet2_core/geometry/Polygon.h"
#include "lanelet2_core/primitives/BasicRegulatoryElements.h"
#include "lanelet2_extension/regulatory_elements/road_marking.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/utility/utilities.hpp"

#include "scene_module/intersection/util.hpp"
#include "utilization/boost_geometry_helper.hpp"
#include "utilization/interpolate.hpp"
#include "utilization/util.hpp"

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

IntersectionModule::IntersectionModule(
  const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock), lane_id_(lane_id)
{
  planner_param_ = planner_param;
  const auto & assigned_lanelet = planner_data->lanelet_map->laneletLayer.get(lane_id);
  turn_direction_ = assigned_lanelet.attributeOr("turn_direction", "else");
  has_traffic_light_ =
    !(assigned_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>().empty());
  state_machine_.setMarginTime(planner_param_.state_transit_margin_time);
}

bool IntersectionModule::modifyPathVelocity(
  autoware_planning_msgs::msg::PathWithLaneId * path,
  autoware_planning_msgs::msg::StopReason * stop_reason)
{
  const bool external_go =
    isTargetExternalInputStatus(autoware_api_msgs::msg::IntersectionStatus::GO);
  const bool external_stop =
    isTargetExternalInputStatus(autoware_api_msgs::msg::IntersectionStatus::STOP);
  RCLCPP_DEBUG(logger_, "===== plan start =====");
  debug_data_ = DebugData();
  *stop_reason =
    planning_utils::initializeStopReason(autoware_planning_msgs::msg::StopReason::INTERSECTION);

  const auto input_path = *path;
  debug_data_.path_raw = input_path;

  State current_state = state_machine_.getState();
  RCLCPP_DEBUG(logger_, "lane_id = %ld, state = %s", lane_id_, toString(current_state).c_str());

  /* get current pose */
  geometry_msgs::msg::PoseStamped current_pose = planner_data_->current_pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  /* get detection area and conflicting area */
  std::vector<lanelet::CompoundPolygon3d> detection_areas;
  std::vector<lanelet::CompoundPolygon3d> conflicting_areas;

  util::getObjectivePolygons(
    lanelet_map_ptr, routing_graph_ptr, lane_id_, planner_param_, &conflicting_areas,
    &detection_areas, logger_);
  if (detection_areas.empty()) {
    RCLCPP_DEBUG(logger_, "no detection area. skip computation.");
    return true;
  }
  debug_data_.detection_area = detection_areas;

  /* set stop-line and stop-judgement-line for base_link */
  int stop_line_idx = -1;
  int pass_judge_line_idx = -1;
  int first_idx_inside_lane = -1;
  const auto target_path = trimPathWithLaneId(*path);
  if (!util::generateStopLine(
      lane_id_, conflicting_areas, planner_data_, planner_param_, path, target_path, &stop_line_idx,
      &pass_judge_line_idx, &first_idx_inside_lane, logger_.get_child("util")))
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "setStopLineIdx fail");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }

  if (stop_line_idx <= 0 || pass_judge_line_idx <= 0) {
    RCLCPP_DEBUG(logger_, "stop line or pass judge line is at path[0], ignore planning.");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return true;
  }

  /* calc closest index */
  int closest_idx = -1;
  if (!planning_utils::calcClosestIndex(input_path, current_pose.pose, closest_idx)) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "calcClosestIndex fail");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }

  /* if current_state = GO, and current_pose is in front of stop_line, ignore planning. */
  bool is_over_pass_judge_line = static_cast<bool>(closest_idx > pass_judge_line_idx);
  if (closest_idx == pass_judge_line_idx) {
    geometry_msgs::msg::Pose pass_judge_line = path->points.at(pass_judge_line_idx).point.pose;
    is_over_pass_judge_line = util::isAheadOf(current_pose.pose, pass_judge_line);
  }
  if (current_state == State::GO && is_over_pass_judge_line && !external_stop) {
    RCLCPP_DEBUG(logger_, "over the pass judge line. no plan needed.");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return true;  // no plan needed.
  }

  /* get dynamic object */
  const auto objects_ptr = planner_data_->dynamic_objects;

  /* calculate dynamic collision around detection area */
  bool has_collision = checkCollision(*path, detection_areas, objects_ptr, closest_idx);
  bool is_stuck = checkStuckVehicleInIntersection(*path, closest_idx, stop_line_idx, objects_ptr);
  bool is_entry_prohibited = (has_collision || is_stuck);
  if (external_go) {
    is_entry_prohibited = false;
  } else if (external_stop) {
    is_entry_prohibited = true;
  }
  state_machine_.setStateWithMarginTime(
    is_entry_prohibited ? State::STOP : State::GO, logger_.get_child("state_machine"), *clock_);

  /* set stop speed : TODO behavior on straight lane should be improved*/
  if (state_machine_.getState() == State::STOP) {
    constexpr double stop_vel = 0.0;
    const double decel_vel = planner_param_.decel_velocity;
    const bool is_stop_required = is_stuck || !has_traffic_light_ || turn_direction_ != "straight";
    const double v = is_stop_required ? stop_vel : decel_vel;
    const double base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
    util::setVelocityFrom(stop_line_idx, v, path);

    if (is_stop_required) {
      debug_data_.stop_required = true;
      debug_data_.stop_wall_pose = util::getAheadPose(stop_line_idx, base_link2front, *path);
      debug_data_.stop_point_pose = path->points.at(stop_line_idx).point.pose;
      debug_data_.judge_point_pose = path->points.at(pass_judge_line_idx).point.pose;

      /* get stop point and stop factor */
      autoware_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = debug_data_.stop_point_pose;
      const auto stop_factor_conflict =
        planning_utils::toRosPoints(debug_data_.conflicting_targets);
      const auto stop_factor_stuck = planning_utils::toRosPoints(debug_data_.stuck_targets);
      stop_factor.stop_factor_points =
        planning_utils::concatVector(stop_factor_conflict, stop_factor_stuck);
      planning_utils::appendStopReason(stop_factor, stop_reason);

    } else {
      debug_data_.stop_required = false;
      debug_data_.slow_wall_pose = util::getAheadPose(stop_line_idx, base_link2front, *path);
    }
  }

  RCLCPP_DEBUG(logger_, "===== plan end =====");
  return true;
}

void IntersectionModule::cutPredictPathWithDuration(
  autoware_perception_msgs::msg::DynamicObjectArray * objects_ptr, const double time_thr) const
{
  const rclcpp::Time current_time = clock_->now();
  for (auto & object : objects_ptr->objects) {                    // each objects
    for (auto & predicted_path : object.state.predicted_paths) {  // each predicted paths
      std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> vp;
      for (auto & predicted_pose : predicted_path.path) {  // each path points
        if ((rclcpp::Time(predicted_pose.header.stamp) - current_time).seconds() < time_thr) {
          vp.push_back(predicted_pose);
        }
      }
      predicted_path.path = vp;
    }
  }
}

bool IntersectionModule::checkCollision(
  const autoware_planning_msgs::msg::PathWithLaneId & path,
  const std::vector<lanelet::CompoundPolygon3d> & detection_areas,
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr objects_ptr,
  const int closest_idx)
{
  /* generate ego-lane polygon */
  const Polygon2d ego_poly = generateEgoIntersectionLanePolygon(
    path, closest_idx, closest_idx, 0.0, 0.0);  // TODO(someone): use Lanelet
  debug_data_.ego_lane_polygon = toGeomMsg(ego_poly);

  /* extract target objects */
  autoware_perception_msgs::msg::DynamicObjectArray target_objects;
  for (const auto & object : objects_ptr->objects) {
    // ignore non-vehicle type objects, such as pedestrian.
    if (!isTargetCollisionVehicleType(object)) {continue;}

    // ignore vehicle in ego-lane. (TODO update check algorithm)
    const auto object_pose = object.state.pose_covariance.pose;
    const bool is_in_ego_lane = bg::within(to_bg2d(object_pose.position), ego_poly);
    if (is_in_ego_lane) {
      continue;  // TODO(Kenji Miyake): check direction?
    }

    // keep vehicle in detection_area
    const Polygon2d obj_poly = toFootprintPolygon(object);

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

autoware_planning_msgs::msg::PathWithLaneId IntersectionModule::trimPathWithLaneId(
  const autoware_planning_msgs::msg::PathWithLaneId & path)
{
  autoware_planning_msgs::msg::PathWithLaneId trimmed_path;
  trimmed_path.header = path.header;
  trimmed_path.drivable_area = path.drivable_area;

  for (const auto & point : path.points) {
    if (util::hasLaneId(point, lane_id_)) {
      trimmed_path.points.emplace_back(point);
    }
  }
  return trimmed_path;
}

Polygon2d IntersectionModule::generateEgoIntersectionLanePolygon(
  const autoware_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const int start_idx, const double extra_dist, const double ignore_dist) const
{
  const size_t assigned_lane_start_idx = start_idx;
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

  size_t ego_area_start_idx = assigned_lane_start_idx;
  {
    // decide start idx with considering ignore_dist
    double dist_sum = 0.0;
    for (size_t i = assigned_lane_start_idx + 1; i < assigned_lane_end_idx; ++i) {
      dist_sum += planning_utils::calcDist2d(path.points.at(i), path.points.at(i - 1));
      ++ego_area_start_idx;
      if (dist_sum > ignore_dist) {
        break;
      }
    }
  }

  if (static_cast<int>(ego_area_start_idx) < closest_idx) {
    // If ego-position is over the start_idx, use closest_idx as start
    ego_area_start_idx = closest_idx;
  }

  size_t ego_area_end_idx = assigned_lane_end_idx;
  {
    // decide end idx with considering extra_dist
    double dist_sum = 0.0;
    for (size_t i = assigned_lane_end_idx + 1; i < path.points.size(); ++i) {
      dist_sum += planning_utils::calcDist2d(path.points.at(i), path.points.at(i - 1));
      if (dist_sum > extra_dist) {
        break;
      }
      ++ego_area_end_idx;
    }
  }

  Polygon2d ego_area;  // open polygon
  const auto width = planner_param_.path_expand_width;
  for (int i = ego_area_start_idx; i <= static_cast<int>(ego_area_end_idx); ++i) {
    double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    double x = path.points.at(i).point.pose.position.x + width * std::sin(yaw);
    double y = path.points.at(i).point.pose.position.y - width * std::cos(yaw);
    ego_area.outer().push_back(Point2d(x, y));
  }
  for (int i = ego_area_end_idx; i >= static_cast<int>(ego_area_start_idx); --i) {
    if (i < 0) {
      break;
    }
    double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    double x = path.points.at(i).point.pose.position.x - width * std::sin(yaw);
    double y = path.points.at(i).point.pose.position.y + width * std::cos(yaw);
    ego_area.outer().push_back(Point2d(x, y));
  }

  return ego_area;
}

double IntersectionModule::calcIntersectionPassingTime(
  const autoware_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const int objective_lane_id) const
{
  double closest_vel =
    (std::max(1e-01, std::fabs(planner_data_->current_velocity->twist.linear.x)));
  double dist_sum = 0.0;
  double passing_time = 0.0;
  int assigned_lane_found = false;

  for (size_t i = closest_idx + 1; i < path.points.size(); ++i) {
    const double dist = planning_utils::calcDist2d(path.points.at(i - 1), path.points.at(i));
    dist_sum += dist;
    // calc vel in idx i+1 (v_{i+1}^2 - v_{i}^2 = 2ax)
    const double next_vel = std::min(
      std::sqrt(std::pow(closest_vel, 2.0) + 2.0 * planner_param_.intersection_max_acc * dist),
      planner_param_.intersection_velocity);
    // calc average vel in idx i~i+1
    const double average_vel =
      std::min((closest_vel + next_vel) / 2.0, planner_param_.intersection_velocity);
    passing_time += dist / average_vel;
    closest_vel = next_vel;

    bool has_objective_lane_id = util::hasLaneId(path.points.at(i), objective_lane_id);

    if (assigned_lane_found && !has_objective_lane_id) {
      break;
    }
    assigned_lane_found = has_objective_lane_id;
  }
  if (!assigned_lane_found) {
    return 0.0;  // has already passed the intersection.
  }

  RCLCPP_DEBUG(logger_, "intersection dist = %f, passing_time = %f", dist_sum, passing_time);

  return passing_time;
}

bool IntersectionModule::checkStuckVehicleInIntersection(
  const autoware_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const int stop_idx,
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr objects_ptr) const
{
  const double detect_length =
    planner_param_.stuck_vehicle_detect_dist + planner_data_->vehicle_info_.vehicle_length_m;
  const Polygon2d stuck_vehicle_detect_area = generateEgoIntersectionLanePolygon(
    path, closest_idx, stop_idx, detect_length, planner_param_.stuck_vehicle_ignore_dist);
  debug_data_.stuck_vehicle_detect_area = toGeomMsg(stuck_vehicle_detect_area);

  for (const auto & object : objects_ptr->objects) {
    if (!isTargetStuckVehicleType(object)) {
      continue;  // not target vehicle type
    }
    const auto obj_v = std::fabs(object.state.twist_covariance.twist.linear.x);
    if (obj_v > planner_param_.stuck_vehicle_vel_thr) {
      continue;  // not stop vehicle
    }

    // check if the footprint is in the stuck detect area
    const Polygon2d obj_footprint = toFootprintPolygon(object);
    const bool is_in_stuck_area = !bg::disjoint(obj_footprint, stuck_vehicle_detect_area);
    if (is_in_stuck_area) {
      RCLCPP_DEBUG(logger_, "stuck vehicle found.");
      debug_data_.stuck_targets.objects.push_back(object);
      return true;
    }
  }
  return false;
}

Polygon2d IntersectionModule::toFootprintPolygon(
  const autoware_perception_msgs::msg::DynamicObject & object) const
{
  Polygon2d obj_footprint;
  if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    obj_footprint = toBoostPoly(object.shape.footprint);
  } else {
    // cylinder type is treated as square-polygon
    obj_footprint = obj2polygon(object.state.pose_covariance.pose, object.shape.dimensions);
  }
  return obj_footprint;
}

bool IntersectionModule::isTargetCollisionVehicleType(
  const autoware_perception_msgs::msg::DynamicObject & object) const
{
  if (
    object.semantic.type == autoware_perception_msgs::msg::Semantic::CAR ||
    object.semantic.type == autoware_perception_msgs::msg::Semantic::BUS ||
    object.semantic.type == autoware_perception_msgs::msg::Semantic::TRUCK ||
    object.semantic.type == autoware_perception_msgs::msg::Semantic::MOTORBIKE ||
    object.semantic.type == autoware_perception_msgs::msg::Semantic::BICYCLE)
  {
    return true;
  }
  return false;
}

bool IntersectionModule::isTargetStuckVehicleType(
  const autoware_perception_msgs::msg::DynamicObject & object) const
{
  if (
    object.semantic.type == autoware_perception_msgs::msg::Semantic::CAR ||
    object.semantic.type == autoware_perception_msgs::msg::Semantic::BUS ||
    object.semantic.type == autoware_perception_msgs::msg::Semantic::TRUCK ||
    object.semantic.type == autoware_perception_msgs::msg::Semantic::MOTORBIKE)
  {
    return true;
  }
  return false;
}
void IntersectionModule::StateMachine::setStateWithMarginTime(
  State state, rclcpp::Logger logger, rclcpp::Clock & clock)
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
      start_time_ = std::make_shared<rclcpp::Time>(clock.now());
    } else {
      const double duration = (clock.now() - *start_time_).seconds();
      if (duration > margin_time_) {
        state_ = State::GO;
        start_time_ = nullptr;  // reset timer
      }
    }
    return;
  }

  RCLCPP_ERROR(logger, "Unsuitable state. ignore request.");
}

void IntersectionModule::StateMachine::setState(State state) {state_ = state;}

void IntersectionModule::StateMachine::setMarginTime(const double t) {margin_time_ = t;}

IntersectionModule::State IntersectionModule::StateMachine::getState() {return state_;}

bool IntersectionModule::isTargetExternalInputStatus(const int target_status)
{
  return planner_data_->external_intersection_status_input &&
         planner_data_->external_intersection_status_input.get().status == target_status &&
         (clock_->now() - planner_data_->external_intersection_status_input.get().header.stamp)
         .seconds() < planner_param_.external_input_timeout;
}
}  // namespace behavior_velocity_planner
