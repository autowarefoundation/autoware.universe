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

#include "utilization/boost_geometry_helper.h"
#include "utilization/util.h"

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;

BlindSpotModule::BlindSpotModule(
  const int64_t module_id, const int64_t lane_id, const std::string & turn_direction)
: SceneModuleInterface(module_id), lane_id_(lane_id), turn_direction_(turn_direction)
{
  constexpr double state_change_margin_time = 2.0;
  state_machine_.setMarginTime(state_change_margin_time);  // [sec]
}

bool BlindSpotModule::modifyPathVelocity(autoware_planning_msgs::PathWithLaneId * path)
{
  debug_data_ = {};

  const auto input_path = *path;
  debug_data_.path_raw = input_path;

  State current_state = state_machine_.getState();
  ROS_DEBUG_COND(
    show_debug_info_, "[BlindSpotModule]: run: state_machine_.getState() = %d", (int)current_state);

  /* get current pose */
  geometry_msgs::PoseStamped current_pose = planner_data_->current_pose;

  /* check if the current_pose is ahead from judgement line */
  int closest = -1;
  if (!planning_utils::calcClosestIndex(input_path, current_pose.pose, closest)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[BlindSpotModule::run] calcClosestIndex fail");
    return false;
  }

  /* set judge line dist */
  double current_velocity = planner_data_->current_velocity->twist.linear.x;
  double max_accel = planner_data_->max_stop_acceleration_threshold_;
  judge_line_dist_ = planning_utils::calcJudgeLineDist(current_velocity, max_accel, 0.0);

  /* set stop-line and stop-judgement-line */
  if (!setStopLineIdx(closest, judge_line_dist_, *path, stop_line_idx_, judge_line_idx_)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[BlindSpotModule::run] setStopLineIdx fail");
    return false;
  }

  if (stop_line_idx_ <= 0 || judge_line_idx_ <= 0) {
    ROS_INFO_COND(
      show_debug_info_,
      "[BlindSpotModule::run] the stop line or judge line is at path[0], ignore "
      "planning. Maybe it is far behind the current position.");
    return true;
  }

  if (current_state == State::STOP) {
    // visualize virtual_wall at vehicle front position
    debug_data_.virtual_wall_pose =
      getAheadPose(stop_line_idx_, planner_data_->base_link2front, *path);
  }
  debug_data_.stop_point_pose = path->points.at(stop_line_idx_).point.pose;
  debug_data_.judge_point_pose = path->points.at(judge_line_idx_).point.pose;
  debug_data_.path_with_judgeline = *path;

  if (current_state == State::GO) {
    const auto p = planning_utils::transformRelCoordinate2D(
      current_pose.pose, path->points.at(judge_line_idx_).point.pose);

    // current_pose is ahead of judge_line
    if (p.position.x > 0.0) {
      ROS_INFO_COND(
        show_debug_info_, "[BlindSpotModule::run] no plan needed. skip collision check.");
      return true;  // no plan needed.
    }
  }

  /* get detection area */
  if (turn_direction_.compare("right") != 0 && turn_direction_.compare("left") != 0) {
    ROS_WARN(
      "blind spot detector is running, turn_direction_ = not right or left. (%s)",
      turn_direction_.c_str());
    return false;
  }

  const auto detection_area = generateDetectionArea(current_pose.pose);

  debug_data_.detection_area = detection_area;

  /* get dynamic object */
  const auto objects_ptr = planner_data_->dynamic_objects;

  /* calculate dynamic collision around detection area */
  const bool is_collision = checkCollision(*path, detection_area, objects_ptr, path_expand_width_);
  if (is_collision) {
    state_machine_.setStateWithMarginTime(State::STOP);
  } else {
    state_machine_.setStateWithMarginTime(State::GO);
  }

  /* set stop speed */
  if (state_machine_.getState() == State::STOP) {
    constexpr double stop_vel = 0.0;
    setVelocityFrom(stop_line_idx_, stop_vel, *path);
  }

  return true;
}

std::vector<geometry_msgs::Point> BlindSpotModule::generateDetectionArea(
  const geometry_msgs::Pose & current_pose)
{
  std::vector<geometry_msgs::Point> blind_spot;

  double detection_width = 5.0;
  double detection_length_forward = 5.0;
  double detection_length_backward = 15.0;
  double vehicle_width = 3.0;
  double vw = vehicle_width * 0.5;

  if (turn_direction_.compare("right") == 0) {
    vw *= -1.0;
    detection_width *= -1.0;
  } else if (turn_direction_.compare("left") == 0) {
    // nothing to do.
  }

  geometry_msgs::Pose fl;
  fl.position.x = detection_length_forward;
  fl.position.y = vw;
  blind_spot.push_back(planning_utils::transformAbsCoordinate2D(fl, current_pose).position);
  geometry_msgs::Pose fr;
  fr.position.x = detection_length_forward;
  fr.position.y = vw + detection_width;
  blind_spot.push_back(planning_utils::transformAbsCoordinate2D(fr, current_pose).position);
  geometry_msgs::Pose rr;
  rr.position.x = -detection_length_backward;
  rr.position.y = vw + detection_width;
  blind_spot.push_back(planning_utils::transformAbsCoordinate2D(rr, current_pose).position);
  geometry_msgs::Pose rl;
  rl.position.x = -detection_length_backward;
  rl.position.y = vw;
  blind_spot.push_back(planning_utils::transformAbsCoordinate2D(rl, current_pose).position);

  return blind_spot;
}

bool BlindSpotModule::setStopLineIdx(
  const int current_pose_closest, const double judge_line_dist,
  autoware_planning_msgs::PathWithLaneId & path, int & stop_line_idx, int & judge_line_idx)
{
  // TEMP: return first assigned_lane_id point's index
  stop_line_idx = -1;
  for (size_t i = 0; i < path.points.size(); ++i) {
    for (const auto & id : path.points.at(i).lane_ids) {
      if (id == lane_id_) {
        stop_line_idx = i;
      }
      if (stop_line_idx != -1) break;
    }
    if (stop_line_idx != -1) break;
  }

  if (stop_line_idx == -1) {
    ROS_ERROR(
      "[BlindSpotModule::setStopLineIdx]: cannot set the stop line. something wrong. please "
      "check code. ");
    return false;  // cannot find stop line.
  }

  // TEMP: should use interpolation (points distance may be very long)
  double curr_dist = 0.0;
  double prev_dist = curr_dist;
  judge_line_idx = -1;

  for (size_t i = stop_line_idx; i > 0; --i) {
    const geometry_msgs::Pose p0 = path.points.at(i).point.pose;
    const geometry_msgs::Pose p1 = path.points.at(i - 1).point.pose;
    curr_dist += planning_utils::calcDist2d(p0, p1);
    if (curr_dist > judge_line_dist) {
      const double dl = std::max(curr_dist - prev_dist, 0.0001 /* avoid 0 divide */);
      const double w_p0 = (curr_dist - judge_line_dist) / dl;
      const double w_p1 = (judge_line_dist - prev_dist) / dl;
      autoware_planning_msgs::PathPointWithLaneId p = path.points.at(i);
      p.point.pose.position.x = w_p0 * p0.position.x + w_p1 * p1.position.x;
      p.point.pose.position.y = w_p0 * p0.position.y + w_p1 * p1.position.y;
      p.point.pose.position.z = w_p0 * p0.position.z + w_p1 * p1.position.z;
      tf2::Quaternion q0_tf, q1_tf;
      tf2::fromMsg(p0.orientation, q0_tf);
      tf2::fromMsg(p1.orientation, q1_tf);
      p.point.pose.orientation = tf2::toMsg(q0_tf.slerp(q1_tf, w_p1));
      auto itr = path.points.begin();
      itr += i;
      path.points.insert(itr, p);
      judge_line_idx = i;
      ++stop_line_idx;
      break;
    }
    prev_dist = curr_dist;
  }
  if (judge_line_idx == -1) {
    ROS_DEBUG(
      "[BlindSpotModule::setStopLineIdx]: cannot set the stop judgement line. path is too short, "
      "or "
      "the vehicle is already ahead of the stop line. stop_line_id = %d",
      stop_line_idx);
  }
  return true;
}

geometry_msgs::Pose BlindSpotModule::getAheadPose(
  const size_t start_idx, const double ahead_dist,
  const autoware_planning_msgs::PathWithLaneId & path) const
{
  if (path.points.size() == 0) {
    return geometry_msgs::Pose{};
  }

  double curr_dist = 0.0;
  double prev_dist = 0.0;
  for (size_t i = start_idx; i < path.points.size() - 1 && i >= 0; ++i) {
    const geometry_msgs::Pose p0 = path.points.at(i).point.pose;
    const geometry_msgs::Pose p1 = path.points.at(i + 1).point.pose;
    curr_dist += planning_utils::calcDist2d(p0, p1);
    if (curr_dist > ahead_dist) {
      const double dl = std::max(curr_dist - prev_dist, 0.0001 /* avoid 0 divide */);
      const double w_p0 = (curr_dist - ahead_dist) / dl;
      const double w_p1 = (ahead_dist - prev_dist) / dl;
      geometry_msgs::Pose p;
      p.position.x = w_p0 * p0.position.x + w_p1 * p1.position.x;
      p.position.y = w_p0 * p0.position.y + w_p1 * p1.position.y;
      p.position.z = w_p0 * p0.position.z + w_p1 * p1.position.z;
      tf2::Quaternion q0_tf, q1_tf;
      tf2::fromMsg(p0.orientation, q0_tf);
      tf2::fromMsg(p1.orientation, q1_tf);
      p.orientation = tf2::toMsg(q0_tf.slerp(q1_tf, w_p1));
      return p;
    }
    prev_dist = curr_dist;
  }
  return path.points.back().point.pose;
}

bool BlindSpotModule::setVelocityFrom(
  const size_t idx, const double vel, autoware_planning_msgs::PathWithLaneId & input)
{
  for (size_t i = idx; i < input.points.size(); ++i) {
    input.points.at(i).point.twist.linear.x =
      std::min(vel, input.points.at(i).point.twist.linear.x);
  }
}

bool BlindSpotModule::checkCollision(
  const autoware_planning_msgs::PathWithLaneId & path,
  const std::vector<geometry_msgs::Point> & detection_area,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr, const double path_width)
{
  /* generates side edge line */
  autoware_planning_msgs::PathWithLaneId path_r;  // right side edge line
  autoware_planning_msgs::PathWithLaneId path_l;  // left side edge line
  generateEdgeLine(path, path_width, path_r, path_l);

  debug_data_.path_right_edge = path_r;
  debug_data_.path_left_edge = path_l;

  /* check collision for each objects and detection area */
  for (const auto & object : objects_ptr->objects) {
    const auto object_pose = object.state.pose_covariance.pose;

    // Ignore objects outside detection area
    const auto detection_area_polygon = linestring2polygon(to_bg2d(detection_area));
    if (!bg::within(to_bg2d(object_pose.position), detection_area_polygon)) {
      continue;
    }

    if (checkPathCollision(path_r, object) || checkPathCollision(path_l, object)) {
      return true;
    }
  }

  return false;
}

bool BlindSpotModule::checkPathCollision(
  const autoware_planning_msgs::PathWithLaneId & path,
  const autoware_perception_msgs::DynamicObject & object)
{
  bool is_collision = false;

  bg::model::linestring<Point> bg_ego_path;
  for (const auto & p : path.points) {
    bg_ego_path.push_back(Point{p.point.pose.position.x, p.point.pose.position.y});
  }

  std::vector<bg::model::linestring<Point>> bg_object_path_arr;
  for (size_t i = 0; i < object.state.predicted_paths.size(); ++i) {
    bg::model::linestring<Point> bg_object_path;
    for (const auto & p : object.state.predicted_paths.at(i).path) {
      bg_object_path.push_back(Point{p.pose.pose.position.x, p.pose.pose.position.y});
    }
    bg_object_path_arr.push_back(bg_object_path);
  }

  for (size_t i = 0; i < object.state.predicted_paths.size(); ++i) {
    bool is_intersects = bg::intersects(bg_ego_path, bg_object_path_arr.at(i));
    is_collision = is_collision || is_intersects;
  }

  return is_collision;
}

bool BlindSpotModule::generateEdgeLine(
  const autoware_planning_msgs::PathWithLaneId & path, const double path_width,
  autoware_planning_msgs::PathWithLaneId & path_r, autoware_planning_msgs::PathWithLaneId & path_l)
{
  path_r = path;
  path_l = path;
  for (int i = 0; i < path.points.size(); ++i) {
    const double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    path_r.points.at(i).point.pose.position.x += path_width * std::sin(yaw);
    path_r.points.at(i).point.pose.position.y -= path_width * std::cos(yaw);
    path_l.points.at(i).point.pose.position.x -= path_width * std::sin(yaw);
    path_l.points.at(i).point.pose.position.y += path_width * std::cos(yaw);
  }
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
      return;
    } else {
      const double duration = (ros::Time::now() - *start_time_).toSec();
      if (duration > margin_time_) {
        state_ = State::GO;
        start_time_ = nullptr;  // reset timer
      }
      return;
    }
  }

  ROS_ERROR(
    "[StateMachine::setStateWithMarginTime()] : Unsuitable state. ignore "
    "request.");
  return;
}

void BlindSpotModule::StateMachine::setState(State state) { state_ = state; }

void BlindSpotModule::StateMachine::setMarginTime(const double t) { margin_time_ = t; }

BlindSpotModule::State BlindSpotModule::StateMachine::getState() { return state_; }
