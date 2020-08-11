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
#include <scene_module/traffic_light/scene.h>

#include <map>

#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>

#include <utilization/util.h>

namespace
{
std::pair<int, double> findWayPointAndDistance(
  const autoware_planning_msgs::PathWithLaneId & input_path, const Eigen::Vector2d & p)
{
  constexpr double max_lateral_dist = 3.0;
  for (size_t i = 0; i < input_path.points.size() - 1; ++i) {
    const double dx = p.x() - input_path.points.at(i).point.pose.position.x;
    const double dy = p.y() - input_path.points.at(i).point.pose.position.y;
    const double dx_wp = input_path.points.at(i + 1).point.pose.position.x -
                         input_path.points.at(i).point.pose.position.x;
    const double dy_wp = input_path.points.at(i + 1).point.pose.position.y -
                         input_path.points.at(i).point.pose.position.y;

    const double theta = std::atan2(dy, dx) - std::atan2(dy_wp, dx_wp);

    const double dist = std::hypot(dx, dy);
    const double dist_wp = std::hypot(dx_wp, dy_wp);

    // check lateral distance
    if (std::fabs(dist * std::sin(theta)) > max_lateral_dist) {
      continue;
    }

    // if the point p is back of the way point, return negative distance
    if (dist * std::cos(theta) < 0) {
      return std::make_pair(static_cast<int>(i), -1.0 * dist);
    }

    if (dist * std::cos(theta) < dist_wp) {
      return std::make_pair(static_cast<int>(i), dist);
    }
  }

  // if the way point is not found, return negative distance from the way point at 0
  const double dx = p.x() - input_path.points.at(0).point.pose.position.x;
  const double dy = p.y() - input_path.points.at(0).point.pose.position.y;
  return std::make_pair(-1, -1.0 * std::hypot(dx, dy));
}

double calcArcLengthFromWayPoint(
  const autoware_planning_msgs::PathWithLaneId & input_path, const int & src, const int & dst)
{
  double length = 0;
  const size_t src_idx = src >= 0 ? static_cast<size_t>(src) : 0;
  const size_t dst_idx = dst >= 0 ? static_cast<size_t>(dst) : 0;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    const double dx_wp = input_path.points.at(i + 1).point.pose.position.x -
                         input_path.points.at(i).point.pose.position.x;
    const double dy_wp = input_path.points.at(i + 1).point.pose.position.y -
                         input_path.points.at(i).point.pose.position.y;
    length += std::hypot(dx_wp, dy_wp);
  }
  return length;
}

double calcSignedArcLength(
  const autoware_planning_msgs::PathWithLaneId & input_path, const geometry_msgs::Pose & p1,
  const Eigen::Vector2d & p2)
{
  std::pair<int, double> src =
    findWayPointAndDistance(input_path, Eigen::Vector2d(p1.position.x, p1.position.y));
  std::pair<int, double> dst = findWayPointAndDistance(input_path, p2);
  if (dst.first == -1) {
    double dx = p1.position.x - p2.x();
    double dy = p1.position.y - p2.y();
    return -1.0 * std::hypot(dx, dy);
  }

  if (src.first < dst.first) {
    return calcArcLengthFromWayPoint(input_path, src.first, dst.first) - src.second + dst.second;
  } else if (src.first > dst.first) {
    return -1.0 *
           (calcArcLengthFromWayPoint(input_path, dst.first, src.first) - dst.second + src.second);
  } else {
    return dst.second - src.second;
  }
}

double calcSignedDistance(const geometry_msgs::Pose & p1, const Eigen::Vector2d & p2)
{
  Eigen::Affine3d map2p1;
  tf2::fromMsg(p1, map2p1);
  auto basecoords_p2 = map2p1.inverse() * Eigen::Vector3d(p2.x(), p2.y(), p1.position.z);
  return basecoords_p2.x() >= 0 ? basecoords_p2.norm() : -basecoords_p2.norm();
}
}  // namespace

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Line = bg::model::linestring<Point>;
using Polygon = bg::model::polygon<Point, false>;

TrafficLightModule::TrafficLightModule(
  const int64_t module_id, const lanelet::TrafficLight & traffic_light_reg_elem,
  lanelet::ConstLanelet lane, const PlannerParam & planner_param)
: SceneModuleInterface(module_id),
  traffic_light_reg_elem_(traffic_light_reg_elem),
  lane_(lane),
  state_(State::APPROACH)
{
  planner_param_ = planner_param;
}

bool TrafficLightModule::modifyPathVelocity(
  autoware_planning_msgs::PathWithLaneId * path, autoware_planning_msgs::StopReason * stop_reason)
{
  debug_data_ = {};
  debug_data_.base_link2front = planner_data_->base_link2front;
  first_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;
  *stop_reason =
    planning_utils::initializeStopReason(autoware_planning_msgs::StopReason::TRAFFIC_LIGHT);

  const auto input_path = *path;

  // get lanelet2 traffic light
  lanelet::ConstLineString3d lanelet_stop_line = *(traffic_light_reg_elem_.stopLine());
  lanelet::ConstLineStringsOrPolygons3d traffic_lights = traffic_light_reg_elem_.trafficLights();

  // get vehicle info
  geometry_msgs::TwistStamped::ConstPtr self_twist_ptr = planner_data_->current_velocity;
  const double pass_judge_line_distance =
    planning_utils::calcJudgeLineDist(self_twist_ptr->twist.linear.x);

  geometry_msgs::PoseStamped self_pose = planner_data_->current_pose;

  // check state
  if (state_ == State::GO_OUT) {
    return true;
  } else {
    for (size_t i = 0; i < lanelet_stop_line.size() - 1; i++) {
      const Line stop_line = {
        {lanelet_stop_line[i].x(), lanelet_stop_line[i].y()},
        {lanelet_stop_line[i + 1].x(), lanelet_stop_line[i + 1].y()}};
      // Check Dead Line
      {
        constexpr double dead_line_range = 5.0;
        Eigen::Vector2d dead_line_point;
        size_t dead_line_point_idx;
        if (!createTargetPoint(
              input_path, stop_line, -2.0 /*overline margin*/, dead_line_point_idx,
              dead_line_point)) {
          continue;
        }

        if (isOverDeadLine(
              self_pose.pose, input_path, dead_line_point_idx, dead_line_point, dead_line_range)) {
          state_ = State::GO_OUT;
          return true;
        }
      }

      // Check Stop Line
      {
        Eigen::Vector2d stop_line_point;
        size_t stop_line_point_idx;
        if (!createTargetPoint(
              input_path, stop_line, planner_param_.stop_margin, stop_line_point_idx,
              stop_line_point)) {
          continue;
        }

        if (
          state_ != State::STOP &&
          calcSignedArcLength(input_path, self_pose.pose, stop_line_point) <
            pass_judge_line_distance) {
          ROS_WARN_THROTTLE(1.0, "[traffic_light] vehicle is over stop border");
          return true;
        }
      }

      if (!getHighestConfidenceTrafficLightState(traffic_lights, tl_state_)) {
        // Don't stop when UNKNOWN or TIMEOUT as discussed at #508
        continue;
      }

      // Check Traffic Light
      if (!isStopRequired(tl_state_.state)) {
        continue;
      }

      // Add Stop WayPoint
      if (!insertTargetVelocityPoint(
            input_path, stop_line, planner_param_.stop_margin, 0.0, *path)) {
        ROS_WARN("[traffic_light] cannot insert stop waypoint");
        continue;
      }
      state_ = State::STOP;
      /* get stop point and stop factor */
      autoware_planning_msgs::StopFactor stop_factor;
      stop_factor.stop_pose = debug_data_.first_stop_pose;
      stop_factor.stop_factor_points = debug_data_.traffic_light_points;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      return true;
    }
  }

  return false;
}

bool TrafficLightModule::isOverDeadLine(
  const geometry_msgs::Pose & self_pose, const autoware_planning_msgs::PathWithLaneId & input_path,
  const size_t & dead_line_point_idx, const Eigen::Vector2d & dead_line_point,
  const double dead_line_range)
{
  if (calcSignedArcLength(input_path, self_pose, dead_line_point) > dead_line_range) {
    return false;
  }

  double yaw;
  if (dead_line_point_idx == 0)
    yaw = std::atan2(
      input_path.points.at(dead_line_point_idx + 1).point.pose.position.y - dead_line_point.y(),
      input_path.points.at(dead_line_point_idx + 1).point.pose.position.x - dead_line_point.x());
  else
    yaw = std::atan2(
      dead_line_point.y() - input_path.points.at(dead_line_point_idx - 1).point.pose.position.y,
      dead_line_point.x() - input_path.points.at(dead_line_point_idx - 1).point.pose.position.x);

  // Calculate transform from dead_line_pose to self_pose
  tf2::Transform tf_dead_line_pose2self_pose;
  {
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    tf2::Transform tf_map2dead_line_pose(
      quat, tf2::Vector3(dead_line_point.x(), dead_line_point.y(), self_pose.position.z));
    tf2::Transform tf_map2self_pose;
    tf2::fromMsg(self_pose, tf_map2self_pose);
    tf_dead_line_pose2self_pose = tf_map2dead_line_pose.inverse() * tf_map2self_pose;

    // debug code
    geometry_msgs::Pose dead_line_pose;
    tf2::toMsg(tf_map2dead_line_pose, dead_line_pose);
    debug_data_.dead_line_poses.push_back(dead_line_pose);
  }

  if (0 < tf_dead_line_pose2self_pose.getOrigin().x()) {
    ROS_WARN("[traffic_light] vehicle is over dead line");
    return true;
  }

  return false;
}

bool TrafficLightModule::isStopRequired(
  const autoware_perception_msgs::TrafficLightState & tl_state)
{
  if (hasLamp(tl_state, autoware_perception_msgs::LampState::GREEN)) {
    return false;
  }

  const std::string turn_direction = lane_.attributeOr("turn_direction", "else");

  if (turn_direction == "else") {
    return true;
  }

  if (turn_direction == "right" && hasLamp(tl_state, autoware_perception_msgs::LampState::RIGHT)) {
    return false;
  }

  if (turn_direction == "left" && hasLamp(tl_state, autoware_perception_msgs::LampState::LEFT)) {
    return false;
  }

  if (turn_direction == "straight" && hasLamp(tl_state, autoware_perception_msgs::LampState::UP)) {
    return false;
  }

  return true;
}

bool TrafficLightModule::getHighestConfidenceTrafficLightState(
  const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
  autoware_perception_msgs::TrafficLightStateStamped & highest_confidence_tl_state)
{
  // search traffic light state
  bool found = false;
  double highest_confidence = 0.0;
  std::string reason;
  for (const auto & traffic_light : traffic_lights) {
    // traffic light must be linestrings
    if (!traffic_light.isLineString()) {
      reason = "NotLineString";
      continue;
    }

    const int id = static_cast<lanelet::ConstLineString3d>(traffic_light).id();
    const auto tl_state_stamped = planner_data_->getTrafficLightState(id);
    if (!tl_state_stamped) {
      reason = "TrafficLightStateNotFound";
      continue;
    }

    const auto header = tl_state_stamped->header;
    const auto tl_state = tl_state_stamped->state;
    if (!((ros::Time::now() - header.stamp).toSec() < planner_param_.tl_state_timeout)) {
      reason = "TimeOut";
      continue;
    }

    if (
      tl_state.lamp_states.empty() ||
      tl_state.lamp_states.front().type == autoware_perception_msgs::LampState::UNKNOWN) {
      reason = "LampStateUnknown";
      continue;
    }

    if (highest_confidence < tl_state.lamp_states.front().confidence) {
      highest_confidence = tl_state.lamp_states.front().confidence;
      highest_confidence_tl_state = *tl_state_stamped;
      const std::vector<geometry_msgs::Point> highest_traffic_light{
        getTrafficLightPosition(traffic_light)};
      // store only highest confidence traffic light (not all traffic light)
      debug_data_.traffic_light_points = highest_traffic_light;
    }
    found = true;
  }
  if (!found) {
    ROS_WARN_THROTTLE(
      1.0, "[traffic_light] cannot find traffic light lamp state (%s).", reason.c_str());
    return false;
  }
  return true;
}

bool TrafficLightModule::insertTargetVelocityPoint(
  const autoware_planning_msgs::PathWithLaneId & input,
  const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &
    stop_line,
  const double & margin, const double & velocity, autoware_planning_msgs::PathWithLaneId & output)
{
  // create target point
  Eigen::Vector2d target_point;
  size_t insert_target_point_idx;
  autoware_planning_msgs::PathPointWithLaneId target_point_with_lane_id;

  if (!createTargetPoint(input, stop_line, margin, insert_target_point_idx, target_point))
    return false;
  const int target_velocity_point_idx = std::max(static_cast<int>(insert_target_point_idx - 1), 0);
  target_point_with_lane_id = output.points.at(target_velocity_point_idx);
  target_point_with_lane_id.point.pose.position.x = target_point.x();
  target_point_with_lane_id.point.pose.position.y = target_point.y();
  target_point_with_lane_id.point.twist.linear.x = velocity;
  output = input;

  // insert target point
  output.points.insert(output.points.begin() + insert_target_point_idx, target_point_with_lane_id);

  // insert 0 velocity after target point
  for (size_t j = insert_target_point_idx; j < output.points.size(); ++j)
    output.points.at(j).point.twist.linear.x =
      std::min(velocity, output.points.at(j).point.twist.linear.x);
  if (velocity == 0.0 && target_velocity_point_idx < first_stop_path_point_index_) {
    first_stop_path_point_index_ = target_velocity_point_idx;
    debug_data_.first_stop_pose = target_point_with_lane_id.point.pose;
  }
  // -- debug code --
  if (velocity == 0.0) debug_data_.stop_poses.push_back(target_point_with_lane_id.point.pose);
  // ----------------
  return true;
}

bool TrafficLightModule::createTargetPoint(
  const autoware_planning_msgs::PathWithLaneId & input,
  const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &
    stop_line,
  const double & margin, size_t & target_point_idx, Eigen::Vector2d & target_point)
{
  for (size_t i = 0; i < input.points.size() - 1; ++i) {
    Line path_line = {
      {input.points.at(i).point.pose.position.x, input.points.at(i).point.pose.position.y},
      {input.points.at(i + 1).point.pose.position.x, input.points.at(i + 1).point.pose.position.y}};
    std::vector<Point> collision_points;
    bg::intersection(stop_line, path_line, collision_points);

    if (collision_points.empty()) continue;

    // check nearest collision point
    Point nearest_collision_point;
    double min_dist;
    for (size_t j = 0; j < collision_points.size(); ++j) {
      double dist = bg::distance(
        Point(input.points.at(i).point.pose.position.x, input.points.at(i).point.pose.position.y),
        collision_points.at(j));
      if (j == 0 || dist < min_dist) {
        min_dist = dist;
        nearest_collision_point = collision_points.at(j);
      }
    }

    // search target point index
    target_point_idx = 0;
    const double base_link2front = planner_data_->base_link2front;
    double length_sum = 0;

    const double target_length = margin + base_link2front;
    Eigen::Vector2d point1, point2;
    if (0 <= target_length) {
      point1 << nearest_collision_point.x(), nearest_collision_point.y();
      point2 << input.points.at(i).point.pose.position.x, input.points.at(i).point.pose.position.y;
      length_sum += (point2 - point1).norm();
      for (size_t j = i; 0 < j; --j) {
        if (target_length < length_sum) {
          target_point_idx = j + 1;
          break;
        }
        point1 << input.points.at(j).point.pose.position.x,
          input.points.at(j).point.pose.position.y;
        point2 << input.points.at(j - 1).point.pose.position.x,
          input.points.at(j - 1).point.pose.position.y;
        length_sum += (point2 - point1).norm();
      }
    } else {
      point1 << nearest_collision_point.x(), nearest_collision_point.y();
      point2 << input.points.at(i + 1).point.pose.position.x,
        input.points.at(i + 1).point.pose.position.y;
      length_sum -= (point2 - point1).norm();
      for (size_t j = i + 1; j < input.points.size() - 1; ++j) {
        if (length_sum < target_length) {
          target_point_idx = j;
          break;
        }
        point1 << input.points.at(j).point.pose.position.x,
          input.points.at(j).point.pose.position.y;
        point2 << input.points.at(j + 1).point.pose.position.x,
          input.points.at(j + 1).point.pose.position.y;
        length_sum -= (point2 - point1).norm();
      }
    }
    // create target point
    getBackwordPointFromBasePoint(
      point2, point1, point2, std::fabs(length_sum - target_length), target_point);
    return true;
  }
  return false;
}

bool TrafficLightModule::getBackwordPointFromBasePoint(
  const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
  const Eigen::Vector2d & base_point, const double backward_length, Eigen::Vector2d & output_point)
{
  Eigen::Vector2d line_vec = line_point2 - line_point1;
  Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
  output_point = base_point + backward_vec;
  return true;
}

bool TrafficLightModule::hasLamp(
  const autoware_perception_msgs::TrafficLightState & tl_state, const uint8_t & lamp_color)
{
  const auto it_lamp = std::find_if(
    tl_state.lamp_states.begin(), tl_state.lamp_states.end(),
    [&lamp_color](const auto & x) { return x.type == lamp_color; });

  return it_lamp != tl_state.lamp_states.end();
}

geometry_msgs::Point TrafficLightModule::getTrafficLightPosition(
  const lanelet::ConstLineStringOrPolygon3d traffic_light)
{
  geometry_msgs::Point tl_center;
  for (const auto tl_point : *traffic_light.lineString()) {
    tl_center.x += tl_point.x() / (*traffic_light.lineString()).size();
    tl_center.y += tl_point.y() / (*traffic_light.lineString()).size();
    tl_center.z += tl_point.z() / (*traffic_light.lineString()).size();
  }
  return tl_center;
}
