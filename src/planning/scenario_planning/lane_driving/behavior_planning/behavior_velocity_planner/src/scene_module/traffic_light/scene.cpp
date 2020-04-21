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

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Line = bg::model::linestring<Point>;
using Polygon = bg::model::polygon<Point, false>;

TrafficLightModule::TrafficLightModule(
  const int64_t module_id, const lanelet::TrafficLight & traffic_light_reg_elem)
: SceneModuleInterface(module_id),
  traffic_light_reg_elem_(traffic_light_reg_elem),
  state_(State::APPROARCH)
{
}

bool TrafficLightModule::modifyPathVelocity(autoware_planning_msgs::PathWithLaneId * path)
{
  debug_data_ = {};
  debug_data_.base_link2front = planner_data_->base_link2front;

  const auto input_path = *path;

  // get lanelet2 traffic light
  lanelet::ConstLineString3d lanelet_stop_line = *(traffic_light_reg_elem_.stopLine());
  lanelet::ConstLineStringsOrPolygons3d traffic_lights = traffic_light_reg_elem_.trafficLights();

  // get vehicle info
  geometry_msgs::TwistStamped::ConstPtr self_twist_ptr = planner_data_->current_velocity;

  const double stop_border_distance_threshold =
    (-1.0 * self_twist_ptr->twist.linear.x * self_twist_ptr->twist.linear.x) /
    (2.0 * max_stop_acceleration_threshold_);
  geometry_msgs::PoseStamped self_pose = planner_data_->current_pose;

  // check state
  if (state_ == State::GO_OUT) {
    return true;
  } else if (state_ == State::APPROARCH) {
    for (int i = 0; i < lanelet_stop_line.size() - 1; i++) {
      Line stop_line = {{lanelet_stop_line[i].x(), lanelet_stop_line[i].y()},
                        {lanelet_stop_line[i + 1].x(), lanelet_stop_line[i + 1].y()}};
      // update state
      {
        Eigen::Vector2d judge_point;
        size_t judge_point_idx;
        if (!createTargetPoint(
              input_path, stop_line, -2.0 /*overline margin*/, judge_point_idx, judge_point))
          continue;
        const double sq_dist = (judge_point.x() - self_pose.pose.position.x) *
                                 (judge_point.x() - self_pose.pose.position.x) +
                               (judge_point.y() - self_pose.pose.position.y) *
                                 (judge_point.y() - self_pose.pose.position.y);
        const double range = 5.0;
        if (sq_dist < range * range) {
          double yaw;
          if (judge_point_idx == 0)
            yaw = std::atan2(
              input_path.points.at(judge_point_idx + 1).point.pose.position.y - judge_point.y(),
              input_path.points.at(judge_point_idx + 1).point.pose.position.x - judge_point.x());
          else
            yaw = std::atan2(
              judge_point.y() - input_path.points.at(judge_point_idx - 1).point.pose.position.y,
              judge_point.x() - input_path.points.at(judge_point_idx - 1).point.pose.position.x);
          tf2::Quaternion quat;
          quat.setRPY(0, 0, yaw);
          tf2::Transform tf_map2judge_pose(
            quat, tf2::Vector3(judge_point.x(), judge_point.y(), self_pose.pose.position.z));
          tf2::Transform tf_map2self_pose;
          tf2::Transform tf_judge_pose2self_pose;
          tf2::fromMsg(self_pose.pose, tf_map2self_pose);
          tf_judge_pose2self_pose = tf_map2judge_pose.inverse() * tf_map2self_pose;
          // -- debug code --
          geometry_msgs::Pose judge_pose;
          tf2::toMsg(tf_map2judge_pose, judge_pose);
          debug_data_.judge_poses.push_back(judge_pose);
          // ----------------
          if (0 < tf_judge_pose2self_pose.getOrigin().x()) {
            state_ = State::GO_OUT;
            return true;
          }
        }
      }

      // search traffic light state
      autoware_perception_msgs::TrafficLightState highest_confidence_tl_state;
      if (!getHighestConfidenceTrafficLightState(traffic_lights, highest_confidence_tl_state))
        continue;
      // check stop border distance
      Eigen::Vector2d stop_line_point;
      size_t stop_line_point_idx;
      if (!createTargetPoint(
            input_path, stop_line, stop_margin_, stop_line_point_idx, stop_line_point))
        continue;
      Eigen::Vector2d self_point;
      self_point << self_pose.pose.position.x, self_pose.pose.position.y;
      const double sq_dist =
        (self_point.x() - stop_line_point.x()) * (self_point.x() - stop_line_point.x()) +
        (self_point.y() - stop_line_point.y()) * (self_point.y() - stop_line_point.y());
      if (sq_dist < stop_border_distance_threshold * stop_border_distance_threshold) {
        ROS_WARN_THROTTLE(
          1.0,
          "[traffic_light] state is red. this vehicle are passing too fast to stop "
          "at the stop line");
        return true;
      }

      // if state is red, insert stop point into path
      if (
        highest_confidence_tl_state.lamp_states.front().type ==
        autoware_perception_msgs::LampState::RED) {
        if (!insertTargetVelocityPoint(input_path, stop_line, stop_margin_, 0.0, *path)) {
          continue;
        }
      }
      // not go to continue in any index, return true
      return true;
    }
  }
  // all index in for() go to continue, return false
  return false;
}

bool TrafficLightModule::getHighestConfidenceTrafficLightState(
  lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
  autoware_perception_msgs::TrafficLightState & highest_confidence_tl_state)
{
  // search traffic light state
  bool found = false;
  double highest_confidence = 0.0;
  for (const auto & traffic_light : traffic_lights) {
    // traffic ligth must be linestrings
    if (!traffic_light.isLineString()) {
      continue;
    }
    const int id = static_cast<lanelet::ConstLineString3d>(traffic_light).id();
    const auto tl_state_stamped = planner_data_->getTrafficLightState(id);
    if (!tl_state_stamped) {
      continue;
    }
    const auto header = tl_state_stamped->header;
    const auto tl_state = tl_state_stamped->traffic_light_state;

    if (!((ros::Time::now() - header.stamp).toSec() < tl_state_timeout_)) {
      continue;
    }
    if (
      tl_state.lamp_states.empty() ||
      tl_state.lamp_states.front().type == autoware_perception_msgs::LampState::UNKNOWN) {
      continue;
    }

    if (highest_confidence < tl_state.lamp_states.front().confidence) {
      highest_confidence = tl_state.lamp_states.front().confidence;
      highest_confidence_tl_state = tl_state;
    }
    found = true;
  }
  if (!found) {
    ROS_WARN_THROTTLE(1.0, "[traffic_light] cannot find traffic light lamp state.");
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
  target_point_with_lane_id =
    output.points.at(std::max(static_cast<int>(insert_target_point_idx - 1), 0));
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
