// Copyright 2022 Tier IV, Inc.
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

#include "scene_module/dynamic_obstacle_stop/utils.hpp"

namespace behavior_velocity_planner
{
namespace dynamic_obstacle_stop_utils
{
bool validCheckDecelPlan(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin)
{
  const double v_min = v_target - std::abs(v_margin);
  const double v_max = v_target + std::abs(v_margin);
  const double a_min = a_target - std::abs(a_margin);
  const double a_max = a_target + std::abs(a_margin);

  if (v_end < v_min || v_max < v_end) {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("validCheckDecelPlan"),
      "[validCheckDecelPlan] valid check error! v_target = " << v_target << ", v_end = " << v_end);
    return false;
  }
  if (a_end < a_min || a_max < a_end) {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("validCheckDecelPlan"),
      "[validCheckDecelPlan] valid check error! a_target = " << a_target << ", a_end = " << a_end);
    return false;
  }

  return true;
}
/**
 * @brief calculate distance until velocity is reached target velocity (TYPE1)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (am) minimum deceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @param (jd) minimum jerk [m/sss]
 * @param (t_min) duration of constant deceleration [s]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType1(
  const double v0, const double vt, const double a0, const double am, const double ja,
  const double jd, const double t_min)
{
  constexpr double epsilon = 1e-3;

  const double j1 = am < a0 ? jd : ja;
  const double t1 = epsilon < (am - a0) / j1 ? (am - a0) / j1 : 0.0;
  const double a1 = a0 + j1 * t1;
  const double v1 = v0 + a0 * t1 + 0.5 * j1 * t1 * t1;
  const double x1 = v0 * t1 + 0.5 * a0 * t1 * t1 + (1.0 / 6.0) * j1 * t1 * t1 * t1;

  const double t2 = epsilon < t_min ? t_min : 0.0;
  const double a2 = a1;
  const double v2 = v1 + a1 * t2;
  const double x2 = x1 + v1 * t2 + 0.5 * a1 * t2 * t2;

  const double t3 = epsilon < (0.0 - am) / ja ? (0.0 - am) / ja : 0.0;
  const double a3 = a2 + ja * t3;
  const double v3 = v2 + a2 * t3 + 0.5 * ja * t3 * t3;
  const double x3 = x2 + v2 * t3 + 0.5 * a2 * t3 * t3 + (1.0 / 6.0) * ja * t3 * t3 * t3;

  const double a_target = 0.0;
  const double v_margin = 0.3;  // [m/s]
  const double a_margin = 0.1;  // [m/s^2]

  if (!validCheckDecelPlan(v3, a3, vt, a_target, v_margin, a_margin)) {
    return {};
  }

  return x3;
}
/**
 * @brief calculate distance until velocity is reached target velocity (TYPE2)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (am) minimum deceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @param (jd) minimum jerk [m/sss]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType2(
  const double v0, const double vt, const double a0, const double ja, const double jd)
{
  constexpr double epsilon = 1e-3;

  const double a1_square = (vt - v0 - 0.5 * (0.0 - a0) / jd * a0) * (2.0 * ja * jd / (ja - jd));
  const double a1 = -std::sqrt(a1_square);

  const double t1 = epsilon < (a1 - a0) / jd ? (a1 - a0) / jd : 0.0;
  const double v1 = v0 + a0 * t1 + 0.5 * jd * t1 * t1;
  const double x1 = v0 * t1 + 0.5 * a0 * t1 * t1 + (1.0 / 6.0) * jd * t1 * t1 * t1;

  const double t2 = epsilon < (0.0 - a1) / ja ? (0.0 - a1) / ja : 0.0;
  const double a2 = a1 + ja * t2;
  const double v2 = v1 + a1 * t2 + 0.5 * ja * t2 * t2;
  const double x2 = x1 + v1 * t2 + 0.5 * a1 * t2 * t2 + (1.0 / 6.0) * ja * t2 * t2 * t2;

  const double a_target = 0.0;
  const double v_margin = 0.3;
  const double a_margin = 0.1;

  if (!validCheckDecelPlan(v2, a2, vt, a_target, v_margin, a_margin)) {
    return {};
  }

  return x2;
}
/**
 * @brief calculate distance until velocity is reached target velocity (TYPE3)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType3(
  const double v0, const double vt, const double a0, const double ja)
{
  constexpr double epsilon = 1e-3;

  const double t_acc = (0.0 - a0) / ja;

  const double t1 = epsilon < t_acc ? t_acc : 0.0;
  const double a1 = a0 + ja * t1;
  const double v1 = v0 + a0 * t1 + 0.5 * ja * t1 * t1;
  const double x1 = v0 * t1 + 0.5 * a0 * t1 * t1 + (1.0 / 6.0) * ja * t1 * t1 * t1;

  const double a_target = 0.0;
  const double v_margin = 0.3;
  const double a_margin = 0.1;

  if (!validCheckDecelPlan(v1, a1, vt, a_target, v_margin, a_margin)) {
    return {};
  }

  return x1;
}
boost::optional<double> calcDecelDistWithJerkAndAccConstraints(
  const double current_vel, const double target_vel, const double current_acc, const double acc_min,
  const double jerk_acc, const double jerk_dec)
{
  constexpr double epsilon = 1e-3;
  const double t_dec =
    acc_min < current_acc ? (acc_min - current_acc) / jerk_dec : (acc_min - current_acc) / jerk_acc;
  const double t_acc = (0.0 - acc_min) / jerk_acc;
  const double t_min = (target_vel - current_vel - current_acc * t_dec -
                        0.5 * jerk_dec * t_dec * t_dec - 0.5 * acc_min * t_acc) /
                       acc_min;

  // check if it is possible to decelerate to the target velocity
  // by simply bringing the current acceleration to zero.
  const auto is_decel_needed =
    0.5 * (0.0 - current_acc) / jerk_acc * current_acc > target_vel - current_vel;

  if (t_min > epsilon) {
    return calcDecelDistPlanType1(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec, t_min);
  } else if (is_decel_needed || current_acc > epsilon) {
    return calcDecelDistPlanType2(current_vel, target_vel, current_acc, jerk_acc, jerk_dec);
  } else {
    return calcDecelDistPlanType3(current_vel, target_vel, current_acc, jerk_acc);
  }

  return {};
}

// void applyMaximumVelocityLimit(const double max_vel_mps, Trajectory & trajectory)
// {
//   for (size_t idx = 0; idx < trajectory.points.size(); ++idx) {
//     if (trajectory.points.at(idx).longitudinal_velocity_mps > max_vel_mps) {
//       trajectory.points.at(idx).longitudinal_velocity_mps = max_vel_mps;
//     }
//   }
// }

Polygon2d createBoostPolyFromMsg(const std::vector<geometry_msgs::msg::Point> & input_poly)
{
  Polygon2d bg_poly;
  for (const auto & p : input_poly) {
    bg_poly.outer().emplace_back(bg::make<Point2d>(p.x, p.y));
  }

  // one more point to close polygon
  const auto & first_point = input_poly.front();
  bg_poly.outer().emplace_back(bg::make<Point2d>(first_point.x, first_point.y));

  return bg_poly;
}

std::uint8_t getHighestProbLabel(const std::vector<ObjectClassification> & classification)
{
  std::uint8_t label = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
  float highest_prob = 0.0;
  for (const auto & _class : classification) {
    if (highest_prob < _class.probability) {
      highest_prob = _class.probability;
      label = _class.label;
    }
  }
  return label;
}

std::vector<geometry_msgs::msg::Pose> getHighestConfidencePath(
  const std::vector<DynamicObstacle::PredictedPath> & predicted_paths)
{
  std::vector<geometry_msgs::msg::Pose> predicted_path{};
  float highest_confidence = 0.0;
  for (const auto & path : predicted_paths) {
    if (path.confidence > highest_confidence) {
      highest_confidence = path.confidence;
      predicted_path = path.path;
    }
  }

  return predicted_path;
}

// apply linear interpolation to position
geometry_msgs::msg::Pose lerpByPose(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2, const float t)
{
  tf2::Transform tf_transform1, tf_transform2;
  tf2::fromMsg(p1, tf_transform1);
  tf2::fromMsg(p2, tf_transform2);
  const auto & tf_point = tf2::lerp(tf_transform1.getOrigin(), tf_transform2.getOrigin(), t);

  geometry_msgs::msg::Pose pose;
  pose.position.x = tf_point.getX();
  pose.position.y = tf_point.getY();
  pose.position.z = tf_point.getZ();
  pose.orientation = p1.orientation;
  return pose;
}

geometry_msgs::msg::Point findLongitudinalNearestPoint(
  const Trajectory & trajectory, const geometry_msgs::msg::Point & src_point,
  const std::vector<geometry_msgs::msg::Point> & target_points)
{
  float min_dist = std::numeric_limits<float>::max();
  geometry_msgs::msg::Point min_dist_point{};

  for (const auto & p : target_points) {
    const float dist = tier4_autoware_utils::calcSignedArcLength(trajectory.points, src_point, p);
    if (dist < min_dist) {
      min_dist = dist;
      min_dist_point = p;
    }
  }

  return min_dist_point;
}

std::vector<geometry_msgs::msg::Point> findLateralSameSidePoints(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & base_pose,
  const geometry_msgs::msg::Point & target_point)
{
  const auto signed_deviation = tier4_autoware_utils::calcLateralDeviation(base_pose, target_point);
  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("findLateralSameSidePoints"), "signed dev of target: " << signed_deviation);

  std::vector<geometry_msgs::msg::Point> same_side_points;
  for (const auto & p : points) {
    const auto signed_deviation_of_point = tier4_autoware_utils::calcLateralDeviation(base_pose, p);
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("findLateralSameSidePoints"),
      "signed dev of point: " << signed_deviation_of_point);
    if (signed_deviation * signed_deviation_of_point > 0) {
      same_side_points.emplace_back(p);
    }
  }

  if (same_side_points.empty()) {
    return points;
  }

  return same_side_points;
}

TextWithPosition createDebugText(
  const std::string text, const geometry_msgs::msg::Pose pose, const float lateral_offset)
{
  const auto offset_pose = tier4_autoware_utils::calcOffsetPose(pose, 0, lateral_offset, 0);

  TextWithPosition text_with_position;
  text_with_position.text = text;
  text_with_position.position = offset_pose.position;

  return text_with_position;
}

TextWithPosition createDebugText(const std::string text, const geometry_msgs::msg::Point position)
{
  TextWithPosition text_with_position;
  text_with_position.text = text;
  text_with_position.position = position;

  return text_with_position;
}

Trajectory convertPathToTrajectory(const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  Trajectory trajectory;
  trajectory.header = path.header;
  for (const auto & p : path.points) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint tmp_point;
    tmp_point.pose = p.point.pose;
    tmp_point.longitudinal_velocity_mps = p.point.longitudinal_velocity_mps;
    trajectory.points.push_back(tmp_point);
  }

  return trajectory;
}

std::vector<geometry_msgs::msg::Point> toRosPoints(const PathPointsWithLaneId & path_points)
{
  std::vector<geometry_msgs::msg::Point> ros_points;
  for (const auto & p : path_points) {
    ros_points.push_back(tier4_autoware_utils::createPoint(
      p.point.pose.position.x, p.point.pose.position.y, p.point.pose.position.z));
  }

  return ros_points;
}

size_t findNearestIndex(
  const PathPointsWithLaneId & path_points, const geometry_msgs::msg::Point & point)
{
  const auto points = toRosPoints(path_points);

  return tier4_autoware_utils::findNearestIndex(points, point);
}

size_t findNearestSegmentIndex(
  const PathPointsWithLaneId & path_points, const geometry_msgs::msg::Point & point)
{
  const auto points = toRosPoints(path_points);

  return tier4_autoware_utils::findNearestSegmentIndex(points, point);
}

// if path points have the same point as target_point, return the index
boost::optional<size_t> haveSamePoint(
  const PathPointsWithLaneId & path_points, const geometry_msgs::msg::Point & target_point)
{
  for (size_t i = 0; i < path_points.size(); i++) {
    const auto & path_point = path_points.at(i).point.pose.position;
    if (isSamePoint(path_point, target_point)) {
      return i;
    }
  }

  return {};
}

bool isSamePoint(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  if (tier4_autoware_utils::calcDistance2d(p1, p2) < std::numeric_limits<float>::epsilon()) {
    return true;
  }

  return false;
}

// insert path velocity which doesn't exceed original velocity
void insertPathVelocityFromIndexLimited(
  const size_t & start_idx, const float velocity_mps, PathPointsWithLaneId & path_points)
{
  for (size_t i = start_idx; i < path_points.size(); i++) {
    const auto current_path_vel = path_points.at(i).point.longitudinal_velocity_mps;
    path_points.at(i).point.longitudinal_velocity_mps = std::min(velocity_mps, current_path_vel);
  }
}

void insertPathVelocityFromIndex(
  const size_t & start_idx, const float velocity_mps, PathPointsWithLaneId & path_points)
{
  for (size_t i = start_idx; i < path_points.size(); i++) {
    path_points.at(i).point.longitudinal_velocity_mps = velocity_mps;
  }
}

boost::optional<size_t> findFirstStopPointIdx(PathPointsWithLaneId & path_points)
{
  for (size_t i = 0; i < path_points.size(); i++) {
    const auto vel = path_points.at(i).point.longitudinal_velocity_mps;
    if (vel < std::numeric_limits<float>::epsilon()) {
      return i;
    }
  }

  return {};
}

std::vector<DynamicObstacle> excludeObstaclesOutSideOfLine(
  const std::vector<DynamicObstacle> & dynamic_obstacles, const Trajectory & trajectory,
  const lanelet::BasicPolygon2d & partition)
{
  std::vector<DynamicObstacle> extracted_dynamic_obstacle;
  for (const auto & obstacle : dynamic_obstacles) {
    const auto obstacle_nearest_idx =
      tier4_autoware_utils::findNearestIndex(trajectory.points, obstacle.pose_.position);
    const auto & obstacle_nearest_traj_point =
      trajectory.points.at(obstacle_nearest_idx).pose.position;

    // create linestring from traj point to obstacle
    const LineString2d traj_p_to_obstacle{
      {obstacle_nearest_traj_point.x, obstacle_nearest_traj_point.y},
      {obstacle.pose_.position.x, obstacle.pose_.position.y}};

    // create linestring for partition
    const LineString2d partition_bg = createLineString2d(partition);

    // ignore obstacle outside of partition
    if (bg::intersects(traj_p_to_obstacle, partition_bg)) {
      continue;
    }
    extracted_dynamic_obstacle.emplace_back(obstacle);
  }

  return extracted_dynamic_obstacle;
}

Trajectory decimateTrajectory(const Trajectory & input_traj, const float step)
{
  if (input_traj.points.empty()) {
    return Trajectory();
  }

  float dist_sum = 0.0;
  Trajectory decimate_traj;
  decimate_traj.header = input_traj.header;
  // push first point
  decimate_traj.points.emplace_back(input_traj.points.front());

  for (size_t i = 1; i < input_traj.points.size(); i++) {
    const auto p1 = input_traj.points.at(i - 1);
    const auto p2 = input_traj.points.at(i);
    const auto dist = tier4_autoware_utils::calcDistance2d(p1.pose.position, p2.pose.position);
    dist_sum += dist;

    if (dist_sum > step) {
      decimate_traj.points.emplace_back(p2);
      dist_sum = 0.0;
    }
  }

  return decimate_traj;
}

LineString2d createLineString2d(const lanelet::BasicPolygon2d & poly)
{
  LineString2d line_string;
  for (const auto & p : poly) {
    Point2d bg_point{p.x(), p.y()};
    line_string.push_back(bg_point);
  }

  return line_string;
}

// trim path from self_pose to trim_distance
PathWithLaneId trimPathFromSelfPose(
  const PathWithLaneId & input, const geometry_msgs::msg::Pose & self_pose,
  const double trim_distance) const
{
  // findNearestSegmentIndex finds the index behind of the specified point
  // so add 1 to the index to get the nearest point ahead of the ego
  // const size_t nearest_idx =
  //   tier4_autoware_utils::findNearestSegmentIndex(input.points, self_pose.position) + 1;

  const size_t nearest_idx =
    tier4_autoware_utils::findNearestIndex(input.points, self_pose.position);

  Trajectory output{};
  double dist_sum = 0;
  for (size_t i = nearest_idx; i < input.points.size(); ++i) {
    output.points.push_back(input.points.at(i));

    if (i != nearest_idx) {
      dist_sum += tier4_autoware_utils::calcDistance2d(input.points.at(i - 1), input.points.at(i));
    }

    if (dist_sum > trim_distance) {
      break;
    }
  }
  output.header = input.header;

  return output;
}

}  // namespace dynamic_obstacle_stop_utils
}  // namespace behavior_velocity_planner
