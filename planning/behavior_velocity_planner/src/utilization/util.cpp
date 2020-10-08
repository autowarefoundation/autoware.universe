/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 *
 * Author: Robin Karlsson
 */

#include "utilization/util.h"

namespace planning_utils
{
double normalizeEulerAngle(double euler)
{
  double res = euler;
  while (res > M_PI) {
    res -= (2.0 * M_PI);
  }
  while (res < -M_PI) {
    res += 2.0 * M_PI;
  }

  return res;
}

geometry_msgs::Quaternion getQuaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

template <class T>
bool calcClosestIndex(
  const T & path, const geometry_msgs::Pose & pose, int & closest, double dist_thr,
  double angle_thr)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  double yaw_pose = tf2::getYaw(pose.orientation);
  closest = -1;

  for (int i = 0; i < static_cast<int>(path.points.size()); ++i) {
    const double dist_squared = calcSquaredDist2d(getPose(path, i), pose);

    /* check distance threshold */
    if (dist_squared > dist_thr * dist_thr) continue;

    /* check angle threshold */
    double yaw_i = tf2::getYaw(getPose(path, i).orientation);
    double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_i);

    if (std::fabs(yaw_diff) > angle_thr) continue;

    if (dist_squared < dist_squared_min) {
      dist_squared_min = dist_squared;
      closest = i;
    }
  }

  return closest == -1 ? false : true;
}

template bool calcClosestIndex<autoware_planning_msgs::Trajectory>(
  const autoware_planning_msgs::Trajectory & path, const geometry_msgs::Pose & pose, int & closest,
  double dist_thr, double angle_thr);
template bool calcClosestIndex<autoware_planning_msgs::PathWithLaneId>(
  const autoware_planning_msgs::PathWithLaneId & path, const geometry_msgs::Pose & pose,
  int & closest, double dist_thr, double angle_thr);
template bool calcClosestIndex<autoware_planning_msgs::Path>(
  const autoware_planning_msgs::Path & path, const geometry_msgs::Pose & pose, int & closest,
  double dist_thr, double angle_thr);

template <class T>
bool calcClosestIndex(
  const T & path, const geometry_msgs::Point & point, int & closest, double dist_thr)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  closest = -1;

  for (int i = 0; i < static_cast<int>(path.points.size()); ++i) {
    const double dist_squared = calcSquaredDist2d(getPose(path, i), point);

    /* check distance threshold */
    if (dist_squared > dist_thr * dist_thr) continue;

    if (dist_squared < dist_squared_min) {
      dist_squared_min = dist_squared;
      closest = i;
    }
  }

  return closest == -1 ? false : true;
}
template bool calcClosestIndex<autoware_planning_msgs::Trajectory>(
  const autoware_planning_msgs::Trajectory & path, const geometry_msgs::Point & point,
  int & closest, double dist_thr);
template bool calcClosestIndex<autoware_planning_msgs::PathWithLaneId>(
  const autoware_planning_msgs::PathWithLaneId & path, const geometry_msgs::Point & point,
  int & closest, double dist_thr);
template bool calcClosestIndex<autoware_planning_msgs::Path>(
  const autoware_planning_msgs::Path & path, const geometry_msgs::Point & point, int & closest,
  double dist_thr);

geometry_msgs::Pose transformRelCoordinate2D(
  const geometry_msgs::Pose & target, const geometry_msgs::Pose & origin)
{
  // translation
  geometry_msgs::Point trans_p;
  trans_p.x = target.position.x - origin.position.x;
  trans_p.y = target.position.y - origin.position.y;

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);

  geometry_msgs::Pose res;
  res.position.x = (std::cos(yaw) * trans_p.x) + (std::sin(yaw) * trans_p.y);
  res.position.y = ((-1.0) * std::sin(yaw) * trans_p.x) + (std::cos(yaw) * trans_p.y);
  res.position.z = target.position.z - origin.position.z;
  res.orientation = getQuaternionFromYaw(tf2::getYaw(target.orientation) - yaw);

  return res;
}

geometry_msgs::Pose transformAbsCoordinate2D(
  const geometry_msgs::Pose & relative, const geometry_msgs::Pose & origin)
{
  // rotation
  geometry_msgs::Point rot_p;
  double yaw = tf2::getYaw(origin.orientation);
  rot_p.x = (std::cos(yaw) * relative.position.x) + (-std::sin(yaw) * relative.position.y);
  rot_p.y = (std::sin(yaw) * relative.position.x) + (std::cos(yaw) * relative.position.y);

  // translation
  geometry_msgs::Pose absolute;
  absolute.position.x = rot_p.x + origin.position.x;
  absolute.position.y = rot_p.y + origin.position.y;
  absolute.position.z = relative.position.z + origin.position.z;
  absolute.orientation = getQuaternionFromYaw(tf2::getYaw(relative.orientation) + yaw);

  return absolute;
}

double calcJudgeLineDist(
  const double velocity, const double max_stop_acceleration,
  const double delay_response_time)  // TODO: also consider jerk
{
  double judge_line_dist =
    (velocity * velocity) / (2.0 * (-max_stop_acceleration)) + delay_response_time * velocity;
  return judge_line_dist;
}

autoware_planning_msgs::StopReason initializeStopReason(const std::string & stop_reason)
{
  autoware_planning_msgs::StopReason stop_reason_msg;
  stop_reason_msg.reason = stop_reason;
  return stop_reason_msg;
}

void appendStopReason(
  const autoware_planning_msgs::StopFactor stop_factor,
  autoware_planning_msgs::StopReason * stop_reason)
{
  stop_reason->stop_factors.emplace_back(stop_factor);
}

std::vector<geometry_msgs::Point> toRosPoints(
  const autoware_perception_msgs::DynamicObjectArray & object)
{
  std::vector<geometry_msgs::Point> points;
  for (const auto obj : object.objects) {
    points.emplace_back(obj.state.pose_covariance.pose.position);
  }
  return points;
}

geometry_msgs::Point toRosPoint(const pcl::PointXYZ & pcl_point)
{
  geometry_msgs::Point point;
  point.x = pcl_point.x;
  point.y = pcl_point.y;
  point.z = pcl_point.z;
  return point;
}

geometry_msgs::Point toRosPoint(const Point2d & boost_point, const double z)
{
  geometry_msgs::Point point;
  point.x = boost_point.x();
  point.y = boost_point.y();
  point.z = z;
  return point;
}

}  // namespace planning_utils
