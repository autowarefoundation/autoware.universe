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
 */

#ifndef COMMON_MATH_PLANNING_UTILS_H
#define COMMON_MATH_PLANNING_UTILS_H

#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/utils.h>
#include <vector>

namespace planning_utils
{
inline geometry_msgs::Point getPoint(const geometry_msgs::Point & p) { return p; }
inline geometry_msgs::Point getPoint(const geometry_msgs::Pose & p) { return p.position; }
inline geometry_msgs::Point getPoint(const geometry_msgs::PoseStamped & p)
{
  return p.pose.position;
}
inline geometry_msgs::Point getPoint(const autoware_planning_msgs::PathPoint & p)
{
  return p.pose.position;
}
inline geometry_msgs::Point getPoint(const autoware_planning_msgs::TrajectoryPoint & p)
{
  return p.pose.position;
}
inline geometry_msgs::Pose getPose(const autoware_planning_msgs::Path & path, int idx)
{
  return path.points.at(idx).pose;
}
inline geometry_msgs::Pose getPose(const autoware_planning_msgs::PathWithLaneId & path, int idx)
{
  return path.points.at(idx).point.pose;
}
inline geometry_msgs::Pose getPose(const autoware_planning_msgs::Trajectory & traj, int idx)
{
  return traj.points.at(idx).pose;
}

inline double square(const double & a) { return a * a; }
double normalizeEulerAngle(double euler);
geometry_msgs::Quaternion getQuaternionFromYaw(double yaw);

template <class T1, class T2>
double calcSquaredDist2d(const T1 & a, const T2 & b)
{
  return square(getPoint(a).x - getPoint(b).x) + square(getPoint(a).y - getPoint(b).y);
}

template <class T1, class T2>
double calcDist2d(const T1 & a, const T2 & b)
{
  return std::sqrt(calcSquaredDist2d<T1, T2>(a, b));
}

template <class T>
bool calcClosestIndex(
  const T & path, const geometry_msgs::Pose & pose, int & closest, double dist_thr = 3.0,
  double angle_thr = M_PI_4);

geometry_msgs::Pose transformRelCoordinate2D(
  const geometry_msgs::Pose & target, const geometry_msgs::Pose & origin);
geometry_msgs::Pose transformAbsCoordinate2D(
  const geometry_msgs::Pose & relative, const geometry_msgs::Pose & origin);

double calcJudgeLineDist(double velocity, double max_accel, double margin);

}  // namespace planning_utils

#endif  // COMMON_MATH_PLANNING_UTILS_H
