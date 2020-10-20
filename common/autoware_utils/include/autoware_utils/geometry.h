/*
 * Copyright 2020 TierIV. All rights reserved.
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

#pragma once

#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

namespace autoware_utils
{
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Point & p) { return p; }
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Pose & p) { return p.position; }
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::PoseStamped & p)
{
  return p.pose.position;
}

inline geometry_msgs::msg::Point getPoint(const autoware_planning_msgs::msg::PathPoint & p)
{
  return p.pose.position;
}
inline geometry_msgs::msg::Point getPoint(const autoware_planning_msgs::msg::TrajectoryPoint & p)
{
  return p.pose.position;
}

inline geometry_msgs::msg::Point createPoint(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

inline tf2::Quaternion createQuaternionFromRPY(
  const double roll, const double pitch, const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return q;
}

template <class Point1, class Point2>
double calcDistance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

template <class Point1, class Point2>
double calcDistance3d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  // To be replaced by std::hypot(dx, dy, dz) in C++17
  return std::hypot(std::hypot(p1.x - p2.x, p1.y - p2.y), p1.z - p2.z);
}
}  // namespace autoware_utils
