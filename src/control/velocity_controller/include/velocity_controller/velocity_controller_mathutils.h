/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef VELOCITY_CONTROLLER_MATHUTILS
#define VELOCITY_CONTROLLER_MATHUTILS

#include <cmath>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include "autoware_planning_msgs/Trajectory.h"

namespace vcutils
{
double calcDistance2D(const geometry_msgs::Pose & p1, const geometry_msgs::Pose & p2);
double calcDistSquared2D(const geometry_msgs::Pose & p1, const geometry_msgs::Pose & p2);
double normalizeEulerAngle(double euler);
bool calcClosestWithThr(
  const autoware_planning_msgs::Trajectory & trajectory, const geometry_msgs::Pose & pose,
  const double angle_thr, const double dist_thr, int32_t & closest_idx);
geometry_msgs::Point transformToRelativeCoordinate2D(
  const geometry_msgs::Point & point, const geometry_msgs::Pose & origin);
}  // namespace vcutils

#endif