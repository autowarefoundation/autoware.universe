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

#ifndef VELOCITY_CONTROLLER_DELAY_COMPENSATION
#define VELOCITY_CONTROLLER_DELAY_COMPENSATION

#include <autoware_planning_msgs/Trajectory.h>
#include "velocity_controller_mathutils.h"

class DelayCompensator
{
public:
  DelayCompensator();
  ~DelayCompensator();

  static int getTrajectoryPointIndexAfterTimeDelay(
    const autoware_planning_msgs::Trajectory & trajectory, int32_t closest_waypoint_index,
    double delay_time, double current_velocity)
  {
    const double delay_distance = current_velocity * delay_time;

    double sum_distance = 0.0;
    for (unsigned int i = closest_waypoint_index; i < trajectory.points.size() - 1; ++i) {
      sum_distance +=
        vcutils::calcDistance2D(trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose);
      if (sum_distance > delay_distance) {
        return i;
      }
    }
    return trajectory.points.size() - 1;
  }

  static autoware_planning_msgs::TrajectoryPoint getTrajectoryPointAfterTimeDelay(
    const autoware_planning_msgs::Trajectory & trajectory, int32_t closest_waypoint_index,
    double delay_time, double current_velocity)
  {
    int target_waypoint_index = getTrajectoryPointIndexAfterTimeDelay(
      trajectory, closest_waypoint_index, delay_time, current_velocity);
    return trajectory.points.at(target_waypoint_index);
  }

  static double getAccelerationAfterTimeDelay(
    const autoware_planning_msgs::Trajectory & trajectory, int32_t closest_waypoint_index,
    double delay_time, double current_velocity)
  {
    auto point_after_delay = getTrajectoryPointAfterTimeDelay(
      trajectory, closest_waypoint_index, delay_time, current_velocity);
    return point_after_delay.accel.linear.x;
  }

  static double getVelocityAfterTimeDelay(
    const autoware_planning_msgs::Trajectory & trajectory, int32_t closest_waypoint_index,
    double delay_time, double current_velocity)
  {
    auto point_after_delay = getTrajectoryPointAfterTimeDelay(
      trajectory, closest_waypoint_index, delay_time, current_velocity);
    return point_after_delay.twist.linear.x;
  }
};

#endif
