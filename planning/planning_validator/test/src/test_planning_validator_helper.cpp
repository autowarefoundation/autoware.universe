// Copyright 2021 Tier IV, Inc.
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

#include "test_planning_validator_helper.hpp"

#include <math.h>

using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

Trajectory generateTrajectory(double interval_distance)
{
  Trajectory traj;
  for (double s = 0.0; s <= 10.0 * interval_distance; s += interval_distance) {
    TrajectoryPoint p;
    p.pose.position.x = s;
    p.longitudinal_velocity_mps = 1.0;
    traj.points.push_back(p);
  }
  return traj;
}

Trajectory generateNanTrajectory()
{
  Trajectory traj = generateTrajectory(1.0);
  traj.points.front().pose.position.x = NAN;
  return traj;
}

Trajectory generateInfTrajectory()
{
  Trajectory traj = generateTrajectory(1.0);
  traj.points.front().pose.position.x = INFINITY;
  return traj;
}

Trajectory generateBadCurvatureTrajectory()
{
  Trajectory traj;

  double y = 1.5;
  for (double s = 0.0; s <= 10.0; s += 1.0) {
    TrajectoryPoint p;
    p.longitudinal_velocity_mps = 1.0;
    p.pose.position.x = s;
    p.pose.position.y = y;
    y *= -1.0;  // invert sign
    traj.points.push_back(p);
  }

  return traj;
}

rclcpp::NodeOptions getNodeOptionsWithDefaultParams()
{
  rclcpp::NodeOptions node_options;

  // for planing validator
  node_options.append_parameter_override("publish_diag", true);
  node_options.append_parameter_override("use_previous_trajectory_on_invalid", true);
  node_options.append_parameter_override("interval_threshold", ERROR_INTERVAL);
  node_options.append_parameter_override("relative_angle_threshold", 1.0);
  node_options.append_parameter_override("curvature_threshold", ERROR_CURVATURE);
  node_options.append_parameter_override("lateral_acc_threshold", 100.0);
  node_options.append_parameter_override("longitudinal_max_acc_threshold", 100.0);
  node_options.append_parameter_override("longitudinal_min_acc_threshold", -100.0);
  node_options.append_parameter_override("steering_threshold", 100.0);
  node_options.append_parameter_override("steering_rate_threshold", 100.0);
  node_options.append_parameter_override("velocity_deviation_threshold", 100.0);
  node_options.append_parameter_override("distance_deviation_threshold", 100.0);

  // for vehicle info
  node_options.append_parameter_override("wheel_radius", 0.5);
  node_options.append_parameter_override("wheel_width", 0.2);
  node_options.append_parameter_override("wheel_base", 3.0);
  node_options.append_parameter_override("wheel_tread", 2.0);
  node_options.append_parameter_override("front_overhang", 1.0);
  node_options.append_parameter_override("rear_overhang", 1.0);
  node_options.append_parameter_override("left_overhang", 0.5);
  node_options.append_parameter_override("right_overhang", 0.5);
  node_options.append_parameter_override("vehicle_height", 1.5);
  node_options.append_parameter_override("max_steer_angle", 0.7);

  return node_options;
}
