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

#include <iostream>

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
#include <boost/shared_ptr.hpp>

#include <autoware_planning_msgs/Trajectory.h>

namespace vpu
{
double square(const double & a);
double calcSquaredDist2d(const geometry_msgs::Point & a, const geometry_msgs::Point & b);
double calcSquaredDist2d(const geometry_msgs::Pose & a, const geometry_msgs::Pose & b);
double calcSquaredDist2d(
  const geometry_msgs::PoseStamped & a, const geometry_msgs::PoseStamped & b);
double calcSquaredDist2d(
  const autoware_planning_msgs::TrajectoryPoint & a,
  const autoware_planning_msgs::TrajectoryPoint & b);
double calcDist2d(const geometry_msgs::Point & a, const geometry_msgs::Point & b);
double calcDist2d(const geometry_msgs::Pose & a, const geometry_msgs::Pose & b);
double calcDist2d(const geometry_msgs::PoseStamped & a, const geometry_msgs::PoseStamped & b);
double calcDist2d(
  const autoware_planning_msgs::TrajectoryPoint & a,
  const autoware_planning_msgs::TrajectoryPoint & b);
int calcClosestWaypoint(
  const autoware_planning_msgs::Trajectory & trajectory, const geometry_msgs::Point & point);
int calcClosestWaypoint(
  const autoware_planning_msgs::Trajectory & trajectory, const geometry_msgs::Pose & pose,
  const double delta_yaw_threshold);
bool extractPathAroundIndex(
  const autoware_planning_msgs::Trajectory & trajectory, const int index,
  const double & ahead_length, const double & behind_length,
  autoware_planning_msgs::Trajectory & extracted_base_waypoints);
double calcLengthOnWaypoints(
  const autoware_planning_msgs::Trajectory & trajectory, const int idx1, const int idx2);
void calcTrajectoryArclength(
  const autoware_planning_msgs::Trajectory & trajectory, std::vector<double> & arclength);
void calcTrajectoryIntervalDistance(
  const autoware_planning_msgs::Trajectory & trajectory, std::vector<double> & intervals);
void setZeroVelocity(autoware_planning_msgs::Trajectory & trajectory);
void mininumVelocityFilter(const double & min_vel, autoware_planning_msgs::Trajectory & trajectory);
void maximumVelocityFilter(const double & max_vel, autoware_planning_msgs::Trajectory & trajectory);
void multiplyConstantToTrajectoryVelocity(
  const double & scalar, autoware_planning_msgs::Trajectory & trajectory);
void insertZeroVelocityAfterIdx(
  const int & stop_idx, autoware_planning_msgs::Trajectory & trajectory);
double getVx(const autoware_planning_msgs::Trajectory & trajectory, const int & i);
bool searchZeroVelocityIdx(const autoware_planning_msgs::Trajectory & trajectory, int & idx);
bool calcTrajectoryCurvatureFrom3Points(
  const autoware_planning_msgs::Trajectory & trajectory, const unsigned int & idx_dist,
  std::vector<double> & k_arr);
double normalizeRadian(const double _angle);
void convertEulerAngleToMonotonic(std::vector<double> & a);
geometry_msgs::Quaternion getQuaternionFromYaw(double yaw);
bool linearInterpTrajectory(
  const std::vector<double> & base_index,
  const autoware_planning_msgs::Trajectory & base_trajectory, const std::vector<double> & out_index,
  autoware_planning_msgs::Trajectory & out_trajectory);
}  // namespace vpu