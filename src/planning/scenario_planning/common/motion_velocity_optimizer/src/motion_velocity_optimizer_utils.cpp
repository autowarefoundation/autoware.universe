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

#include <motion_velocity_optimizer/interpolate.h>
#include <motion_velocity_optimizer/motion_velocity_optimizer_utils.hpp>

namespace vpu
{
double square(const double & a) { return a * a; }
double calcSquaredDist2d(const geometry_msgs::Point & a, const geometry_msgs::Point & b)
{
  return square(a.x - b.x) + square(a.y - b.y);
}
double calcSquaredDist2d(const geometry_msgs::Pose & a, const geometry_msgs::Pose & b)
{
  return square(a.position.x - b.position.x) + square(a.position.y - b.position.y);
}
double calcSquaredDist2d(const geometry_msgs::PoseStamped & a, const geometry_msgs::PoseStamped & b)
{
  return square(a.pose.position.x - b.pose.position.x) +
         square(a.pose.position.y - b.pose.position.y);
}
double calcSquaredDist2d(
  const autoware_planning_msgs::TrajectoryPoint & a,
  const autoware_planning_msgs::TrajectoryPoint & b)
{
  return square(a.pose.position.x - b.pose.position.x) +
         square(a.pose.position.y - b.pose.position.y);
}

double calcDist2d(const geometry_msgs::Point & a, const geometry_msgs::Point & b)
{
  return std::sqrt(calcSquaredDist2d(a, b));
}
double calcDist2d(const geometry_msgs::Pose & a, const geometry_msgs::Pose & b)
{
  return std::sqrt(calcSquaredDist2d(a, b));
}
double calcDist2d(const geometry_msgs::PoseStamped & a, const geometry_msgs::PoseStamped & b)
{
  return std::sqrt(calcSquaredDist2d(a, b));
}
double calcDist2d(
  const autoware_planning_msgs::TrajectoryPoint & a,
  const autoware_planning_msgs::TrajectoryPoint & b)
{
  return std::sqrt(calcSquaredDist2d(a, b));
}

int calcClosestWaypoint(
  const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::Point & point)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  int idx_min = -1;

  for (int i = 0; i < (int)traj.points.size(); ++i) {
    const double dx = traj.points.at(i).pose.position.x - point.x;
    const double dy = traj.points.at(i).pose.position.y - point.y;
    const double dist_squared = dx * dx + dy * dy;
    if (dist_squared < dist_squared_min) {
      dist_squared_min = dist_squared;
      idx_min = i;
    }
  }
  return idx_min;
}

int calcClosestWaypoint(
  const autoware_planning_msgs::Trajectory & trajectory, const geometry_msgs::Pose & pose,
  const double delta_yaw_threshold)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  int idx_min = -1;

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const double dx = trajectory.points.at(i).pose.position.x - pose.position.x;
    const double dy = trajectory.points.at(i).pose.position.y - pose.position.y;
    const double dist_squared = dx * dx + dy * dy;
    const double traj_point_yaw = tf2::getYaw(trajectory.points.at(i).pose.orientation);
    const double pose_yaw = tf2::getYaw(pose.orientation);
    const double delta_yaw = traj_point_yaw - pose_yaw;
    const double norm_delta_yaw = normalizeRadian(delta_yaw);
    if (dist_squared < dist_squared_min && std::fabs(norm_delta_yaw) < delta_yaw_threshold) {
      dist_squared_min = dist_squared;
      idx_min = i;
    }
  }
  return idx_min;
}

bool extractPathAroundIndex(
  const autoware_planning_msgs::Trajectory & trajectory, const int index,
  const double & ahead_length, const double & behind_length,
  autoware_planning_msgs::Trajectory & extracted_base_trajectory)
{
  if (trajectory.points.size() == 0 || (int)trajectory.points.size() - 1 < index || index < 0) {
    return false;
  }

  double dist_sum_tmp = 0.0;

  // calc ahead distance
  int ahead_index = trajectory.points.size() - 1;
  for (int i = index; i < (int)trajectory.points.size() - 1; ++i) {
    dist_sum_tmp += vpu::calcDist2d(trajectory.points.at(i), trajectory.points.at(i + 1));
    if (dist_sum_tmp > ahead_length) {
      ahead_index = i + 1;
      break;
    }
  }

  // calc behind distance
  dist_sum_tmp = 0.0;
  int behind_index = 0;
  for (int i = index; i > 0; --i) {
    dist_sum_tmp += vpu::calcDist2d(trajectory.points.at(i), trajectory.points[i - 1]);
    if (dist_sum_tmp > behind_length) {
      behind_index = i - 1;
      break;
    }
  }

  // extruct trajectory
  extracted_base_trajectory.points.clear();
  for (int i = behind_index; i < ahead_index + 1; ++i) {
    extracted_base_trajectory.points.push_back(trajectory.points.at(i));
  }
  extracted_base_trajectory.header = trajectory.header;

  return true;
}

double calcLengthOnWaypoints(
  const autoware_planning_msgs::Trajectory & path, const int idx1, const int idx2)
{
  if (idx1 == idx2)  // zero distance
    return 0.0;

  if (
    idx1 < 0 || idx2 < 0 || (int)path.points.size() - 1 < idx1 ||
    (int)path.points.size() - 1 < idx2) {
    std::cerr << "vpu::calcLengthOnWaypoints(): invalid index" << std::endl;
    return 0.0;
  }

  const int idx_from = std::min(idx1, idx2);
  const int idx_to = std::max(idx1, idx2);
  double dist_sum = 0.0;
  for (int i = idx_from; i < idx_to; ++i) {
    dist_sum += vpu::calcDist2d(path.points.at(i), path.points.at(i + 1));
  }
  return dist_sum;
}

void calcTrajectoryArclength(
  const autoware_planning_msgs::Trajectory & trajectory, std::vector<double> & arclength)
{
  double dist = 0.0;
  arclength.clear();
  arclength.push_back(dist);
  for (unsigned int i = 1; i < trajectory.points.size(); ++i) {
    const autoware_planning_msgs::TrajectoryPoint tp = trajectory.points.at(i);
    const autoware_planning_msgs::TrajectoryPoint tp_prev = trajectory.points.at(i - 1);
    dist += vpu::calcDist2d(tp.pose, tp_prev.pose);
    arclength.push_back(dist);
  }
}

void calcTrajectoryIntervalDistance(
  const autoware_planning_msgs::Trajectory & trajectory, std::vector<double> & intervals)
{
  intervals.clear();
  for (unsigned int i = 1; i < trajectory.points.size(); ++i) {
    const autoware_planning_msgs::TrajectoryPoint tp = trajectory.points.at(i);
    const autoware_planning_msgs::TrajectoryPoint tp_prev = trajectory.points.at(i - 1);
    const double dist = vpu::calcDist2d(tp.pose, tp_prev.pose);
    intervals.push_back(dist);
  }
}

void setZeroVelocity(autoware_planning_msgs::Trajectory & trajectory)
{
  for (auto & tp : trajectory.points) {
    tp.twist.linear.x = 0.0;
  }
  return;
}

void mininumVelocityFilter(const double & min_vel, autoware_planning_msgs::Trajectory & trajectory)
{
  for (auto & tp : trajectory.points) {
    if (tp.twist.linear.x < min_vel) tp.twist.linear.x = min_vel;
  }
}

void maximumVelocityFilter(const double & max_vel, autoware_planning_msgs::Trajectory & trajectory)
{
  const double abs_max_vel = std::fabs(max_vel);
  for (auto & tp : trajectory.points) {
    if (tp.twist.linear.x > abs_max_vel)
      tp.twist.linear.x = abs_max_vel;
    else if (tp.twist.linear.x < -abs_max_vel)
      tp.twist.linear.x = -abs_max_vel;
  }
}
void multiplyConstantToTrajectoryVelocity(
  const double & scalar, autoware_planning_msgs::Trajectory & trajectory)
{
  for (auto & tp : trajectory.points) {
    tp.twist.linear.x *= scalar;
  }
}

void insertZeroVelocityAfterIdx(
  const int & stop_idx, autoware_planning_msgs::Trajectory & trajectory)
{
  if (stop_idx < 0) return;

  for (int i = stop_idx; i < (int)trajectory.points.size(); ++i) {
    trajectory.points.at(i).twist.linear.x = 0.0;
  }
}

double getVx(const autoware_planning_msgs::Trajectory & trajectory, const int & i)
{
  return trajectory.points.at(i).twist.linear.x;
}

bool searchZeroVelocityIdx(const autoware_planning_msgs::Trajectory & trajectory, int & idx)
{
  for (unsigned int i = 0; i < trajectory.points.size(); ++i) {
    if (std::fabs(vpu::getVx(trajectory, i)) < 1.0E-3) {
      idx = i;
      return true;
    }
  }
  return false;
}

bool calcTrajectoryCurvatureFrom3Points(
  const autoware_planning_msgs::Trajectory & trajectory, const unsigned int & idx_dist,
  std::vector<double> & k_arr)
{
  k_arr.clear();
  if (trajectory.points.size() < 2 * idx_dist + 1) {
    ROS_DEBUG(
      "[calcTrajectoryCurvatureFrom3Points] cannot calc curvature idx_dist = %d, trajectory.size() "
      "= %lu",
      idx_dist, trajectory.points.size());
    for (unsigned int i = 0; i < trajectory.points.size(); ++i) {
      k_arr.push_back(0.0);
    }
    return false;
  }

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::Point p1, p2, p3;
  for (unsigned int i = idx_dist; i < trajectory.points.size() - idx_dist; ++i) {
    p1.x = trajectory.points.at(i - idx_dist).pose.position.x;
    p2.x = trajectory.points.at(i).pose.position.x;
    p3.x = trajectory.points.at(i + idx_dist).pose.position.x;
    p1.y = trajectory.points.at(i - idx_dist).pose.position.y;
    p2.y = trajectory.points.at(i).pose.position.y;
    p3.y = trajectory.points.at(i + idx_dist).pose.position.y;
    double den = std::max(calcDist2d(p1, p2) * calcDist2d(p2, p3) * calcDist2d(p3, p1), 0.0001);
    double curvature = 2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
    k_arr.push_back(curvature);
  }

  // for debug
  if (k_arr.size() == 0) {
    ROS_ERROR("[calcTrajectoryCurvatureFrom3Points] k_arr.size() = 0, somthing wrong. pls check.");
    return false;
  }

  /* first and last curvature is copied from next value */
  for (unsigned int i = 0; i < idx_dist; ++i) {
    k_arr.insert(k_arr.begin(), k_arr.front());
    k_arr.push_back(k_arr.back());
  }
  return true;
}

double normalizeRadian(const double _angle)
{
  double n_angle = std::fmod(_angle, 2 * M_PI);
  n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;

  // another way
  // Math.atan2(Math.sin(_angle), Math.cos(_angle));
  return n_angle;
}

void convertEulerAngleToMonotonic(std::vector<double> & a)
{
  for (unsigned int i = 1; i < a.size(); ++i) {
    const double da = a[i] - a[i - 1];
    a[i] = a[i - 1] + normalizeRadian(da);
  }
}

geometry_msgs::Quaternion getQuaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

bool linearInterpTrajectory(
  const std::vector<double> & base_index,
  const autoware_planning_msgs::Trajectory & base_trajectory, const std::vector<double> & out_index,
  autoware_planning_msgs::Trajectory & out_trajectory)
{
  std::vector<double> px, py, pz, pyaw, tlx, taz, alx, aaz;
  for (const auto & p : base_trajectory.points) {
    px.push_back(p.pose.position.x);
    py.push_back(p.pose.position.y);
    pz.push_back(p.pose.position.z);
    pyaw.push_back(tf2::getYaw(p.pose.orientation));
    tlx.push_back(p.twist.linear.x);
    taz.push_back(p.twist.angular.z);
    alx.push_back(p.accel.linear.x);
    aaz.push_back(p.accel.angular.z);
  }

  convertEulerAngleToMonotonic(pyaw);

  std::vector<double> px_p, py_p, pz_p, pyaw_p, tlx_p, taz_p, alx_p, aaz_p;

  if (
    !LinearInterpolate::interpolate(base_index, px, out_index, px_p) ||
    !LinearInterpolate::interpolate(base_index, py, out_index, py_p) ||
    !LinearInterpolate::interpolate(base_index, pz, out_index, pz_p) ||
    !LinearInterpolate::interpolate(base_index, pyaw, out_index, pyaw_p) ||
    !LinearInterpolate::interpolate(base_index, tlx, out_index, tlx_p) ||
    !LinearInterpolate::interpolate(base_index, taz, out_index, taz_p) ||
    !LinearInterpolate::interpolate(base_index, alx, out_index, alx_p) ||
    !LinearInterpolate::interpolate(base_index, aaz, out_index, aaz_p)) {
    ROS_WARN("[linearInterpTrajectory] interpolation error!!");
    return false;
  }

  out_trajectory.header = base_trajectory.header;
  out_trajectory.points.clear();
  autoware_planning_msgs::TrajectoryPoint point;
  for (unsigned int i = 0; i < out_index.size(); ++i) {
    point.pose.position.x = px_p.at(i);
    point.pose.position.y = py_p.at(i);
    point.pose.position.z = pz_p.at(i);
    point.pose.orientation = getQuaternionFromYaw(pyaw_p.at(i));
    point.twist.linear.x = tlx_p.at(i);
    point.twist.angular.z = taz_p.at(i);
    point.accel.linear.x = alx_p.at(i);
    point.accel.angular.z = aaz_p.at(i);
    out_trajectory.points.push_back(point);
  }
  return true;
}

}  // namespace vpu
