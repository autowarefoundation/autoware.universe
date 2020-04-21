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

#include "mpc_follower/mpc_utils.h"

geometry_msgs::Quaternion MPCUtils::getQuaternionFromYaw(const double & yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

void MPCUtils::convertEulerAngleToMonotonic(std::vector<double> * a)
{
  if (!a) {
    return;
  }
  for (unsigned int i = 1; i < a->size(); ++i) {
    const double da = a->at(i) - a->at(i - 1);
    a->at(i) = a->at(i - 1) + MPCUtils::normalizeRadian(da);
  }
}

double MPCUtils::normalizeRadian(const double angle)
{
  double n_angle = std::fmod(angle, 2 * M_PI);
  n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
  return n_angle;
}

double MPCUtils::calcDist2d(
  const geometry_msgs::PoseStamped & p0, const geometry_msgs::PoseStamped & p1)
{
  return calcDist2d(p0.pose.position, p1.pose.position);
}

double MPCUtils::calcDist2d(const geometry_msgs::Pose & p0, const geometry_msgs::Pose & p1)
{
  return calcDist2d(p0.position, p1.position);
}

double MPCUtils::calcDist2d(const geometry_msgs::Point & p0, const geometry_msgs::Point & p1)
{
  return std::hypot(p0.x - p1.x, p0.y - p1.y);
}

double MPCUtils::calcSquaredDist2d(const geometry_msgs::Point & p0, const geometry_msgs::Point & p1)
{
  double dx = p1.x - p0.x;
  double dy = p1.y - p0.y;
  return dx * dx + dy * dy;
}

double MPCUtils::calcDist3d(const geometry_msgs::Point & p0, const geometry_msgs::Point & p1)
{
  double dx = p1.x - p0.x;
  double dy = p1.y - p0.y;
  double dz = p1.z - p0.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double MPCUtils::calcLateralError(
  const geometry_msgs::Pose & ego_pose, const geometry_msgs::Pose & ref_pose)
{
  const double err_x = ego_pose.position.x - ref_pose.position.x;
  const double err_y = ego_pose.position.y - ref_pose.position.y;
  const double ref_yaw = tf2::getYaw(ref_pose.orientation);
  const double lat_err = -std::sin(ref_yaw) * err_x + std::cos(ref_yaw) * err_y;
  return lat_err;
}

void MPCUtils::calcMPCTrajectoryArclength(
  const MPCTrajectory & trajectory, std::vector<double> * arclength)
{
  double dist = 0.0;
  arclength->clear();
  arclength->push_back(dist);
  for (unsigned int i = 1; i < trajectory.size(); ++i) {
    double dx = trajectory.x.at(i) - trajectory.x.at(i - 1);
    double dy = trajectory.y.at(i) - trajectory.y.at(i - 1);
    dist += std::sqrt(dx * dx + dy * dy);
    arclength->push_back(dist);
  }
}

bool MPCUtils::resampleMPCTrajectoryByDistance(
  const MPCTrajectory & input, const double resample_interval_dist, MPCTrajectory * output)
{
  if (!output) {
    return false;
  }
  if (input.size() == 0) {
    *output = input;
    return true;
  }
  std::vector<double> input_arclength;
  calcMPCTrajectoryArclength(input, &input_arclength);

  if (input_arclength.size() == 0) {
    return false;
  }

  std::vector<double> output_arclength;
  for (double s = 0; s < input_arclength.back(); s += resample_interval_dist) {
    output_arclength.push_back(s);
  }

  // splineInterpMPCTrajectory(input_arclength, input, output_arclength, output);

  std::vector<double> input_yaw = input.yaw;
  MPCUtils::convertEulerAngleToMonotonic(&input_yaw);

  LinearInterpolate linear_interp;
  SplineInterpolate spline_interp;
  if (
    !spline_interp.interpolate(input_arclength, input.x, output_arclength, output->x) ||
    !spline_interp.interpolate(input_arclength, input.y, output_arclength, output->y) ||
    !spline_interp.interpolate(input_arclength, input.z, output_arclength, output->z) ||
    !spline_interp.interpolate(input_arclength, input_yaw, output_arclength, output->yaw) ||
    !linear_interp.interpolate(input_arclength, input.vx, output_arclength, output->vx) ||
    !spline_interp.interpolate(input_arclength, input.k, output_arclength, output->k) ||
    !linear_interp.interpolate(
      input_arclength, input.relative_time, output_arclength, output->relative_time)) {
    std::cerr << "linearInterpMPCTrajectory error!" << std::endl;
    return false;
  }

  return true;
}

bool MPCUtils::linearInterpMPCTrajectory(
  const std::vector<double> & in_index, const MPCTrajectory & in_traj,
  const std::vector<double> & out_index, MPCTrajectory * out_traj)
{
  if (!out_traj) {
    return false;
  }

  if (in_traj.size() == 0) {
    *out_traj = in_traj;
    return true;
  }

  std::vector<double> in_traj_yaw = in_traj.yaw;
  MPCUtils::convertEulerAngleToMonotonic(&in_traj_yaw);

  LinearInterpolate linear_interp;
  if (
    !linear_interp.interpolate(in_index, in_traj.x, out_index, out_traj->x) ||
    !linear_interp.interpolate(in_index, in_traj.y, out_index, out_traj->y) ||
    !linear_interp.interpolate(in_index, in_traj.z, out_index, out_traj->z) ||
    !linear_interp.interpolate(in_index, in_traj_yaw, out_index, out_traj->yaw) ||
    !linear_interp.interpolate(in_index, in_traj.vx, out_index, out_traj->vx) ||
    !linear_interp.interpolate(in_index, in_traj.k, out_index, out_traj->k) ||
    !linear_interp.interpolate(
      in_index, in_traj.relative_time, out_index, out_traj->relative_time)) {
    std::cerr << "linearInterpMPCTrajectory error!" << std::endl;
    return false;
  }

  if (out_traj->size() == 0) {
    std::cerr << "[mpc util] linear interpolation error" << std::endl;
    return false;
  }

  return true;
}

bool MPCUtils::splineInterpMPCTrajectory(
  const std::vector<double> & in_index, const MPCTrajectory & in_traj,
  const std::vector<double> & out_index, MPCTrajectory * out_traj)
{
  if (!out_traj) {
    return false;
  }
  if (in_traj.size() == 0) {
    *out_traj = in_traj;
    return true;
  }

  std::vector<double> in_traj_yaw = in_traj.yaw;
  MPCUtils::convertEulerAngleToMonotonic(&in_traj_yaw);

  out_traj->clear();
  SplineInterpolate spline_interp;
  if (
    !spline_interp.interpolate(in_index, in_traj.x, out_index, out_traj->x) ||
    !spline_interp.interpolate(in_index, in_traj.y, out_index, out_traj->y) ||
    !spline_interp.interpolate(in_index, in_traj.z, out_index, out_traj->z) ||
    !spline_interp.interpolate(in_index, in_traj_yaw, out_index, out_traj->yaw) ||
    !spline_interp.interpolate(in_index, in_traj.vx, out_index, out_traj->vx) ||
    !spline_interp.interpolate(in_index, in_traj.k, out_index, out_traj->k)) {
    std::cerr << "splineInterpMPCTrajectory error!" << std::endl;
    return false;
  }

  // use linear interpolation for time.
  calcMPCTrajectoryTime(out_traj);

  if (out_traj->size() == 0) {
    std::cerr << "[mpc util] spline interpolation error" << std::endl;
    return false;
  }

  return true;
}

void MPCUtils::calcTrajectoryYawFromXY(MPCTrajectory * traj)
{
  if (traj->yaw.size() == 0) return;

  for (unsigned int i = 1; i < traj->yaw.size() - 1; ++i) {
    const double dx = traj->x[i + 1] - traj->x[i - 1];
    const double dy = traj->y[i + 1] - traj->y[i - 1];
    traj->yaw[i] = std::atan2(dy, dx);
  }
  if (traj->yaw.size() > 1) {
    traj->yaw[0] = traj->yaw[1];
    traj->yaw.back() = traj->yaw[traj->yaw.size() - 2];
  }
}

bool MPCUtils::calcTrajectoryCurvature(int curvature_smoothing_num, MPCTrajectory * traj)
{
  if (!traj) {
    return false;
  }

  int traj_size = static_cast<int>(traj->x.size());
  traj->k.clear();
  traj->k.resize(traj_size, 0.0);

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::Point p1, p2, p3;
  int max_smoothing_num = static_cast<int>(std::floor(0.5 * (traj_size - 1)));
  int L = std::min(curvature_smoothing_num, max_smoothing_num);
  for (int i = L; i < traj_size - L; ++i) {
    p1.x = traj->x[i - L];
    p2.x = traj->x[i];
    p3.x = traj->x[i + L];
    p1.y = traj->y[i - L];
    p2.y = traj->y[i];
    p3.y = traj->y[i + L];
    double den = std::max(calcDist2d(p1, p2) * calcDist2d(p2, p3) * calcDist2d(p3, p1), 0.0001);
    const double curvature =
      2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
    traj->k.at(i) = curvature;
  }

  /* first and last curvature is copied from next value */
  for (int i = 0; i < std::min(L, traj_size); ++i) {
    traj->k.at(i) = traj->k.at(std::min(L, traj_size - 1));
    traj->k.at(traj_size - i - 1) = traj->k.at(std::max(traj_size - L - 1, 0));
  }
  return true;
}

bool MPCUtils::convertToMPCTrajectory(
  const autoware_planning_msgs::Trajectory & input, MPCTrajectory * output)
{
  if (!output) {
    return false;
  }

  output->clear();
  for (uint i = 0; i < input.points.size(); ++i) {
    geometry_msgs::Pose p = input.points.at(i).pose;
    const double x = p.position.x;
    const double y = p.position.y;
    const double z = p.position.z;
    const double yaw = tf2::getYaw(p.orientation);
    const double vx = input.points.at(i).twist.linear.x;
    const double k = 0.0;
    const double t = 0.0;
    output->push_back(x, y, z, yaw, vx, k, t);
  }
  calcMPCTrajectoryTime(output);
  return true;
}

bool MPCUtils::calcMPCTrajectoryTime(MPCTrajectory * traj)
{
  if (!traj) {
    return false;
  }
  double t = 0.0;
  int traj_size = traj->x.size();
  traj->relative_time.clear();
  traj->relative_time.push_back(t);
  for (int i = 0; i < traj_size - 1; ++i) {
    double dx = traj->x.at(i + 1) - traj->x.at(i);
    double dy = traj->y.at(i + 1) - traj->y.at(i);
    double dz = traj->z.at(i + 1) - traj->z.at(i);
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double v = std::max(std::fabs(traj->vx.at(i)), 0.1);
    t += (dist / v);
    traj->relative_time.push_back(t);
  }
  return true;
}

void MPCUtils::dynamicSmoothingVelocity(
  const int start_idx, const double start_vel, const double acc_lim, const double tau,
  MPCTrajectory * traj)
{
  const double ep = 1.0E-3;
  double curr_v = start_vel;
  std::vector<double> smoothed_vel;
  MPCTrajectory tmp = *traj;
  traj->vx.at(start_idx) = start_vel;

  int traj_size = static_cast<int>(traj->size());

  for (int i = start_idx + 1; i < traj_size; ++i) {
    const double ds =
      std::hypot(traj->x.at(i) - traj->x.at(i - 1), traj->y.at(i) - traj->y.at(i - 1));
    const double dt = ds / std::max(std::fabs(curr_v), ep);
    const double a = tau / std::max(tau + dt, ep);
    const double updated_v = a * curr_v + (1.0 - a) * traj->vx.at(i);
    const double dv = std::max(-acc_lim * dt, std::min(acc_lim * dt, updated_v - curr_v));
    curr_v = curr_v + dv;
    traj->vx.at(i) = curr_v;
  }
  calcMPCTrajectoryTime(traj);
}

int MPCUtils::calcNearestIndex(const MPCTrajectory & traj, const geometry_msgs::Pose & self_pose)
{
  if (traj.size() == 0) {
    return -1;
  }
  const double my_yaw = tf2::getYaw(self_pose.orientation);
  int nearest_idx = -1;
  double min_dist_squared = std::numeric_limits<double>::max();
  for (uint i = 0; i < traj.size(); ++i) {
    const double dx = self_pose.position.x - traj.x[i];
    const double dy = self_pose.position.y - traj.y[i];
    const double dist_squared = dx * dx + dy * dy;

    /* ignore when yaw error is large, for crossing path */
    const double err_yaw = normalizeRadian(my_yaw - traj.yaw[i]);
    if (std::fabs(err_yaw) > (M_PI / 3.0)) {
      continue;
    }
    if (dist_squared < min_dist_squared) {
      min_dist_squared = dist_squared;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}
bool MPCUtils::calcNearestPoseInterp(
  const MPCTrajectory & traj, const geometry_msgs::Pose & self_pose,
  geometry_msgs::Pose * nearest_pose, int * nearest_index, double * nearest_time)
{
  if (traj.size() == 0 || !nearest_pose || !nearest_index || !nearest_time) {
    return false;
  }
  int nearest_idx = calcNearestIndex(traj, self_pose);
  if (nearest_idx == -1) {
    ROS_WARN_DELAYED_THROTTLE(
      3.0, "[calcNearestPoseInterp] fail to get nearest. traj.size = %d", (int)traj.size());
    return false;
  }

  int traj_size = static_cast<int>(traj.size());

  *nearest_index = nearest_idx;

  if (traj.size() == 1) {
    nearest_pose->position.x = traj.x[nearest_idx];
    nearest_pose->position.y = traj.y[nearest_idx];
    nearest_pose->orientation = getQuaternionFromYaw(traj.yaw[nearest_idx]);
    *nearest_time = traj.relative_time[nearest_idx];
    return true;
  }

  auto calcSquaredDist = [](const geometry_msgs::Pose & p, const MPCTrajectory & t, int idx) {
    const double dx = p.position.x - t.x[idx];
    const double dy = p.position.y - t.y[idx];
    return dx * dx + dy * dy;
  };

  /* get second nearest index = next to nearest_index */
  int next = std::min(nearest_idx + 1, traj_size - 1);
  int prev = std::max(nearest_idx - 1, 0);
  double dist_to_next = calcSquaredDist(self_pose, traj, next);
  double dist_to_prev = calcSquaredDist(self_pose, traj, prev);
  int second_nearest_index = (dist_to_next < dist_to_prev) ? next : prev;

  const double a_sq = calcSquaredDist(self_pose, traj, nearest_idx);
  const double b_sq = calcSquaredDist(self_pose, traj, second_nearest_index);
  const double dx3 = traj.x[nearest_idx] - traj.x[second_nearest_index];
  const double dy3 = traj.y[nearest_idx] - traj.y[second_nearest_index];
  const double c_sq = dx3 * dx3 + dy3 * dy3;

  /* if distance between two points are too close */
  if (c_sq < 1.0E-5) {
    nearest_pose->position.x = traj.x[nearest_idx];
    nearest_pose->position.y = traj.y[nearest_idx];
    nearest_pose->orientation = getQuaternionFromYaw(traj.yaw[nearest_idx]);
    *nearest_time = traj.relative_time[nearest_idx];
    return true;
  }

  /* linear interpolation */
  const double alpha = std::max(std::min(0.5 * (c_sq - a_sq + b_sq) / c_sq, 1.0), 0.0);
  nearest_pose->position.x =
    alpha * traj.x[nearest_idx] + (1 - alpha) * traj.x[second_nearest_index];
  nearest_pose->position.y =
    alpha * traj.y[nearest_idx] + (1 - alpha) * traj.y[second_nearest_index];
  double tmp_yaw_err = normalizeRadian(traj.yaw[nearest_idx] - traj.yaw[second_nearest_index]);
  const double nearest_yaw = normalizeRadian(traj.yaw[second_nearest_index] + alpha * tmp_yaw_err);
  nearest_pose->orientation = getQuaternionFromYaw(nearest_yaw);
  *nearest_time = alpha * traj.relative_time[nearest_idx] +
                  (1 - alpha) * traj.relative_time[second_nearest_index];
  return true;
}

visualization_msgs::MarkerArray MPCUtils::convertTrajToMarker(
  const MPCTrajectory & traj, std::string ns, double r, double g, double b, double z,
  std::string & frame_id)
{
  visualization_msgs::MarkerArray markers;

  // generate line
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns + "/line";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.15;
  marker.color.a = 0.9;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.pose.orientation.w = 1.0;
  for (unsigned int i = 0; i < traj.x.size(); ++i) {
    geometry_msgs::Point p;
    p.x = traj.x.at(i);
    p.y = traj.y.at(i);
    p.z = traj.z.at(i) + z;
    marker.points.push_back(p);
  }
  markers.markers.push_back(marker);

  // generate poses
  visualization_msgs::Marker marker_poses;
  marker_poses.header.frame_id = frame_id;
  marker_poses.header.stamp = ros::Time();
  marker_poses.ns = ns + "/poses";
  marker_poses.lifetime = ros::Duration(0.5);
  marker_poses.type = visualization_msgs::Marker::ARROW;
  marker_poses.action = visualization_msgs::Marker::ADD;
  marker_poses.scale.x = 0.2;
  marker_poses.scale.y = 0.1;
  marker_poses.scale.z = 0.2;
  marker_poses.color.a = 0.99;  // Don't forget to set the alpha!
  marker_poses.color.r = r;
  marker_poses.color.g = g;
  marker_poses.color.b = b;
  for (unsigned int i = 0; i < traj.size(); ++i) {
    marker_poses.id = i;
    marker_poses.pose.position.x = traj.x.at(i);
    marker_poses.pose.position.y = traj.y.at(i);
    marker_poses.pose.position.z = traj.z.at(i);
    marker_poses.pose.orientation = MPCUtils::getQuaternionFromYaw(traj.yaw.at(i));
    markers.markers.push_back(marker_poses);
  }

  // generate velocity text
  // visualization_msgs::Marker marker_text;
  // marker_text.header.frame_id = frame_id;
  // marker_text.header.stamp = ros::Time();
  // marker_text.ns = ns + "/text";
  // marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // marker_text.action = visualization_msgs::Marker::ADD;
  // marker_text.scale.z = 0.2;
  // marker_text.color.a = 0.99;  // Don't forget to set the alpha!
  // marker_text.color.r = r;
  // marker_text.color.g = g;
  // marker_text.color.b = b;

  // for (unsigned int i = 0; i < traj.size(); ++i) {
  //   marker_text.id = i;
  //   marker_text.pose.position.x = traj.x.at(i);
  //   marker_text.pose.position.y = traj.y.at(i);
  //   marker_text.pose.position.z = traj.z.at(i);
  //   marker_text.pose.orientation = MPCUtils::getQuaternionFromYaw(traj.yaw.at(i));
  //   std::ostringstream oss;
  //   oss << std::fixed << std::setprecision(1) << traj.vx.at(i) << ", " << i;
  //   marker_text.text = oss.str();
  //   markers.markers.push_back(marker_text);
  // }

  return markers;
}
