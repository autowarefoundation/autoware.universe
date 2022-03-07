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

#include "obstacle_velocity_planner/resample.hpp"

#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation.hpp"

#include <vector>

namespace
{
rclcpp::Duration safeSubtraction(const rclcpp::Time & t1, const rclcpp::Time & t2)
{
  rclcpp::Duration duration = rclcpp::Duration::from_seconds(0.0);
  try {
    duration = t1 - t2;
  } catch (std::runtime_error & err) {
    if (t1 > t2) {
      duration = rclcpp::Duration::max() * -1.0;
    } else {
      duration = rclcpp::Duration::max();
    }
  }
  return duration;
}

// tf2::toMsg does not have this type of function
geometry_msgs::msg::Point toMsg(tf2::Vector3 vec)
{
  geometry_msgs::msg::Point point;
  point.x = vec.x();
  point.y = vec.y();
  point.z = vec.z();
  return point;
}
}  // namespace

namespace resampling
{
autoware_perception_msgs::msg::PredictedPath resamplePredictedPath(
  const autoware_perception_msgs::msg::PredictedPath & input_path, const std::vector<double> & time_vec,
  const rclcpp::Time & start_time, const double duration)
{
  autoware_perception_msgs::msg::PredictedPath resampled_path;

  const auto prediction_duration = rclcpp::Duration::from_seconds(duration);
  const auto end_time = start_time + prediction_duration;

  for(const auto & time : time_vec) {
    const auto current_time = start_time + rclcpp::Duration::from_seconds(time);
    if(current_time > end_time) {
      break;
    }

    geometry_msgs::msg::Pose pose;
    if (!lerpByTimeStamp(input_path, current_time, &pose)) {
      continue;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped predicted_pose;
    predicted_pose.header.frame_id = "map";
    predicted_pose.header.stamp = current_time;
    predicted_pose.pose.pose = pose;
    resampled_path.path.push_back(predicted_pose);
  }

  return resampled_path;
}

geometry_msgs::msg::Pose lerpByPose(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2, const double t)
{
  tf2::Transform tf_transform1, tf_transform2;
  tf2::fromMsg(p1, tf_transform1);
  tf2::fromMsg(p2, tf_transform2);
  const auto & tf_point = tf2::lerp(tf_transform1.getOrigin(), tf_transform2.getOrigin(), t);
  const auto & tf_quaternion =
    tf2::slerp(tf_transform1.getRotation(), tf_transform2.getRotation(), t);

  geometry_msgs::msg::Pose pose;
  pose.position = toMsg(tf_point);
  pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

bool lerpByTimeStamp(
  const autoware_perception_msgs::msg::PredictedPath & path, const rclcpp::Time & t,
  geometry_msgs::msg::Pose * lerped_pt)
{
  auto clock{rclcpp::Clock{RCL_ROS_TIME}};
  if (lerped_pt == nullptr) {
    RCLCPP_WARN_STREAM_THROTTLE(
      rclcpp::get_logger("DynamicAvoidance.resample"), clock, 1000,
      "failed to lerp by time due to nullptr pt");
    return false;
  }
  if (path.path.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      rclcpp::get_logger("DynamicAvoidance.resample"), clock, 1000,
      "Empty path. Failed to interpolate path by time!");
    return false;
  }
  if (t < path.path.front().header.stamp) {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("DynamicAvoidance.resample"),
      "failed to interpolate path by time!"
        << std::endl
        << "path start time: " << path.path.front().header.stamp.sec << std::endl
        << "path end time  : " << path.path.back().header.stamp.sec << std::endl
        << "query time     : " << t.seconds());

    *lerped_pt = path.path.front().pose.pose;
    return false;
  }

  if (t > path.path.back().header.stamp) {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("DynamicAvoidance.resample"),
      "failed to interpolate path by time!"
        << std::endl
        << "path start time: " << path.path.front().header.stamp.sec << std::endl
        << "path end time  : " << path.path.back().header.stamp.sec << std::endl
        << "query time     : " << t.seconds());
    *lerped_pt = path.path.back().pose.pose;

    return false;
  }

  for (size_t i = 1; i < path.path.size(); i++) {
    const auto & pt = path.path.at(i);
    const auto & prev_pt = path.path.at(i - 1);
    if (t <= pt.header.stamp) {
      const rclcpp::Duration duration = safeSubtraction(pt.header.stamp, prev_pt.header.stamp);
      const auto offset = t - prev_pt.header.stamp;
      const auto ratio = offset.seconds() / duration.seconds();
      *lerped_pt = lerpByPose(prev_pt.pose.pose, pt.pose.pose, ratio);
      return true;
    }
  }

  RCLCPP_ERROR_STREAM(
    rclcpp::get_logger("DynamicAvoidance.resample"), "Something failed in function: " << __func__);
  return false;
}

inline void convertEulerAngleToMonotonic(std::vector<double> & a)
{
  for (unsigned int i = 1; i < a.size(); ++i) {
    const double da = a[i] - a[i - 1];
    a[i] = a[i - 1] + autoware_utils::normalizeRadian(da);
  }
}

autoware_planning_msgs::msg::Trajectory applyLinearInterpolation(
  const std::vector<double> & base_index, const autoware_planning_msgs::msg::Trajectory & base_trajectory,
  const std::vector<double> & out_index, const bool use_spline_for_pose)
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

  std::vector<double> px_p, py_p, pz_p, pyaw_p;
  if (use_spline_for_pose) {
    px_p = interpolation::slerp(base_index, px, out_index);
    py_p = interpolation::slerp(base_index, py, out_index);
    pz_p = interpolation::slerp(base_index, pz, out_index);
    pyaw_p = interpolation::slerp(base_index, pyaw, out_index);
  } else {
    px_p = interpolation::lerp(base_index, px, out_index);
    py_p = interpolation::lerp(base_index, py, out_index);
    pz_p = interpolation::lerp(base_index, pz, out_index);
    pyaw_p = interpolation::lerp(base_index, pyaw, out_index);
  }
  const auto tlx_p = interpolation::lerp(base_index, tlx, out_index);
  const auto taz_p = interpolation::lerp(base_index, taz, out_index);
  const auto alx_p = interpolation::lerp(base_index, alx, out_index);
  const auto aaz_p = interpolation::lerp(base_index, aaz, out_index);

  autoware_planning_msgs::msg::Trajectory out_trajectory;
  out_trajectory.header = base_trajectory.header;
  autoware_planning_msgs::msg::TrajectoryPoint point;
  for (unsigned int i = 0; i < out_index.size(); ++i) {
    point.pose.position.x = px_p.at(i);
    point.pose.position.y = py_p.at(i);
    point.pose.position.z = pz_p.at(i);
    point.pose.orientation = autoware_utils::createQuaternionFromYaw(pyaw_p.at(i));

    point.twist.linear.x = tlx_p.at(i);
    point.twist.angular.z = taz_p.at(i);
    point.accel.linear.x = alx_p.at(i);
    point.accel.angular.z = aaz_p.at(i);
    out_trajectory.points.push_back(point);
  }
  return out_trajectory;
}
}  // namespace resampling
