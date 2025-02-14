// Copyright 2020-2024 Tier IV, Inc.
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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__GEOMETRY_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__GEOMETRY_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"
#include "autoware/universe_utils/math/constants.hpp"
#include "autoware/universe_utils/math/normalization.hpp"
#include "autoware/universe_utils/ros/msg_covariance.hpp"

#include <exception>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

// TODO(wep21): Remove these apis
//              after they are implemented in ros2 geometry2.
namespace tf2
{
void fromMsg(const geometry_msgs::msg::PoseStamped & msg, tf2::Stamped<tf2::Transform> & out);
#ifdef ROS_DISTRO_GALACTIC
// Remove after this commit is released
// https://github.com/ros2/geometry2/commit/e9da371d81e388a589540357c050e262442f1b4a
inline geometry_msgs::msg::Point & toMsg(const tf2::Vector3 & in, geometry_msgs::msg::Point & out)
{
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

// Remove after this commit is released
// https://github.com/ros2/geometry2/commit/e9da371d81e388a589540357c050e262442f1b4a
inline void fromMsg(const geometry_msgs::msg::Point & in, tf2::Vector3 & out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}

template <>
inline void doTransform(
  const geometry_msgs::msg::Point & t_in, geometry_msgs::msg::Point & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Vector3 v_in;
  fromMsg(t_in, v_in);
  tf2::Vector3 v_out = t * v_in;
  toMsg(v_out, t_out);
}

template <>
inline void doTransform(
  const geometry_msgs::msg::Pose & t_in, geometry_msgs::msg::Pose & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Vector3 v;
  fromMsg(t_in.position, v);
  tf2::Quaternion r;
  fromMsg(t_in.orientation, r);

  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Transform v_out = t * tf2::Transform(r, v);
  toMsg(v_out, t_out);
}
#endif
}  // namespace tf2

namespace autoware::universe_utils
{
template <class T>
geometry_msgs::msg::Point getPoint(const T & p)
{
  return geometry_msgs::build<geometry_msgs::msg::Point>().x(p.x).y(p.y).z(p.z);
}

template <>
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Point & p)
{
  return p;
}

template <>
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Pose & p)
{
  return p.position;
}

template <>
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::PoseStamped & p)
{
  return p.pose.position;
}

template <>
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::PoseWithCovarianceStamped & p)
{
  return p.pose.pose.position;
}

template <>
inline geometry_msgs::msg::Point getPoint(const autoware_planning_msgs::msg::PathPoint & p)
{
  return p.pose.position;
}

template <>
inline geometry_msgs::msg::Point getPoint(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p)
{
  return p.point.pose.position;
}

template <>
inline geometry_msgs::msg::Point getPoint(const autoware_planning_msgs::msg::TrajectoryPoint & p)
{
  return p.pose.position;
}

template <class T>
geometry_msgs::msg::Pose getPose([[maybe_unused]] const T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getPose can be used.");
  throw std::logic_error("Only specializations of getPose can be used.");
}

template <>
inline geometry_msgs::msg::Pose getPose(const geometry_msgs::msg::Pose & p)
{
  return p;
}

template <>
inline geometry_msgs::msg::Pose getPose(const geometry_msgs::msg::PoseStamped & p)
{
  return p.pose;
}

template <>
inline geometry_msgs::msg::Pose getPose(const autoware_planning_msgs::msg::PathPoint & p)
{
  return p.pose;
}

template <>
inline geometry_msgs::msg::Pose getPose(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p)
{
  return p.point.pose;
}

template <>
inline geometry_msgs::msg::Pose getPose(const autoware_planning_msgs::msg::TrajectoryPoint & p)
{
  return p.pose;
}

template <class T>
double getLongitudinalVelocity([[maybe_unused]] const T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getVelocity can be used.");
  throw std::logic_error("Only specializations of getVelocity can be used.");
}

template <>
inline double getLongitudinalVelocity(const autoware_planning_msgs::msg::PathPoint & p)
{
  return p.longitudinal_velocity_mps;
}

template <>
inline double getLongitudinalVelocity(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p)
{
  return p.point.longitudinal_velocity_mps;
}

template <>
inline double getLongitudinalVelocity(const autoware_planning_msgs::msg::TrajectoryPoint & p)
{
  return p.longitudinal_velocity_mps;
}

template <class T>
void setPose([[maybe_unused]] const geometry_msgs::msg::Pose & pose, [[maybe_unused]] T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getPose can be used.");
  throw std::logic_error("Only specializations of getPose can be used.");
}

template <>
inline void setPose(const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Pose & p)
{
  p = pose;
}

template <>
inline void setPose(const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::PoseStamped & p)
{
  p.pose = pose;
}

template <>
inline void setPose(
  const geometry_msgs::msg::Pose & pose, autoware_planning_msgs::msg::PathPoint & p)
{
  p.pose = pose;
}

template <>
inline void setPose(
  const geometry_msgs::msg::Pose & pose,
  autoware_internal_planning_msgs::msg::PathPointWithLaneId & p)
{
  p.point.pose = pose;
}

template <>
inline void setPose(
  const geometry_msgs::msg::Pose & pose, autoware_planning_msgs::msg::TrajectoryPoint & p)
{
  p.pose = pose;
}

template <class T>
inline void setOrientation(const geometry_msgs::msg::Quaternion & orientation, T & p)
{
  auto pose = getPose(p);
  pose.orientation = orientation;
  setPose(pose, p);
}

template <class T>
void setLongitudinalVelocity([[maybe_unused]] const float velocity, [[maybe_unused]] T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getLongitudinalVelocity can be used.");
  throw std::logic_error("Only specializations of getLongitudinalVelocity can be used.");
}

template <>
inline void setLongitudinalVelocity(
  const float velocity, autoware_planning_msgs::msg::TrajectoryPoint & p)
{
  p.longitudinal_velocity_mps = velocity;
}

template <>
inline void setLongitudinalVelocity(
  const float velocity, autoware_planning_msgs::msg::PathPoint & p)
{
  p.longitudinal_velocity_mps = velocity;
}

template <>
inline void setLongitudinalVelocity(
  const float velocity, autoware_internal_planning_msgs::msg::PathPointWithLaneId & p)
{
  p.point.longitudinal_velocity_mps = velocity;
}

inline geometry_msgs::msg::Point createPoint(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Quaternion & quat);

geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Pose & pose);

geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseStamped & pose);

geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

geometry_msgs::msg::Quaternion createQuaternion(
  const double x, const double y, const double z, const double w);

geometry_msgs::msg::Vector3 createTranslation(const double x, const double y, const double z);

// Revival of tf::createQuaternionFromRPY
// https://answers.ros.org/question/304397/recommended-way-to-construct-quaternion-from-rollpitchyaw-with-tf2/
geometry_msgs::msg::Quaternion createQuaternionFromRPY(
  const double roll, const double pitch, const double yaw);

geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw);

template <class Point1, class Point2>
double calcDistance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

template <class Point1, class Point2>
double calcSquaredDistance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  const auto dx = p1.x - p2.x;
  const auto dy = p1.y - p2.y;
  return dx * dx + dy * dy;
}

template <class Point1, class Point2>
double calcDistance3d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  // To be replaced by std::hypot(dx, dy, dz) in C++17
  return std::hypot(std::hypot(p1.x - p2.x, p1.y - p2.y), p1.z - p2.z);
}

/**
 * @brief calculate elevation angle of two points.
 * @details This function returns the elevation angle of the position of the two input points
 *          with respect to the origin of their coordinate system.
 *          If the two points are in the same position, the calculation result will be unstable.
 * @param p_from source point
 * @param p_to target point
 * @return -pi/2 <= elevation angle <= pi/2
 */
double calcElevationAngle(
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to);

/**
 * @brief calculate azimuth angle of two points.
 * @details This function returns the azimuth angle of the position of the two input points
 *          with respect to the origin of their coordinate system.
 *          If x and y of the two points are the same, the calculation result will be unstable.
 * @param p_from source point
 * @param p_to target point
 * @return -pi < azimuth angle < pi.
 */
double calcAzimuthAngle(
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to);

geometry_msgs::msg::Pose transform2pose(const geometry_msgs::msg::Transform & transform);

geometry_msgs::msg::PoseStamped transform2pose(
  const geometry_msgs::msg::TransformStamped & transform);

geometry_msgs::msg::Transform pose2transform(const geometry_msgs::msg::Pose & pose);

geometry_msgs::msg::TransformStamped pose2transform(
  const geometry_msgs::msg::PoseStamped & pose, const std::string & child_frame_id);

template <class Point1, class Point2>
tf2::Vector3 point2tfVector(const Point1 & src, const Point2 & dst)
{
  const auto src_p = getPoint(src);
  const auto dst_p = getPoint(dst);

  double dx = dst_p.x - src_p.x;
  double dy = dst_p.y - src_p.y;
  double dz = dst_p.z - src_p.z;
  return tf2::Vector3(dx, dy, dz);
}

Point3d transformPoint(const Point3d & point, const geometry_msgs::msg::Transform & transform);

Point2d transformPoint(const Point2d & point, const geometry_msgs::msg::Transform & transform);

Eigen::Vector3d transformPoint(
  const Eigen::Vector3d & point, const geometry_msgs::msg::Pose & pose);

geometry_msgs::msg::Point transformPoint(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & pose);

geometry_msgs::msg::Point32 transformPoint(
  const geometry_msgs::msg::Point32 & point32, const geometry_msgs::msg::Pose & pose);

template <class T>
T transformVector(const T & points, const geometry_msgs::msg::Transform & transform)
{
  T transformed;
  for (const auto & point : points) {
    transformed.push_back(transformPoint(point, transform));
  }
  return transformed;
}

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform);

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Transform & transform);

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Pose & pose_transform);

// Transform pose in world coordinates to local coordinates
/*
geometry_msgs::msg::Pose inverseTransformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform);
*/

// Transform pose in world coordinates to local coordinates
geometry_msgs::msg::Pose inverseTransformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Transform & transform);

// Transform pose in world coordinates to local coordinates
geometry_msgs::msg::Pose inverseTransformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Pose & transform_pose);

// Transform point in world coordinates to local coordinates
Eigen::Vector3d inverseTransformPoint(
  const Eigen::Vector3d & point, const geometry_msgs::msg::Pose & pose);

// Transform point in world coordinates to local coordinates
geometry_msgs::msg::Point inverseTransformPoint(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & pose);

double calcCurvature(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3);

template <class Pose1, class Pose2>
bool isDrivingForward(const Pose1 & src_pose, const Pose2 & dst_pose)
{
  // check the first point direction
  const double src_yaw = tf2::getYaw(getPose(src_pose).orientation);
  const double pose_direction_yaw = calcAzimuthAngle(getPoint(src_pose), getPoint(dst_pose));
  return std::fabs(normalizeRadian(src_yaw - pose_direction_yaw)) < pi / 2.0;
}

/**
 * @brief Calculate offset pose. The offset values are defined in the local coordinate of the input
 * pose.
 */
geometry_msgs::msg::Pose calcOffsetPose(
  const geometry_msgs::msg::Pose & p, const double x, const double y, const double z,
  const double yaw = 0.0);

/**
 * @brief Calculate a point by linear interpolation.
 * @param src source point
 * @param dst destination point
 * @param ratio interpolation ratio, which should be [0.0, 1.0]
 * @return interpolated point
 */
template <class Point1, class Point2>
geometry_msgs::msg::Point calcInterpolatedPoint(
  const Point1 & src, const Point2 & dst, const double ratio)
{
  const auto src_point = getPoint(src);
  const auto dst_point = getPoint(dst);

  tf2::Vector3 src_vec;
  src_vec.setX(src_point.x);
  src_vec.setY(src_point.y);
  src_vec.setZ(src_point.z);

  tf2::Vector3 dst_vec;
  dst_vec.setX(dst_point.x);
  dst_vec.setY(dst_point.y);
  dst_vec.setZ(dst_point.z);

  // Get pose by linear interpolation
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  const auto & vec = tf2::lerp(src_vec, dst_vec, clamped_ratio);

  geometry_msgs::msg::Point point;
  point.x = vec.x();
  point.y = vec.y();
  point.z = vec.z();

  return point;
}

/**
 * @brief Calculate a pose by linear interpolation.
 * Note that if dist(src_pose, dst_pose)<=0.01
 * the orientation of the output pose is same as the orientation
 * of the dst_pose
 * @param src source point
 * @param dst destination point
 * @param ratio interpolation ratio, which should be [0.0, 1.0]
 * @param set_orientation_from_position_direction set position by spherical interpolation if false
 * @return interpolated point
 */
template <class Pose1, class Pose2>
geometry_msgs::msg::Pose calcInterpolatedPose(
  const Pose1 & src_pose, const Pose2 & dst_pose, const double ratio,
  const bool set_orientation_from_position_direction = true)
{
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);

  geometry_msgs::msg::Pose output_pose;
  output_pose.position =
    calcInterpolatedPoint(getPoint(src_pose), getPoint(dst_pose), clamped_ratio);

  if (set_orientation_from_position_direction) {
    const double input_poses_dist = calcDistance2d(getPoint(src_pose), getPoint(dst_pose));
    const bool is_driving_forward = isDrivingForward(src_pose, dst_pose);

    // Get orientation from interpolated point and src_pose
    if ((is_driving_forward && clamped_ratio > 1.0 - (1e-6)) || input_poses_dist < 1e-3) {
      output_pose.orientation = getPose(dst_pose).orientation;
    } else if (!is_driving_forward && clamped_ratio < 1e-6) {
      output_pose.orientation = getPose(src_pose).orientation;
    } else {
      const auto & base_pose = is_driving_forward ? dst_pose : src_pose;
      const double pitch = calcElevationAngle(getPoint(output_pose), getPoint(base_pose));
      const double yaw = calcAzimuthAngle(getPoint(output_pose), getPoint(base_pose));
      output_pose.orientation = createQuaternionFromRPY(0.0, pitch, yaw);
    }
  } else {
    // Get orientation by spherical linear interpolation
    tf2::Transform src_tf;
    tf2::Transform dst_tf;
    tf2::fromMsg(getPose(src_pose), src_tf);
    tf2::fromMsg(getPose(dst_pose), dst_tf);
    const auto & quaternion = tf2::slerp(src_tf.getRotation(), dst_tf.getRotation(), clamped_ratio);
    output_pose.orientation = tf2::toMsg(quaternion);
  }

  return output_pose;
}

inline geometry_msgs::msg::Vector3 createVector3(const double x, double y, double z)
{
  return geometry_msgs::build<geometry_msgs::msg::Vector3>().x(x).y(y).z(z);
}

inline geometry_msgs::msg::Twist createTwist(
  const geometry_msgs::msg::Vector3 & velocity, geometry_msgs::msg::Vector3 & angular)
{
  return geometry_msgs::build<geometry_msgs::msg::Twist>().linear(velocity).angular(angular);
}

inline double calcNorm(const geometry_msgs::msg::Vector3 & v)
{
  return std::hypot(v.x, v.y, v.z);
}

/**
 * @brief Judge whether twist covariance is valid.
 *
 * @param twist_with_covariance source twist with covariance
 * @return If all element of covariance is 0, return false.
 */
//
bool isTwistCovarianceValid(const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance);

// NOTE: much faster than boost::geometry::intersects()
std::optional<geometry_msgs::msg::Point> intersect(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3, const geometry_msgs::msg::Point & p4);

/**
 * @brief Check if 2 convex polygons intersect using the GJK algorithm
 * @details much faster than boost::geometry::intersects()
 */
bool intersects_convex(const Polygon2d & convex_polygon1, const Polygon2d & convex_polygon2);

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__GEOMETRY_HPP_
