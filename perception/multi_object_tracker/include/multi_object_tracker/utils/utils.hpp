// Copyright 2020 Tier IV, Inc.
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
//
//
// Author: v1.0 Yukihiro Saito
//

#ifndef MULTI_OBJECT_TRACKER__UTILS__UTILS_HPP_
#define MULTI_OBJECT_TRACKER__UTILS__UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <cmath>
#include <tuple>
#include <vector>

namespace utils
{
enum MSG_COV_IDX {
  X_X = 0,
  X_Y = 1,
  X_Z = 2,
  X_ROLL = 3,
  X_PITCH = 4,
  X_YAW = 5,
  Y_X = 6,
  Y_Y = 7,
  Y_Z = 8,
  Y_ROLL = 9,
  Y_PITCH = 10,
  Y_YAW = 11,
  Z_X = 12,
  Z_Y = 13,
  Z_Z = 14,
  Z_ROLL = 15,
  Z_PITCH = 16,
  Z_YAW = 17,
  ROLL_X = 18,
  ROLL_Y = 19,
  ROLL_Z = 20,
  ROLL_ROLL = 21,
  ROLL_PITCH = 22,
  ROLL_YAW = 23,
  PITCH_X = 24,
  PITCH_Y = 25,
  PITCH_Z = 26,
  PITCH_ROLL = 27,
  PITCH_PITCH = 28,
  PITCH_YAW = 29,
  YAW_X = 30,
  YAW_Y = 31,
  YAW_Z = 32,
  YAW_ROLL = 33,
  YAW_PITCH = 34,
  YAW_YAW = 35
};

enum BBOX_IDX {
  FRONT_SURFACE = 0,
  RIGHT_SURFACE = 1,
  REAR_SURFACE = 2,
  LEFT_SURFACE = 3,
  FRONT_R_CORNER = 4,
  REAR_R_CORNER = 5,
  REAR_L_CORNER = 6,
  FRONT_L_CORNER = 7,
  INSIDE = -1
};

inline bool isLargeVehicleLabel(const uint8_t label)
{
  using Label = autoware_auto_perception_msgs::msg::ObjectClassification;
  return label == Label::BUS || label == Label::TRUCK || label == Label::TRAILER;
}

/**
 * @brief Get the Nearest Corner or Surface from detected object
 *
 * @param object
 * @return int index
 */
inline int getNearestCornerSurface(
  const double x, const double y, const double yaw, const double width, const double length,
  const geometry_msgs::msg::Transform & self_transform)
{
  double x0, y0, xl, yl;
  // get local vehicle pose
  x0 = self_transform.translation.x;
  y0 = self_transform.translation.y;

  // localize self vehicle pose to object coordinate
  // R.T (X0-X)
  xl = std::cos(yaw) * (x0 - x) + std::sin(yaw) * (y0 - y);
  yl = -std::sin(yaw) * (x0 - x) + std::cos(yaw) * (y0 - y);

  // Determine Index
  //     x+ (front)
  //         __
  // y+     |  | y-
  // (left) |  | (right)
  //         --
  //     x- (rear)
  int xgrid, ygrid;
  const int labels[3][3] = {
    {BBOX_IDX::FRONT_L_CORNER, BBOX_IDX::FRONT_SURFACE, BBOX_IDX::FRONT_R_CORNER},
    {BBOX_IDX::LEFT_SURFACE, BBOX_IDX::INSIDE, BBOX_IDX::RIGHT_SURFACE},
    {BBOX_IDX::REAR_L_CORNER, BBOX_IDX::REAR_SURFACE, BBOX_IDX::REAR_R_CORNER}};
  if (xl > length / 2.0) {
    xgrid = 0;  // front
  } else if (xl > -length / 2.0) {
    xgrid = 1;  // middle
  } else {
    xgrid = 2;  // rear
  }
  if (yl > width / 2.0) {
    ygrid = 0;  // left
  } else if (yl > -width / 2.0) {
    ygrid = 1;  // middle
  } else {
    ygrid = 2;  // right
  }

  return labels[xgrid][ygrid];  // 0 to 7 + 1(null) value
}

/**
 * @brief Get the Nearest Corner or Surface from detected object
 *
 * @param object
 * @return int index
 */
inline int getNearestCornerSurfaceFromObject(
  const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & self_transform)
{
  // only work for BBOX shape
  if (object.shape.type != autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return false;
  }

  double x, y, yaw, width, length;
  x = object.kinematics.pose_with_covariance.pose.position.x;
  y = object.kinematics.pose_with_covariance.pose.position.y;
  yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  width = object.shape.dimensions.y;
  length = object.shape.dimensions.x;

  return getNearestCornerSurface(x, y, yaw, width, length, self_transform);
}

inline Eigen::Vector2d getTrackingPoint(
  const double x, const double y, const double yaw, const double w, const double l, const int indx)
{
  const Eigen::Vector2d center{x, y};
  Eigen::Matrix2d Rot = Eigen::Rotation2Dd(yaw).toRotationMatrix();
  Eigen::Vector2d tracking_point;
  // front tracking point
  if (
    indx == BBOX_IDX::FRONT_L_CORNER || indx == BBOX_IDX::FRONT_R_CORNER ||
    indx == BBOX_IDX::FRONT_SURFACE) {
    Eigen::Vector2d offset_vec{l / 2.0, 0.0};
    tracking_point = center + Rot * offset_vec;
  } else if (
    indx == BBOX_IDX::REAR_L_CORNER || indx == BBOX_IDX::REAR_R_CORNER ||
    indx == BBOX_IDX::REAR_SURFACE) {
    // rear tracking point
    Eigen::Vector2d offset_vec{-l / 2.0, 0.0};
    tracking_point = center + Rot * offset_vec;
  } else {
    // side
    tracking_point = center;
  }
  return tracking_point;
}

/**
 * @brief Get tracking point for post processing
 */
inline Eigen::Vector2d getTrackingPointFromObject(
  const autoware_auto_perception_msgs::msg::DetectedObject & object, const int indx)
{
  double x, y, yaw, w, l;
  x = object.kinematics.pose_with_covariance.pose.position.x;
  y = object.kinematics.pose_with_covariance.pose.position.y;
  yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  w = object.shape.dimensions.y;
  l = object.shape.dimensions.x;

  return getTrackingPoint(x, y, yaw, w, l, indx);
}

/**
 * @brief post processing to recover tracking point
 * @note x,y,yaw,w,l : tracking result to be fixed
 * @note indx: closest surface of corner index of
 */
inline Eigen::Vector2d recoverFromTrackingPoint(
  const double x, const double y, const double yaw, const double w, const double l, const int indx,
  const Eigen::Vector2d & tracking_point)
{
  const Eigen::Vector2d center{x, y};
  const Eigen::Matrix2d Rinv = Eigen::Rotation2Dd(-yaw).toRotationMatrix();

  const Eigen::Vector2d tracker_localized_tracking_point = Rinv * (tracking_point - center);
  Eigen::Vector2d longitude_offset{0, 0};
  // front tracking point
  if (
    indx == BBOX_IDX::FRONT_L_CORNER || indx == BBOX_IDX::FRONT_R_CORNER ||
    indx == BBOX_IDX::FRONT_SURFACE) {
    longitude_offset(0) = tracker_localized_tracking_point.x() - l / 2.0;
  } else if (
    indx == BBOX_IDX::REAR_L_CORNER || indx == BBOX_IDX::REAR_R_CORNER ||
    indx == BBOX_IDX::REAR_SURFACE) {
    // rear tracking point
    longitude_offset(0) = tracker_localized_tracking_point.x() + l / 2.0;
  } else {
    // side
    longitude_offset(0) = 0.0;
  }

  const Eigen::Vector2d offset = Rinv.transpose() * longitude_offset;

  return offset;
}

inline Eigen::Vector2d keepNearestFrontOrRearSurface(
  const double x, const double y, const double l_before, const double l_after,
  const double input_yaw, double estimate_yaw, const double indx)
{
  const Eigen::Vector2d center{x, y};
  // fix front or back revert
  while (M_PI_2 <= input_yaw - estimate_yaw) {
    estimate_yaw = estimate_yaw + M_PI;
  }
  while (M_PI_2 <= estimate_yaw - input_yaw) {
    estimate_yaw = estimate_yaw - M_PI;
  }
  Eigen::Matrix2d Rot = Eigen::Rotation2Dd(estimate_yaw).toRotationMatrix();

  Eigen::Vector2d tracking_point;
  // front tracking point
  if (
    indx == BBOX_IDX::FRONT_L_CORNER || indx == BBOX_IDX::FRONT_R_CORNER ||
    indx == BBOX_IDX::FRONT_SURFACE) {
    Eigen::Vector2d offset_vec{(l_after - l_before) / 2.0, 0.0};
    tracking_point = center + Rot * offset_vec;
  } else if (
    indx == BBOX_IDX::REAR_L_CORNER || indx == BBOX_IDX::REAR_R_CORNER ||
    indx == BBOX_IDX::REAR_SURFACE) {
    // rear tracking point
    Eigen::Vector2d offset_vec{-(l_after - l_before) / 2.0, 0.0};
    tracking_point = center + Rot * offset_vec;
  } else {
    // side
    tracking_point = center;
  }
  return tracking_point;
}

/**
 * @brief Calc offset from center to anchor pointGet the Nearest Corner or Surface from detected
 * object
 *
 * @param object
 * @return int index
 */
inline void calcAnchorPointOffset(
  const double w, const double l, const int indx,
  const autoware_auto_perception_msgs::msg::DetectedObject & input_object, const double & yaw,
  autoware_auto_perception_msgs::msg::DetectedObject & offset_object, Eigen::Vector2d & offset)
{
  // copy value
  offset_object = input_object;
  // invalid index
  if (indx == BBOX_IDX::INSIDE) {
    return;  // do nothing
  }

  // current object width and height
  double w_n, l_n;
  l_n = input_object.shape.dimensions.x;
  w_n = input_object.shape.dimensions.y;

  // if surface
  if (indx < 4) {
    const double sign[4][2] = {{1, 0}, {0, -1}, {-1, 0}, {0, 1}};
    offset(0, 0) = sign[indx][0] * (l_n - l) / 2.0;
    offset(1, 0) = sign[indx][1] * (w_n - w) / 2.0;
  } else {
    // corner
    const double sign[4][2] = {{1, -1}, {-1, -1}, {-1, 1}, {1, 1}};
    offset(0, 0) = sign[indx - 4][0] * (l_n - l) / 2.0;
    offset(1, 0) = sign[indx - 4][1] * (w_n - w) / 2.0;
  }

  // const double yaw = tf2::getYaw(input_object.kinematics.pose_with_covariance.pose.orientation);
  Eigen::Matrix2d R = Eigen::Rotation2Dd(yaw).toRotationMatrix();
  Eigen::Vector2d rotated_offset = R * offset;

  offset_object.kinematics.pose_with_covariance.pose.position.x += rotated_offset.x();
  offset_object.kinematics.pose_with_covariance.pose.position.y += rotated_offset.y();
}

}  // namespace utils

#endif  // MULTI_OBJECT_TRACKER__UTILS__UTILS_HPP_
