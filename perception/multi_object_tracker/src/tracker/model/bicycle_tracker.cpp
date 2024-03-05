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

#include "multi_object_tracker/tracker/model/bicycle_tracker.hpp"

#include "multi_object_tracker/utils/utils.hpp"

#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <bits/stdc++.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include "object_recognition_utils/object_recognition_utils.hpp"

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

BicycleTracker::BicycleTracker(
  const rclcpp::Time & time, const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & /*self_transform*/)
: Tracker(time, object.classification),
  logger_(rclcpp::get_logger("BicycleTracker")),
  last_update_time_(time),
  z_(object.kinematics.pose_with_covariance.pose.position.z)
{
  object_ = object;

  // Initialize parameters
  // measurement noise covariance: detector uncertainty + ego vehicle motion uncertainty
  float r_stddev_x = 0.5;                                  // in vehicle coordinate [m]
  float r_stddev_y = 0.4;                                  // in vehicle coordinate [m]
  float r_stddev_yaw = tier4_autoware_utils::deg2rad(30);  // in map coordinate [rad]
  ekf_params_.r_cov_x = std::pow(r_stddev_x, 2.0);
  ekf_params_.r_cov_y = std::pow(r_stddev_y, 2.0);
  ekf_params_.r_cov_yaw = std::pow(r_stddev_yaw, 2.0);
  // initial state covariance
  float p0_stddev_x = 0.8;                                     // in object coordinate [m]
  float p0_stddev_y = 0.5;                                     // in object coordinate [m]
  float p0_stddev_yaw = tier4_autoware_utils::deg2rad(25);     // in map coordinate [rad]
  float p0_stddev_vel = tier4_autoware_utils::kmph2mps(1000);  // in object coordinate [m/s]
  float p0_stddev_slip = tier4_autoware_utils::deg2rad(5);     // in object coordinate [rad/s]
  ekf_params_.p0_cov_x = std::pow(p0_stddev_x, 2.0);
  ekf_params_.p0_cov_y = std::pow(p0_stddev_y, 2.0);
  ekf_params_.p0_cov_yaw = std::pow(p0_stddev_yaw, 2.0);
  ekf_params_.p0_cov_vel = std::pow(p0_stddev_vel, 2.0);
  ekf_params_.p0_cov_slip = std::pow(p0_stddev_slip, 2.0);

  // initialize state vector X
  Eigen::MatrixXd X(DIM, 1);
  X(IDX::X) = object.kinematics.pose_with_covariance.pose.position.x;
  X(IDX::Y) = object.kinematics.pose_with_covariance.pose.position.y;
  X(IDX::YAW) = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  X(IDX::SLIP) = 0.0;
  if (object.kinematics.has_twist) {
    X(IDX::VEL) = object.kinematics.twist_with_covariance.twist.linear.x;
  } else {
    X(IDX::VEL) = 0.0;
  }

  // UNCERTAINTY MODEL
  // initialize state covariance matrix P
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(DIM, DIM);
  if (!object.kinematics.has_position_covariance) {
    const double cos_yaw = std::cos(X(IDX::YAW));
    const double sin_yaw = std::sin(X(IDX::YAW));
    const double sin_2yaw = std::sin(2.0f * X(IDX::YAW));
    // Rotate the covariance matrix according to the vehicle yaw
    // because p0_cov_x and y are in the vehicle coordinate system.
    P(IDX::X, IDX::X) =
      ekf_params_.p0_cov_x * cos_yaw * cos_yaw + ekf_params_.p0_cov_y * sin_yaw * sin_yaw;
    P(IDX::X, IDX::Y) = 0.5f * (ekf_params_.p0_cov_x - ekf_params_.p0_cov_y) * sin_2yaw;
    P(IDX::Y, IDX::Y) =
      ekf_params_.p0_cov_x * sin_yaw * sin_yaw + ekf_params_.p0_cov_y * cos_yaw * cos_yaw;
    P(IDX::Y, IDX::X) = P(IDX::X, IDX::Y);
    P(IDX::YAW, IDX::YAW) = ekf_params_.p0_cov_yaw;
    P(IDX::VEL, IDX::VEL) = ekf_params_.p0_cov_vel;
    P(IDX::SLIP, IDX::SLIP) = ekf_params_.p0_cov_slip;
  } else {
    P(IDX::X, IDX::X) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_X];
    P(IDX::X, IDX::Y) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_Y];
    P(IDX::Y, IDX::Y) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
    P(IDX::Y, IDX::X) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_X];
    P(IDX::YAW, IDX::YAW) =
      object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW];
    if (object.kinematics.has_twist_covariance) {
      P(IDX::VEL, IDX::VEL) =
        object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::X_X];
    } else {
      P(IDX::VEL, IDX::VEL) = ekf_params_.p0_cov_vel;
    }
    P(IDX::SLIP, IDX::SLIP) = ekf_params_.p0_cov_slip;
  }

  // OBJECT SHAPE MODEL
  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    bounding_box_ = {
      object.shape.dimensions.x, object.shape.dimensions.y, object.shape.dimensions.z};
  } else {
    bounding_box_ = {1.0, 0.5, 1.7};
  }

  // Set motion model
  motion_model_.init(time, X, P, bounding_box_.length);

  // Set motion model parameters
  constexpr double q_stddev_acc_long = 9.8 * 0.35;  // [m/(s*s)] uncertain longitudinal acceleration
  constexpr double q_stddev_acc_lat = 9.8 * 0.15;   // [m/(s*s)] uncertain lateral acceleration
  constexpr double q_stddev_yaw_rate_min = 5.0;     // [deg/s] uncertain yaw change rate, minimum
  constexpr double q_stddev_yaw_rate_max = 15.0;    // [deg/s] uncertain yaw change rate, maximum
  constexpr double q_stddev_slip_rate_min =
    1.0;  // [deg/s] uncertain slip angle change rate, minimum
  constexpr double q_stddev_slip_rate_max =
    10.0;                                  // [deg/s] uncertain slip angle change rate, maximum
  constexpr double q_max_slip_angle = 30;  // [deg] max slip angle
  constexpr double lf_ratio = 0.3;         // [-] ratio of front wheel position
  constexpr double lf_min = 0.3;           // [m] minimum front wheel position
  constexpr double lr_ratio = 0.3;         // [-] ratio of rear wheel position
  constexpr double lr_min = 0.3;           // [m] minimum rear wheel position
  motion_model_.setMotionParams(
    q_stddev_acc_long, q_stddev_acc_lat, q_stddev_yaw_rate_min, q_stddev_yaw_rate_max,
    q_stddev_slip_rate_min, q_stddev_slip_rate_max, q_max_slip_angle, lf_ratio, lf_min, lr_ratio,
    lr_min);

  // Set motion limits
  constexpr double max_vel = tier4_autoware_utils::kmph2mps(80);  // [m/s] maximum velocity
  constexpr double max_slip = 30;                                 // [deg] maximum slip angle
  motion_model_.setMotionLimits(max_vel, max_slip);               // maximum velocity and slip angle
}

bool BicycleTracker::predict(const rclcpp::Time & time)
{
  // predict state vector X t+1
  bool is_predicted = motion_model_.predictState(time);
  if (is_predicted) {
    last_update_time_ = time;
  }
  return is_predicted;
}

autoware_auto_perception_msgs::msg::DetectedObject BicycleTracker::getUpdatingObject(
  const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & /*self_transform*/)
{
  autoware_auto_perception_msgs::msg::DetectedObject updating_object;

  // OBJECT SHAPE MODEL
  // convert to bounding box if input is convex shape
  if (object.shape.type != autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    utils::convertConvexHullToBoundingBox(object, updating_object);
  } else {
    updating_object = object;
  }

  // UNCERTAINTY MODEL
  if (!object.kinematics.has_position_covariance) {
    auto & pose_cov = updating_object.kinematics.pose_with_covariance.covariance;
    pose_cov[utils::MSG_COV_IDX::X_X] = ekf_params_.r_cov_x;        // x - x
    pose_cov[utils::MSG_COV_IDX::X_Y] = 0;                          // x - y
    pose_cov[utils::MSG_COV_IDX::Y_X] = 0;                          // y - x
    pose_cov[utils::MSG_COV_IDX::Y_Y] = ekf_params_.r_cov_y;        // y - y
    pose_cov[utils::MSG_COV_IDX::YAW_YAW] = ekf_params_.r_cov_yaw;  // yaw - yaw
  }

  return updating_object;
}

bool BicycleTracker::measureWithPose(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  // current (predicted) state
  Eigen::MatrixXd X_t = motion_model_.getStateVector();

  // MOTION MODEL (update)

  // get measurement yaw angle to update
  double measurement_yaw = 0.0;
  bool is_yaw_available = utils::getMeasurementYaw(object, X_t(IDX::YAW), measurement_yaw);

  // update
  bool is_updated = false;
  {
    const double x = object.kinematics.pose_with_covariance.pose.position.x;
    const double y = object.kinematics.pose_with_covariance.pose.position.y;
    const double yaw = measurement_yaw;

    if (is_yaw_available) {
      is_updated = motion_model_.updateStatePoseHead(
        x, y, yaw, object.kinematics.pose_with_covariance.covariance);
    } else {
      is_updated =
        motion_model_.updateStatePose(x, y, object.kinematics.pose_with_covariance.covariance);
    }
    motion_model_.limitStates();
  }

  // position z
  constexpr float gain = 0.9;
  z_ = gain * z_ + (1.0 - gain) * object.kinematics.pose_with_covariance.pose.position.z;

  return is_updated;
}

bool BicycleTracker::measureWithShape(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  constexpr float gain = 0.2;
  constexpr float gain_inv = 1.0 - gain;

  // update object size
  bounding_box_.length = gain_inv * bounding_box_.length + gain * object.shape.dimensions.x;
  bounding_box_.width = gain_inv * bounding_box_.width + gain * object.shape.dimensions.y;
  bounding_box_.height = gain_inv * bounding_box_.height + gain * object.shape.dimensions.z;

  // update motion model
  motion_model_.updateExtendedState(bounding_box_.length);

  return true;
}

bool BicycleTracker::measure(
  const autoware_auto_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & self_transform)
{
  // keep the latest input object
  object_ = object;

  // update classification
  const auto & current_classification = getClassification();
  if (object_recognition_utils::getHighestProbLabel(object.classification) == Label::UNKNOWN) {
    setClassification(current_classification);
  }

  // check time gap
  if (0.01 /*10msec*/ < std::fabs((time - last_update_time_).seconds())) {
    RCLCPP_WARN(
      logger_, "There is a large gap between predicted time and measurement time. (%f)",
      (time - last_update_time_).seconds());
  }

  // update object
  const autoware_auto_perception_msgs::msg::DetectedObject updating_object =
    getUpdatingObject(object, self_transform);
  measureWithPose(updating_object);
  measureWithShape(updating_object);

  return true;
}

bool BicycleTracker::getTrackedObject(
  const rclcpp::Time & time, autoware_auto_perception_msgs::msg::TrackedObject & object) const
{
  object = object_recognition_utils::toTrackedObject(object_);
  object.object_id = getUUID();
  object.classification = getClassification();

  auto & pose_with_cov = object.kinematics.pose_with_covariance;
  auto & twist_with_cov = object.kinematics.twist_with_covariance;

  // predict from motion model
  if (!motion_model_.getPredictedState(
        time, pose_with_cov.pose, pose_with_cov.covariance, twist_with_cov.twist,
        twist_with_cov.covariance)) {
    RCLCPP_WARN(logger_, "BicycleTracker::getTrackedObject: Failed to get predicted state.");
    return false;
  }

  // position
  pose_with_cov.pose.position.z = z_;

  // set shape
  object.shape.dimensions.x = bounding_box_.length;
  object.shape.dimensions.y = bounding_box_.width;
  object.shape.dimensions.z = bounding_box_.height;
  const auto origin_yaw = tf2::getYaw(object_.kinematics.pose_with_covariance.pose.orientation);
  const auto ekf_pose_yaw = tf2::getYaw(pose_with_cov.pose.orientation);
  object.shape.footprint =
    tier4_autoware_utils::rotatePolygon(object.shape.footprint, origin_yaw - ekf_pose_yaw);

  return true;
}
