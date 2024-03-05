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

#include "multi_object_tracker/tracker/model/normal_vehicle_tracker.hpp"

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

NormalVehicleTracker::NormalVehicleTracker(
  const rclcpp::Time & time, const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & /*self_transform*/)
: Tracker(time, object.classification),
  logger_(rclcpp::get_logger("NormalVehicleTracker")),
  last_update_time_(time),
  z_(object.kinematics.pose_with_covariance.pose.position.z),
  tracking_offset_(Eigen::Vector2d::Zero())
{
  object_ = object;

  // Initialize parameters
  // measurement noise covariance: detector uncertainty + ego vehicle motion uncertainty
  float r_stddev_x = 0.5;                                  // in vehicle coordinate [m]
  float r_stddev_y = 0.4;                                  // in vehicle coordinate [m]
  float r_stddev_yaw = tier4_autoware_utils::deg2rad(20);  // in map coordinate [rad]
  float r_stddev_vel = 1.0;                                // in object coordinate [m/s]
  ekf_params_.r_cov_x = std::pow(r_stddev_x, 2.0);
  ekf_params_.r_cov_y = std::pow(r_stddev_y, 2.0);
  ekf_params_.r_cov_yaw = std::pow(r_stddev_yaw, 2.0);
  ekf_params_.r_cov_vel = std::pow(r_stddev_vel, 2.0);
  // initial state covariance
  float p0_stddev_x = 1.0;                                     // in object coordinate [m]
  float p0_stddev_y = 0.3;                                     // in object coordinate [m]
  float p0_stddev_yaw = tier4_autoware_utils::deg2rad(25);     // in map coordinate [rad]
  float p0_stddev_vel = tier4_autoware_utils::kmph2mps(1000);  // in object coordinate [m/s]
  float p0_stddev_slip = tier4_autoware_utils::deg2rad(5);     // in object coordinate [rad/s]
  ekf_params_.p0_cov_x = std::pow(p0_stddev_x, 2.0);
  ekf_params_.p0_cov_y = std::pow(p0_stddev_y, 2.0);
  ekf_params_.p0_cov_yaw = std::pow(p0_stddev_yaw, 2.0);
  ekf_params_.p0_cov_vel = std::pow(p0_stddev_vel, 2.0);
  ekf_params_.p0_cov_slip = std::pow(p0_stddev_slip, 2.0);

  // velocity deviation threshold
  //   if the predicted velocity is close to the observed velocity,
  //   the observed velocity is used as the measurement.
  velocity_deviation_threshold_ = tier4_autoware_utils::kmph2mps(10);  // [m/s]

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
    last_input_bounding_box_ = bounding_box_;
  } else {
    autoware_auto_perception_msgs::msg::DetectedObject bbox_object;
    utils::convertConvexHullToBoundingBox(object, bbox_object);
    bounding_box_ = {
      bbox_object.shape.dimensions.x, bbox_object.shape.dimensions.y,
      bbox_object.shape.dimensions.z};
    last_input_bounding_box_ = bounding_box_;
  }

  // Set motion model
  motion_model_.init(time, X, P, bounding_box_.length);

  // Set motion model parameters
  constexpr double q_stddev_acc_long = 9.8 * 0.35;  // [m/(s*s)] uncertain longitudinal acceleration
  constexpr double q_stddev_acc_lat = 9.8 * 0.15;   // [m/(s*s)] uncertain lateral acceleration
  constexpr double q_stddev_yaw_rate_min = 1.5;     // [deg/s] uncertain yaw change rate, minimum
  constexpr double q_stddev_yaw_rate_max = 15.0;    // [deg/s] uncertain yaw change rate, maximum
  constexpr double q_stddev_slip_rate_min =
    0.3;  // [deg/s] uncertain slip angle change rate, minimum
  constexpr double q_stddev_slip_rate_max =
    10.0;                                  // [deg/s] uncertain slip angle change rate, maximum
  constexpr double q_max_slip_angle = 30;  // [deg] max slip angle
  constexpr double lf_ratio = 0.3;         // [-] ratio of front wheel position
  constexpr double lf_min = 1.0;           // [m] minimum front wheel position
  constexpr double lr_ratio = 0.25;        // [-] ratio of rear wheel position
  constexpr double lr_min = 1.0;           // [m] minimum rear wheel position
  motion_model_.setMotionParams(
    q_stddev_acc_long, q_stddev_acc_lat, q_stddev_yaw_rate_min, q_stddev_yaw_rate_max,
    q_stddev_slip_rate_min, q_stddev_slip_rate_max, q_max_slip_angle, lf_ratio, lf_min, lr_ratio,
    lr_min);

  // Set motion limits
  constexpr double max_vel = tier4_autoware_utils::kmph2mps(100);  // [m/s] maximum velocity
  constexpr double max_slip = 30;                                  // [deg] maximum slip angle
  motion_model_.setMotionLimits(max_vel, max_slip);  // maximum velocity and slip angle
}

bool NormalVehicleTracker::predict(const rclcpp::Time & time)
{
  // predict state vector X t+1
  bool is_predicted = motion_model_.predictState(time);
  if (is_predicted) {
    last_update_time_ = time;
  }
  return is_predicted;
}

autoware_auto_perception_msgs::msg::DetectedObject NormalVehicleTracker::getUpdatingObject(
  const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & self_transform)
{
  autoware_auto_perception_msgs::msg::DetectedObject updating_object = object;

  // current (predicted) state
  Eigen::MatrixXd X_t = motion_model_.getStateVector();

  // OBJECT SHAPE MODEL
  // convert to bounding box if input is convex shape
  autoware_auto_perception_msgs::msg::DetectedObject bbox_object;
  if (object.shape.type != autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    utils::convertConvexHullToBoundingBox(object, bbox_object);
  } else {
    bbox_object = object;
  }

  // get offset measurement
  int nearest_corner_index = utils::getNearestCornerOrSurface(
    X_t(IDX::X), X_t(IDX::Y), X_t(IDX::YAW), bounding_box_.width, bounding_box_.length,
    self_transform);
  utils::calcAnchorPointOffset(
    last_input_bounding_box_.width, last_input_bounding_box_.length, nearest_corner_index,
    bbox_object, X_t(IDX::YAW), updating_object, tracking_offset_);

  // UNCERTAINTY MODEL
  if (!object.kinematics.has_position_covariance) {
    // measurement noise covariance
    float r_cov_x;
    float r_cov_y;
    using Label = autoware_auto_perception_msgs::msg::ObjectClassification;
    const uint8_t label = object_recognition_utils::getHighestProbLabel(object.classification);
    if (label == Label::CAR) {
      r_cov_x = ekf_params_.r_cov_x;
      r_cov_y = ekf_params_.r_cov_y;
    } else if (utils::isLargeVehicleLabel(label)) {
      // if label is changed, enlarge the measurement noise covariance
      constexpr float r_stddev_x = 2.0;  // [m]
      constexpr float r_stddev_y = 2.0;  // [m]
      r_cov_x = std::pow(r_stddev_x, 2.0);
      r_cov_y = std::pow(r_stddev_y, 2.0);
    } else {
      r_cov_x = ekf_params_.r_cov_x;
      r_cov_y = ekf_params_.r_cov_y;
    }

    // yaw angle fix
    double pose_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    bool is_yaw_available =
      object.kinematics.orientation_availability !=
      autoware_auto_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;

    // fill covariance matrix
    auto & pose_cov = updating_object.kinematics.pose_with_covariance.covariance;
    const double cos_yaw = std::cos(pose_yaw);
    const double sin_yaw = std::sin(pose_yaw);
    const double sin_2yaw = std::sin(2.0f * pose_yaw);
    pose_cov[utils::MSG_COV_IDX::X_X] =
      r_cov_x * cos_yaw * cos_yaw + r_cov_y * sin_yaw * sin_yaw;                // x - x
    pose_cov[utils::MSG_COV_IDX::X_Y] = 0.5f * (r_cov_x - r_cov_y) * sin_2yaw;  // x - y
    pose_cov[utils::MSG_COV_IDX::Y_Y] =
      r_cov_x * sin_yaw * sin_yaw + r_cov_y * cos_yaw * cos_yaw;            // y - y
    pose_cov[utils::MSG_COV_IDX::Y_X] = pose_cov[utils::MSG_COV_IDX::X_Y];  // y - x
    pose_cov[utils::MSG_COV_IDX::X_YAW] = 0.0;                              // x - yaw
    pose_cov[utils::MSG_COV_IDX::Y_YAW] = 0.0;                              // y - yaw
    pose_cov[utils::MSG_COV_IDX::YAW_X] = 0.0;                              // yaw - x
    pose_cov[utils::MSG_COV_IDX::YAW_Y] = 0.0;                              // yaw - y
    pose_cov[utils::MSG_COV_IDX::YAW_YAW] = ekf_params_.r_cov_yaw;          // yaw - yaw
    if (!is_yaw_available) {
      pose_cov[utils::MSG_COV_IDX::YAW_YAW] *= 1e3;  // yaw is not available, multiply large value
    }
    auto & twist_cov = updating_object.kinematics.twist_with_covariance.covariance;
    twist_cov[utils::MSG_COV_IDX::X_X] = ekf_params_.r_cov_vel;  // vel - vel
  }

  return updating_object;
}

bool NormalVehicleTracker::measureWithPose(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  // current (predicted) state
  Eigen::MatrixXd X_t = motion_model_.getStateVector();

  // MOTION MODEL (update)

  // velocity capability is checked only when the object has velocity measurement
  // and the predicted velocity is close to the observed velocity
  bool is_velocity_available = false;
  if (object.kinematics.has_twist) {
    const double & predicted_vel = X_t(IDX::VEL);
    const double & observed_vel = object.kinematics.twist_with_covariance.twist.linear.x;
    if (std::fabs(predicted_vel - observed_vel) < velocity_deviation_threshold_) {
      // Velocity deviation is small
      is_velocity_available = true;
    }
  }

  // update
  bool is_updated = false;
  {
    const double x = object.kinematics.pose_with_covariance.pose.position.x;
    const double y = object.kinematics.pose_with_covariance.pose.position.y;
    const double yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    const double vel = object.kinematics.twist_with_covariance.twist.linear.x;

    if (is_velocity_available) {
      is_updated = motion_model_.updateStatePoseHeadVel(
        x, y, yaw, vel, object.kinematics.pose_with_covariance.covariance,
        object.kinematics.twist_with_covariance.covariance);
    } else {
      is_updated = motion_model_.updateStatePoseHead(
        x, y, yaw, object.kinematics.pose_with_covariance.covariance);
    }
    motion_model_.limitStates();
  }

  // position z
  constexpr float gain = 0.9;
  z_ = gain * z_ + (1.0 - gain) * object.kinematics.pose_with_covariance.pose.position.z;

  return is_updated;
}

bool NormalVehicleTracker::measureWithShape(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  constexpr float gain = 0.2;
  constexpr float gain_inv = 1.0 - gain;

  // update object size
  bounding_box_.length = gain_inv * bounding_box_.length + gain * object.shape.dimensions.x;
  bounding_box_.width = gain_inv * bounding_box_.width + gain * object.shape.dimensions.y;
  bounding_box_.height = gain_inv * bounding_box_.height + gain * object.shape.dimensions.z;
  last_input_bounding_box_ = {
    object.shape.dimensions.x, object.shape.dimensions.y, object.shape.dimensions.z};

  // update motion model
  motion_model_.updateExtendedState(bounding_box_.length);
  // update offset into object position
  motion_model_.adjustPosition(gain * tracking_offset_.x(), gain * tracking_offset_.y());
  // update offset
  tracking_offset_.x() = gain_inv * tracking_offset_.x();
  tracking_offset_.y() = gain_inv * tracking_offset_.y();

  return true;
}

bool NormalVehicleTracker::measure(
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

bool NormalVehicleTracker::getTrackedObject(
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
    RCLCPP_WARN(logger_, "NormalVehicleTracker::getTrackedObject: Failed to get predicted state.");
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
