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

#include "multi_object_tracker/tracker/model/pedestrian_tracker.hpp"

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

PedestrianTracker::PedestrianTracker(
  const rclcpp::Time & time, const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & /*self_transform*/)
: Tracker(time, object.classification),
  logger_(rclcpp::get_logger("PedestrianTracker")),
  last_update_time_(time),
  z_(object.kinematics.pose_with_covariance.pose.position.z)
{
  object_ = object;

  // Initialize parameters
  // measurement noise covariance
  float r_stddev_x = 0.4;                                  // [m]
  float r_stddev_y = 0.4;                                  // [m]
  float r_stddev_yaw = tier4_autoware_utils::deg2rad(30);  // [rad]
  ekf_params_.r_cov_x = std::pow(r_stddev_x, 2.0);
  ekf_params_.r_cov_y = std::pow(r_stddev_y, 2.0);
  ekf_params_.r_cov_yaw = std::pow(r_stddev_yaw, 2.0);
  // initial state covariance
  float p0_stddev_x = 2.0;                                    // [m]
  float p0_stddev_y = 2.0;                                    // [m]
  float p0_stddev_yaw = tier4_autoware_utils::deg2rad(1000);  // [rad]
  float p0_stddev_vx = tier4_autoware_utils::kmph2mps(120);   // [m/s]
  float p0_stddev_wz = tier4_autoware_utils::deg2rad(360);    // [rad/s]
  ekf_params_.p0_cov_x = std::pow(p0_stddev_x, 2.0);
  ekf_params_.p0_cov_y = std::pow(p0_stddev_y, 2.0);
  ekf_params_.p0_cov_yaw = std::pow(p0_stddev_yaw, 2.0);
  ekf_params_.p0_cov_vx = std::pow(p0_stddev_vx, 2.0);
  ekf_params_.p0_cov_wz = std::pow(p0_stddev_wz, 2.0);

  // initialize state vector X
  Eigen::MatrixXd X(DIM, 1);
  X(IDX::X) = object.kinematics.pose_with_covariance.pose.position.x;
  X(IDX::Y) = object.kinematics.pose_with_covariance.pose.position.y;
  X(IDX::YAW) = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  if (object.kinematics.has_twist) {
    X(IDX::VEL) = object.kinematics.twist_with_covariance.twist.linear.x;
    X(IDX::WZ) = object.kinematics.twist_with_covariance.twist.angular.z;
  } else {
    X(IDX::VEL) = 0.0;
    X(IDX::WZ) = 0.0;
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
    P(IDX::VEL, IDX::VEL) = ekf_params_.p0_cov_vx;
    P(IDX::WZ, IDX::WZ) = ekf_params_.p0_cov_wz;
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
      P(IDX::WZ, IDX::WZ) =
        object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW];
    } else {
      P(IDX::VEL, IDX::VEL) = ekf_params_.p0_cov_vx;
      P(IDX::WZ, IDX::WZ) = ekf_params_.p0_cov_wz;
    }
  }

  // OBJECT SHAPE MODEL
  bounding_box_ = {0.5, 0.5, 1.7};
  cylinder_ = {0.3, 1.7};
  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    bounding_box_ = {
      object.shape.dimensions.x, object.shape.dimensions.y, object.shape.dimensions.z};
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    cylinder_ = {object.shape.dimensions.x, object.shape.dimensions.z};
  }

  // Set motion model
  motion_model_.init(time, X, P);

  // Set motion model parameters
  constexpr double q_stddev_x = 0.4;                                  // [m/s]
  constexpr double q_stddev_y = 0.4;                                  // [m/s]
  constexpr double q_stddev_yaw = tier4_autoware_utils::deg2rad(20);  // [rad/s]
  constexpr double q_stddev_vx = tier4_autoware_utils::kmph2mps(5);   // [m/(s*s)]
  constexpr double q_stddev_wz = tier4_autoware_utils::deg2rad(30);   // [rad/(s*s)]
  motion_model_.setMotionParams(q_stddev_x, q_stddev_y, q_stddev_yaw, q_stddev_vx, q_stddev_wz);

  // Set motion limits
  motion_model_.setMotionLimits(
    tier4_autoware_utils::kmph2mps(100), /* [m/s] maximum velocity */
    30.0                                 /* [deg/s] maximum turn rate */
  );
}

bool PedestrianTracker::predict(const rclcpp::Time & time)
{
  // predict state vector X t+1
  bool is_predicted = motion_model_.predictState(time);
  if (is_predicted) {
    last_update_time_ = time;
  }
  return is_predicted;
}

bool PedestrianTracker::measureWithPose(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  // position z
  constexpr float gain = 0.9;
  z_ = gain * z_ + (1.0 - gain) * object.kinematics.pose_with_covariance.pose.position.z;

  // motion model
  bool is_updated = false;
  {
    const double x = object.kinematics.pose_with_covariance.pose.position.x;
    const double y = object.kinematics.pose_with_covariance.pose.position.y;

    is_updated =
      motion_model_.updateStatePose(x, y, object.kinematics.pose_with_covariance.covariance);
    motion_model_.limitStates();
  }

  return is_updated;
}

bool PedestrianTracker::measureWithShape(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  constexpr float gain = 0.2;
  constexpr float gain_inv = 1.0 - gain;

  // constexpr float gain = 0.9;
  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    bounding_box_.length = gain_inv * bounding_box_.length + gain * object.shape.dimensions.x;
    bounding_box_.width = gain_inv * bounding_box_.width + gain * object.shape.dimensions.y;
    bounding_box_.height = gain_inv * bounding_box_.height + gain * object.shape.dimensions.z;
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    cylinder_.width = gain_inv * cylinder_.width + gain * object.shape.dimensions.x;
    cylinder_.height = gain_inv * cylinder_.height + gain * object.shape.dimensions.z;
  } else {
    return false;
  }

  // set minimum size
  bounding_box_.length = std::max(bounding_box_.length, 0.3);
  bounding_box_.width = std::max(bounding_box_.width, 0.3);
  bounding_box_.height = std::max(bounding_box_.height, 0.3);
  cylinder_.width = std::max(cylinder_.width, 0.3);
  cylinder_.height = std::max(cylinder_.height, 0.3);

  return true;
}

bool PedestrianTracker::measure(
  const autoware_auto_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & self_transform)
{
  // keep the latest input object
  object_ = object;

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

  measureWithPose(object);
  measureWithShape(object);

  (void)self_transform;  // currently do not use self vehicle position
  return true;
}

bool PedestrianTracker::getTrackedObject(
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

  // // predict state
  // KalmanFilter tmp_ekf_for_no_update = ekf_;
  // const double dt = (time - last_update_time_).seconds();
  // if (0.001 /*1msec*/ < dt) {
  //   predict(dt, tmp_ekf_for_no_update);
  // }
  // Eigen::MatrixXd X_t(DIM, 1);                // predicted state
  // Eigen::MatrixXd P(DIM, DIM);  // predicted state
  // tmp_ekf_for_no_update.getX(X_t);
  // tmp_ekf_for_no_update.getP(P);

  // /*  put predicted pose and twist to output object  */
  // auto & pose_with_cov = object.kinematics.pose_with_covariance;
  // auto & twist_with_cov = object.kinematics.twist_with_covariance;

  // // position
  // pose_with_cov.pose.position.x = X_t(IDX::X);
  // pose_with_cov.pose.position.y = X_t(IDX::Y);
  // pose_with_cov.pose.position.z = z_;
  // // quaternion
  // {
  //   double roll, pitch, yaw;
  //   tf2::Quaternion original_quaternion;
  //   tf2::fromMsg(object_.kinematics.pose_with_covariance.pose.orientation, original_quaternion);
  //   tf2::Matrix3x3(original_quaternion).getRPY(roll, pitch, yaw);
  //   tf2::Quaternion filtered_quaternion;
  //   filtered_quaternion.setRPY(roll, pitch, X_t(IDX::YAW));
  //   pose_with_cov.pose.orientation.x = filtered_quaternion.x();
  //   pose_with_cov.pose.orientation.y = filtered_quaternion.y();
  //   pose_with_cov.pose.orientation.z = filtered_quaternion.z();
  //   pose_with_cov.pose.orientation.w = filtered_quaternion.w();
  //   object.kinematics.orientation_availability =
  //     autoware_auto_perception_msgs::msg::TrackedObjectKinematics::SIGN_UNKNOWN;
  // }
  // // position covariance
  // constexpr double z_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  // constexpr double r_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  // constexpr double p_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  // pose_with_cov.covariance[utils::MSG_COV_IDX::X_X] = P(IDX::X, IDX::X);
  // pose_with_cov.covariance[utils::MSG_COV_IDX::X_Y] = P(IDX::X, IDX::Y);
  // pose_with_cov.covariance[utils::MSG_COV_IDX::Y_X] = P(IDX::Y, IDX::X);
  // pose_with_cov.covariance[utils::MSG_COV_IDX::Y_Y] = P(IDX::Y, IDX::Y);
  // pose_with_cov.covariance[utils::MSG_COV_IDX::Z_Z] = z_cov;
  // pose_with_cov.covariance[utils::MSG_COV_IDX::ROLL_ROLL] = r_cov;
  // pose_with_cov.covariance[utils::MSG_COV_IDX::PITCH_PITCH] = p_cov;
  // pose_with_cov.covariance[utils::MSG_COV_IDX::YAW_YAW] = P(IDX::YAW, IDX::YAW);

  // // twist
  // twist_with_cov.twist.linear.x = X_t(IDX::VEL);
  // twist_with_cov.twist.angular.z = X_t(IDX::WZ);
  // // twist covariance
  // constexpr double vy_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  // constexpr double vz_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  // constexpr double wx_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  // constexpr double wy_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative

  // twist_with_cov.covariance[utils::MSG_COV_IDX::X_X] = P(IDX::VEL, IDX::VEL);
  // twist_with_cov.covariance[utils::MSG_COV_IDX::Y_Y] = vy_cov;
  // twist_with_cov.covariance[utils::MSG_COV_IDX::Z_Z] = vz_cov;
  // twist_with_cov.covariance[utils::MSG_COV_IDX::X_YAW] = P(IDX::VEL, IDX::WZ);
  // twist_with_cov.covariance[utils::MSG_COV_IDX::YAW_X] = P(IDX::WZ, IDX::VEL);
  // twist_with_cov.covariance[utils::MSG_COV_IDX::ROLL_ROLL] = wx_cov;
  // twist_with_cov.covariance[utils::MSG_COV_IDX::PITCH_PITCH] = wy_cov;
  // twist_with_cov.covariance[utils::MSG_COV_IDX::YAW_YAW] = P(IDX::WZ, IDX::WZ);

  // set shape
  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    object.shape.dimensions.x = bounding_box_.length;
    object.shape.dimensions.y = bounding_box_.width;
    object.shape.dimensions.z = bounding_box_.height;
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    object.shape.dimensions.x = cylinder_.width;
    object.shape.dimensions.y = cylinder_.width;
    object.shape.dimensions.z = cylinder_.height;
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    const auto origin_yaw = tf2::getYaw(object_.kinematics.pose_with_covariance.pose.orientation);
    const auto ekf_pose_yaw = tf2::getYaw(pose_with_cov.pose.orientation);
    object.shape.footprint =
      tier4_autoware_utils::rotatePolygon(object.shape.footprint, origin_yaw - ekf_pose_yaw);
  }

  return true;
}
