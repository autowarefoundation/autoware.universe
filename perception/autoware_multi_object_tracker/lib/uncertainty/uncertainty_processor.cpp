// Copyright 2024 Tier IV, Inc.
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
// Author: v1.0 Taekjin Lee

#include "autoware/multi_object_tracker/uncertainty/uncertainty_processor.hpp"

namespace autoware::multi_object_tracker
{
namespace uncertainty
{
using autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

object_model::StateCovariance covarianceFromObjectClass(const ObjectClassification & object_class)
{
  const auto & label = object_class.label;
  ObjectModel obj_class_model(object_model::ObjectModelType::Unknown);
  switch (label) {
    case ObjectClassification::CAR:
      obj_class_model = object_model::normal_vehicle;
      break;
    case ObjectClassification::BUS:
    case ObjectClassification::TRUCK:
    case ObjectClassification::TRAILER:
      obj_class_model = object_model::big_vehicle;
      break;
    case ObjectClassification::BICYCLE:
    case ObjectClassification::MOTORCYCLE:
      obj_class_model = object_model::bicycle;
      break;
    case ObjectClassification::PEDESTRIAN:
      obj_class_model = object_model::pedestrian;
      break;
    default:
      obj_class_model = object_model::normal_vehicle;
      break;
  }
  return obj_class_model.measurement_covariance;
}

DetectedObject modelUncertaintyByClass(
  const DetectedObject & object, const ObjectClassification & object_class)
{
  DetectedObject updating_object = object;

  // measurement noise covariance
  const object_model::StateCovariance measurement_covariance =
    covarianceFromObjectClass(object_class);

  const auto & r_cov_x = measurement_covariance.pos_x;
  const auto & r_cov_y = measurement_covariance.pos_y;

  // yaw angle
  const double pose_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);

  // fill position covariance matrix
  auto & pose_cov = updating_object.kinematics.pose_with_covariance.covariance;
  const double cos_yaw = std::cos(pose_yaw);
  const double sin_yaw = std::sin(pose_yaw);
  const double sin_2yaw = std::sin(2.0 * pose_yaw);
  pose_cov[XYZRPY_COV_IDX::X_X] =
    r_cov_x * cos_yaw * cos_yaw + r_cov_y * sin_yaw * sin_yaw;           // x - x
  pose_cov[XYZRPY_COV_IDX::X_Y] = 0.5 * (r_cov_x - r_cov_y) * sin_2yaw;  // x - y
  pose_cov[XYZRPY_COV_IDX::Y_Y] =
    r_cov_x * sin_yaw * sin_yaw + r_cov_y * cos_yaw * cos_yaw;     // y - y
  pose_cov[XYZRPY_COV_IDX::Y_X] = pose_cov[XYZRPY_COV_IDX::X_Y];   // y - x
  pose_cov[XYZRPY_COV_IDX::X_YAW] = 0.0;                           // x - yaw
  pose_cov[XYZRPY_COV_IDX::Y_YAW] = 0.0;                           // y - yaw
  pose_cov[XYZRPY_COV_IDX::YAW_X] = 0.0;                           // yaw - x
  pose_cov[XYZRPY_COV_IDX::YAW_Y] = 0.0;                           // yaw - y
  pose_cov[XYZRPY_COV_IDX::YAW_YAW] = measurement_covariance.yaw;  // yaw - yaw
  const bool is_yaw_available =
    object.kinematics.orientation_availability !=
    autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;
  if (!is_yaw_available) {
    pose_cov[XYZRPY_COV_IDX::YAW_YAW] *= 1e3;  // yaw is not available, multiply large value
  }

  // fill twist covariance matrix
  auto & twist_cov = updating_object.kinematics.twist_with_covariance.covariance;
  twist_cov[XYZRPY_COV_IDX::X_X] = measurement_covariance.vel_long;
  twist_cov[XYZRPY_COV_IDX::X_Y] = 0.0;
  twist_cov[XYZRPY_COV_IDX::Y_X] = 0.0;
  twist_cov[XYZRPY_COV_IDX::Y_Y] = measurement_covariance.vel_lat;

  return updating_object;
}

DetectedObjects modelUncertainty(const DetectedObjects & detected_objects)
{
  DetectedObjects updating_objects;
  updating_objects.header = detected_objects.header;
  for (const auto & object : detected_objects.objects) {
    if (object.kinematics.has_position_covariance) {
      updating_objects.objects.push_back(object);
      continue;
    }
    const ObjectClassification & object_class =
      autoware::object_recognition_utils::getHighestProbClassification(object.classification);
    updating_objects.objects.push_back(modelUncertaintyByClass(object, object_class));
  }
  return updating_objects;
}

void normalizeUncertainty(DetectedObjects & detected_objects)
{
  constexpr double min_cov_dist = 1e-4;
  constexpr double min_cov_rad = 1e-6;
  constexpr double min_cov_vel = 1e-4;

  for (auto & object : detected_objects.objects) {
    // normalize position covariance
    auto & pose_cov = object.kinematics.pose_with_covariance.covariance;
    pose_cov[XYZRPY_COV_IDX::X_X] = std::max(pose_cov[XYZRPY_COV_IDX::X_X], min_cov_dist);
    pose_cov[XYZRPY_COV_IDX::Y_Y] = std::max(pose_cov[XYZRPY_COV_IDX::Y_Y], min_cov_dist);
    pose_cov[XYZRPY_COV_IDX::Z_Z] = std::max(pose_cov[XYZRPY_COV_IDX::Z_Z], min_cov_dist);
    pose_cov[XYZRPY_COV_IDX::YAW_YAW] = std::max(pose_cov[XYZRPY_COV_IDX::YAW_YAW], min_cov_rad);

    // normalize twist covariance
    auto & twist_cov = object.kinematics.twist_with_covariance.covariance;
    twist_cov[XYZRPY_COV_IDX::X_X] = std::max(twist_cov[XYZRPY_COV_IDX::X_X], min_cov_vel);
    twist_cov[XYZRPY_COV_IDX::Y_Y] = std::max(twist_cov[XYZRPY_COV_IDX::Y_Y], min_cov_vel);
  }
}

}  // namespace uncertainty
}  // namespace autoware::multi_object_tracker
