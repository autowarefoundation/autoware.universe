// Copyright 2022 TIER IV, Inc.
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

#include "autoware/lidar_centerpoint/ros_utils.hpp"

#include "autoware/object_recognition_utils/object_recognition_utils.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/math/constants.hpp"

#include <string>
#include <vector>

namespace autoware::lidar_centerpoint
{

using Label = autoware_perception_msgs::msg::ObjectClassification;

void box3DToDetectedObject(
  const Box3D & box3d, const std::vector<std::string> & class_names, const bool has_twist,
  const bool has_variance, autoware_perception_msgs::msg::DetectedObject & obj)
{
  // TODO(yukke42): the value of classification confidence of DNN, not probability.
  obj.existence_probability = box3d.score;

  // classification
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.probability = 1.0f;
  if (box3d.label >= 0 && static_cast<size_t>(box3d.label) < class_names.size()) {
    classification.label = getSemanticType(class_names[box3d.label]);
  } else {
    classification.label = Label::UNKNOWN;
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("lidar_centerpoint"), "Unexpected label: UNKNOWN is set.");
  }

  if (autoware::object_recognition_utils::isCarLikeVehicle(classification.label)) {
    obj.kinematics.orientation_availability =
      autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
  }

  obj.classification.emplace_back(classification);

  // pose and shape
  // mmdet3d yaw format to ros yaw format
  const float yaw = -box3d.yaw - autoware::universe_utils::pi / 2;
  obj.kinematics.pose_with_covariance.pose.position =
    autoware::universe_utils::createPoint(box3d.x, box3d.y, box3d.z);
  obj.kinematics.pose_with_covariance.pose.orientation =
    autoware::universe_utils::createQuaternionFromYaw(yaw);
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions =
    autoware::universe_utils::createTranslation(box3d.length, box3d.width, box3d.height);
  if (has_variance) {
    obj.kinematics.has_position_covariance = has_variance;
    obj.kinematics.pose_with_covariance.covariance = convertPoseCovarianceMatrix(box3d);
  }

  // twist
  if (has_twist) {
    float vel_x = box3d.vel_x;
    float vel_y = box3d.vel_y;

    // twist of the object is based on the object coordinate system
    geometry_msgs::msg::Twist twist;
    twist.linear.x = std::cos(yaw) * vel_x + std::sin(yaw) * vel_y;
    twist.linear.y = -std::sin(yaw) * vel_x + std::cos(yaw) * vel_y;
    twist.angular.z = 0;  // angular velocity is not supported

    obj.kinematics.twist_with_covariance.twist = twist;
    obj.kinematics.has_twist = has_twist;
    if (has_variance) {
      obj.kinematics.has_twist_covariance = has_variance;
      obj.kinematics.twist_with_covariance.covariance = convertTwistCovarianceMatrix(box3d, yaw);
    }
  }
}

uint8_t getSemanticType(const std::string & class_name)
{
  if (class_name == "CAR") {
    return Label::CAR;
  } else if (class_name == "TRUCK") {
    return Label::TRUCK;
  } else if (class_name == "BUS") {
    return Label::BUS;
  } else if (class_name == "TRAILER") {
    return Label::TRAILER;
  } else if (class_name == "BICYCLE") {
    return Label::BICYCLE;
  } else if (class_name == "MOTORBIKE") {
    return Label::MOTORCYCLE;
  } else if (class_name == "PEDESTRIAN") {
    return Label::PEDESTRIAN;
  } else {
    return Label::UNKNOWN;
  }
}

std::array<double, 36> convertPoseCovarianceMatrix(const Box3D & box3d)
{
  using POSE_IDX = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  std::array<double, 36> pose_covariance{};
  pose_covariance[POSE_IDX::X_X] = box3d.x_variance;
  pose_covariance[POSE_IDX::Y_Y] = box3d.y_variance;
  pose_covariance[POSE_IDX::Z_Z] = box3d.z_variance;
  pose_covariance[POSE_IDX::YAW_YAW] = box3d.yaw_variance;
  return pose_covariance;
}

std::array<double, 36> convertTwistCovarianceMatrix(const Box3D & box3d, const float yaw)
{
  using POSE_IDX = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

  // twist covariance matrix is based on the object coordinate system
  std::array<double, 36> twist_covariance{};
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);
  twist_covariance[POSE_IDX::X_X] =
    box3d.vel_x_variance * cos_yaw * cos_yaw + box3d.vel_y_variance * sin_yaw * sin_yaw;
  twist_covariance[POSE_IDX::X_Y] =
    (box3d.vel_y_variance - box3d.vel_x_variance) * sin_yaw * cos_yaw;
  twist_covariance[POSE_IDX::Y_X] = twist_covariance[POSE_IDX::X_Y];
  twist_covariance[POSE_IDX::Y_Y] =
    box3d.vel_x_variance * sin_yaw * sin_yaw + box3d.vel_y_variance * cos_yaw * cos_yaw;
  return twist_covariance;
}

}  // namespace autoware::lidar_centerpoint
