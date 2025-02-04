// Copyright 2023 TIER IV, Inc.
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

#include "autoware/tracking_object_merger/utils/utils.hpp"

#include "autoware_perception_msgs/msg/shape.hpp"
#include "autoware_perception_msgs/msg/tracked_object.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <unordered_map>

using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
namespace autoware::tracking_object_merger::utils
{

/**
 * @brief linear interpolation for tracked object
 *
 * @param obj1 : obj1 and obj2 must have same shape type
 * @param obj2 : obj1 and obj2 must have same shape type
 * @param weight : 0 <= weight <= 1, weight = 0: obj1, weight = 1: obj2
 * @return TrackedObject : interpolated object
 */
TrackedObject linearInterpolationForTrackedObject(
  const TrackedObject & obj1, const TrackedObject & obj2, const double weight)
{
  // interpolate obj1 and obj2 with weight: obj1 * (1 - weight) + obj2 * weight
  // weight = 0: obj1, weight = 1: obj2

  // weight check (0 <= weight <= 1)
  if (weight < 0 || weight > 1) {
    std::cerr << "weight must be 0 <= weight <= 1 in linearInterpolationForTrackedObject function."
              << std::endl;
    return obj1;
  }

  // output object
  TrackedObject output;
  // copy input object at first
  output = obj1;

  // get current twist and pose
  const auto twist1 = obj1.kinematics.twist_with_covariance.twist;
  const auto pose1 = obj1.kinematics.pose_with_covariance.pose;
  const auto twist2 = obj2.kinematics.twist_with_covariance.twist;
  const auto pose2 = obj2.kinematics.pose_with_covariance.pose;

  // interpolate twist
  auto & output_twist = output.kinematics.twist_with_covariance.twist;
  output_twist.linear.x = twist1.linear.x * (1 - weight) + twist2.linear.x * weight;
  output_twist.linear.y = twist1.linear.y * (1 - weight) + twist2.linear.y * weight;
  output_twist.linear.z = twist1.linear.z * (1 - weight) + twist2.linear.z * weight;
  output_twist.angular.x = twist1.angular.x * (1 - weight) + twist2.angular.x * weight;
  output_twist.angular.y = twist1.angular.y * (1 - weight) + twist2.angular.y * weight;
  output_twist.angular.z = twist1.angular.z * (1 - weight) + twist2.angular.z * weight;

  // interpolate pose
  auto & output_pose = output.kinematics.pose_with_covariance.pose;
  output_pose.position.x = pose1.position.x * (1 - weight) + pose2.position.x * weight;
  output_pose.position.y = pose1.position.y * (1 - weight) + pose2.position.y * weight;
  output_pose.position.z = pose1.position.z * (1 - weight) + pose2.position.z * weight;
  // interpolate orientation with slerp
  // https://en.wikipedia.org/wiki/Slerp
  const auto q1 = tf2::Quaternion(
    pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w);
  const auto q2 = tf2::Quaternion(
    pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w);
  const auto q = q1.slerp(q2, weight);
  output_pose.orientation.x = q.x();
  output_pose.orientation.y = q.y();
  output_pose.orientation.z = q.z();
  output_pose.orientation.w = q.w();

  // interpolate shape(mostly for bounding box)
  const auto shape1 = obj1.shape;
  const auto shape2 = obj2.shape;
  if (shape1.type != shape2.type) {
    // if shape type is different, return obj1
  } else {
    if (shape1.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
      auto & output_shape = output.shape;
      output_shape.dimensions.x = shape1.dimensions.x * (1 - weight) + shape2.dimensions.x * weight;
      output_shape.dimensions.y = shape1.dimensions.y * (1 - weight) + shape2.dimensions.y * weight;
      output_shape.dimensions.z = shape1.dimensions.z * (1 - weight) + shape2.dimensions.z * weight;
    } else if (shape1.type == autoware_perception_msgs::msg::Shape::CYLINDER) {  // NOLINT
      // (TODO) implement and remove NOLINT
    } else if (shape1.type == autoware_perception_msgs::msg::Shape::POLYGON) {  // NOLINT
      // (TODO) implement and remove NOLINT
    } else {
      // when type is unknown, print warning and do nothing
      std::cerr << "unknown shape type in linearInterpolationForTrackedObject function."
                << std::endl;
      return output;
    }
  }

  return output;
}

/**
 * @brief predict past or future tracked object
 *
 * @param obj
 * @param dt
 * @return TrackedObject
 */
TrackedObject predictPastOrFutureTrackedObject(const TrackedObject & obj, const double dt)
{
  // output object
  TrackedObject output;
  // copy input object at first
  output = obj;
  if (dt == 0) {
    return output;
  }

  // get current twist and pose
  const auto twist = obj.kinematics.twist_with_covariance.twist;
  const auto pose = obj.kinematics.pose_with_covariance.pose;

  // get yaw
  const auto yaw = tf2::getYaw(pose.orientation);
  const auto vx = twist.linear.x;
  const auto vy = twist.linear.y;

  // linear prediction equation:
  // x = x0 + vx * cos(yaw) * dt - vy * sin(yaw) * dt
  // y = y0 + vx * sin(yaw) * dt + vy * cos(yaw) * dt
  auto & output_pose = output.kinematics.pose_with_covariance.pose;
  output_pose.position.x += vx * std::cos(yaw) * dt - vy * std::sin(yaw) * dt;
  output_pose.position.y += vx * std::sin(yaw) * dt + vy * std::cos(yaw) * dt;

  // (TODO) covariance prediction

  return output;
}

/**
 * @brief predict past or future tracked objects
 *
 * @param input_objects
 * @param header
 * @return TrackedObjects
 */
TrackedObjects predictPastOrFutureTrackedObjects(
  const TrackedObjects & input_objects, const std_msgs::msg::Header & header)
{
  // for each object, predict past or future
  TrackedObjects output_objects;
  output_objects.header = input_objects.header;
  output_objects.header.stamp = header.stamp;

  const auto dt = (rclcpp::Time(header.stamp) - rclcpp::Time(input_objects.header.stamp)).seconds();
  for (const auto & obj : input_objects.objects) {
    output_objects.objects.push_back(predictPastOrFutureTrackedObject(obj, dt));
  }
  return output_objects;
}

/**
 * @brief interpolate tracked objects
 *
 * @param objects1
 * @param objects2
 * @param header : header of output object
 * @return TrackedObjects
 */
TrackedObjects interpolateTrackedObjects(
  const TrackedObjects & objects1, const TrackedObjects & objects2, std_msgs::msg::Header header)
{
  // Assumption: time of output header is between objects1 and objects2
  //              time of objects1 < header < objects2

  // define output objects
  TrackedObjects output_objects;
  output_objects.header = header;

  // calc weight
  const auto dt =
    (rclcpp::Time(objects1.header.stamp) - rclcpp::Time(objects2.header.stamp)).seconds();
  const auto dt1 = (rclcpp::Time(objects1.header.stamp) - rclcpp::Time(header.stamp)).seconds();
  const auto weight = dt1 / dt;

  // check if object number is not zero
  if (objects1.objects.size() == 0 && objects2.objects.size() == 0) {
    // if objects1 and objects2 are empty, return empty objects
    return output_objects;
  } else if (objects1.objects.size() == 0) {
    // if objects1 is empty return objects2
    for (const auto & obj2 : objects2.objects) {
      output_objects.objects.push_back(predictPastOrFutureTrackedObject(obj2, dt1 - dt));
    }
    return output_objects;
  } else if (objects2.objects.size() == 0) {
    // if objects2 is empty return objects1
    for (const auto & obj1 : objects1.objects) {
      output_objects.objects.push_back(predictPastOrFutureTrackedObject(obj1, dt1));
    }
  } else {
    // if both objects1 and objects2 are not empty do nothing
  }

  // map to handle selected objects
  std::unordered_map<std::size_t, bool> selected_objects1;
  // iterate with int
  for (std::size_t i = 0; i < objects1.objects.size(); i++) {
    selected_objects1[i] = false;
  }
  // search for objects with int iterator
  for (std::size_t i = 0; i < objects1.objects.size(); i++) {
    for (std::size_t j = 0; j < objects2.objects.size(); j++) {
      if (objects1.objects[i].object_id == objects2.objects[j].object_id) {
        // if id is same, interpolate
        const auto interp_objects =
          linearInterpolationForTrackedObject(objects1.objects[i], objects2.objects[j], weight);
        output_objects.objects.push_back(interp_objects);
        selected_objects1[i] = true;
        break;
      }
    }
    if (selected_objects1[i] == false) {
      // if obj1 is not selected, predict future
      const auto pred_objects = predictPastOrFutureTrackedObject(objects1.objects[i], dt1);
      output_objects.objects.push_back(pred_objects);
    }
  }

  return output_objects;
}

}  // namespace autoware::tracking_object_merger::utils

namespace autoware::tracking_object_merger::merger_utils
{

double mean(const double a, const double b)
{
  return (a + b) / 2.0;
}

/**
 * @brief compare two tracked objects motion direction is same or not
 *
 * @param main_obj
 * @param sub_obj
 * @return true
 * @return false
 */
bool objectsHaveSameMotionDirections(const TrackedObject & main_obj, const TrackedObject & sub_obj)
{
  // get yaw
  const auto main_yaw = tf2::getYaw(main_obj.kinematics.pose_with_covariance.pose.orientation);
  const auto sub_yaw = tf2::getYaw(sub_obj.kinematics.pose_with_covariance.pose.orientation);
  // get velocity
  const auto main_vx = main_obj.kinematics.twist_with_covariance.twist.linear.x;
  const auto main_vy = main_obj.kinematics.twist_with_covariance.twist.linear.y;
  const auto sub_vx = sub_obj.kinematics.twist_with_covariance.twist.linear.x;
  const auto sub_vy = sub_obj.kinematics.twist_with_covariance.twist.linear.y;
  // calc velocity direction
  const auto main_v_yaw = std::atan2(main_vy, main_vx);
  const auto sub_v_yaw = std::atan2(sub_vy, sub_vx);
  // get motion yaw angle
  const auto main_motion_yaw = main_yaw + main_v_yaw;
  const auto sub_motion_yaw = sub_yaw + sub_v_yaw;
  // diff of motion yaw angle
  const auto motion_yaw_diff = std::fabs(main_motion_yaw - sub_motion_yaw);
  const auto normalized_motion_yaw_diff =
    autoware::universe_utils::normalizeRadian(motion_yaw_diff);  // -pi ~ pi
  // evaluate if motion yaw angle is same
  constexpr double yaw_threshold = M_PI / 4.0;  // 45 deg
  if (std::abs(normalized_motion_yaw_diff) < yaw_threshold) {
    return true;
  } else {
    return false;
  }
}

// object kinematics merger
// currently only support velocity fusion
autoware_perception_msgs::msg::TrackedObjectKinematics objectKinematicsVXMerger(
  const TrackedObject & main_obj, const TrackedObject & sub_obj, const MergePolicy policy)
{
  autoware_perception_msgs::msg::TrackedObjectKinematics output_kinematics;
  // copy main object at first
  output_kinematics = main_obj.kinematics;
  auto sub_obj_ = sub_obj;
  // do not merge if motion direction is different
  if (!objectsHaveSameMotionDirections(main_obj, sub_obj)) {
    return output_kinematics;
  }

  // currently only merge vx
  if (policy == MergePolicy::SKIP) {
    return output_kinematics;
  } else if (policy == MergePolicy::OVERWRITE) {
    // use main_obj's orientation
    // take sub_obj's velocity vector and convert into main_obj's frame, but take only x component
    const auto sub_vx = sub_obj_.kinematics.twist_with_covariance.twist.linear.x;
    const auto sub_vy = sub_obj_.kinematics.twist_with_covariance.twist.linear.y;
    const auto main_yaw = tf2::getYaw(main_obj.kinematics.pose_with_covariance.pose.orientation);
    const auto sub_yaw = tf2::getYaw(sub_obj_.kinematics.pose_with_covariance.pose.orientation);
    const auto sub_vx_in_main_frame =
      sub_vx * std::cos(sub_yaw - main_yaw) + sub_vy * std::sin(sub_yaw - main_yaw);
    output_kinematics.twist_with_covariance.twist.linear.x = sub_vx_in_main_frame;

    return output_kinematics;
  } else if (policy == MergePolicy::FUSION) {
    // use main_obj's orientation
    // take main_obj's velocity vector and convert into sub_obj's frame, but take only x component
    const auto main_vx = main_obj.kinematics.twist_with_covariance.twist.linear.x;
    const auto sub_vx = sub_obj_.kinematics.twist_with_covariance.twist.linear.x;
    const auto sub_vy = sub_obj_.kinematics.twist_with_covariance.twist.linear.y;
    const auto main_yaw = tf2::getYaw(main_obj.kinematics.pose_with_covariance.pose.orientation);
    const auto sub_yaw = tf2::getYaw(sub_obj_.kinematics.pose_with_covariance.pose.orientation);
    const auto sub_vel_in_main_frame_x =
      sub_vx * std::cos(sub_yaw - main_yaw) + sub_vy * std::sin(sub_yaw - main_yaw);
    // inverse weight
    const auto main_vx_cov = main_obj.kinematics.twist_with_covariance.covariance[0];
    const auto sub_vx_cov = sub_obj_.kinematics.twist_with_covariance.covariance[0];
    const auto sub_vy_cov = sub_obj_.kinematics.twist_with_covariance.covariance[7];
    const auto sub_vel_cov_in_main_frame_x =
      sub_vx_cov * std::cos(sub_yaw - main_yaw) * std::cos(sub_yaw - main_yaw) +
      sub_vy_cov * std::sin(sub_yaw - main_yaw) * std::sin(sub_yaw - main_yaw);
    double main_vx_weight, sub_vx_weight;
    if (main_vx_cov == 0.0) {
      main_vx_weight = 1.0 * 1e6;
    } else {
      main_vx_weight = 1.0 / main_vx_cov;
    }
    if (sub_vel_cov_in_main_frame_x == 0.0) {
      sub_vx_weight = 1.0 * 1e6;
    } else {
      sub_vx_weight = 1.0 / sub_vel_cov_in_main_frame_x;
    }

    // merge velocity with covariance
    output_kinematics.twist_with_covariance.twist.linear.x =
      (main_vx * main_vx_weight + sub_vel_in_main_frame_x * sub_vx_weight) /
      (main_vx_weight + sub_vx_weight);
    output_kinematics.twist_with_covariance.covariance[0] = 1.0 / (main_vx_weight + sub_vx_weight);

    return output_kinematics;
  } else {
    std::cerr << "unknown merge policy in objectKinematicsMerger function." << std::endl;
    return output_kinematics;
  }
  return output_kinematics;
}

// object classification merger
TrackedObject objectClassificationMerger(
  const TrackedObject & main_obj, const TrackedObject & sub_obj, const MergePolicy policy)
{
  if (policy == MergePolicy::SKIP) {
    return main_obj;
  } else if (policy == MergePolicy::OVERWRITE) {
    return sub_obj;
  } else if (policy == MergePolicy::FUSION) {
    // merge two std::vector into one
    TrackedObject dummy_obj;
    dummy_obj.classification = main_obj.classification;
    for (const auto & sub_class : sub_obj.classification) {
      dummy_obj.classification.push_back(sub_class);
    }
    return dummy_obj;
  } else {
    std::cerr << "unknown merge policy in objectClassificationMerger function." << std::endl;
    return main_obj;
  }
}

void updateExceptVelocity(TrackedObject & main_obj, const TrackedObject & sub_obj)
{
  // do not update if motion direction is different
  if (!objectsHaveSameMotionDirections(main_obj, sub_obj)) {
    return;
  }
  // take main_obj's velocity vector and convert into sub_obj's frame
  // use sub_obj's orientation, but take only x component
  const auto main_vx = main_obj.kinematics.twist_with_covariance.twist.linear.x;
  const auto main_vy = main_obj.kinematics.twist_with_covariance.twist.linear.y;
  const auto main_yaw = tf2::getYaw(main_obj.kinematics.pose_with_covariance.pose.orientation);
  const auto sub_yaw = tf2::getYaw(sub_obj.kinematics.pose_with_covariance.pose.orientation);
  const auto main_vx_in_sub_frame =
    main_vx * std::cos(main_yaw - sub_yaw) + main_vy * std::sin(main_yaw - sub_yaw);

  // copy sub object to fused object
  main_obj = sub_obj;
  // recover main object's velocity
  main_obj.kinematics.twist_with_covariance.twist.linear.x = main_vx_in_sub_frame;

  return;
}

void updateOnlyObjectVelocity(TrackedObject & main_obj, const TrackedObject & sub_obj)
{
  main_obj.kinematics = objectKinematicsVXMerger(main_obj, sub_obj, MergePolicy::OVERWRITE);
}

void updateOnlyClassification(TrackedObject & main_obj, const TrackedObject & sub_obj)
{
  main_obj = objectClassificationMerger(main_obj, sub_obj, MergePolicy::OVERWRITE);
}

void updateWholeTrackedObject(TrackedObject & main_obj, const TrackedObject & sub_obj)
{
  if (!objectsHaveSameMotionDirections(main_obj, sub_obj)) {
    // warning
    // std::cerr << "[object_tracking_merger]: motion direction is different in "
    //              "updateWholeTrackedObject function."
    //           << std::endl;
  }
  main_obj = sub_obj;
}

}  // namespace autoware::tracking_object_merger::merger_utils
