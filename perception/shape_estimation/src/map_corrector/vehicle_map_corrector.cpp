/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 *
 *
 * v1.0 Yukihiro Saito
 */

#include "vehicle_map_corrector.hpp"
#include "cmath"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

bool VehicleMapCorrector::correct(
  const VectorMap & vector_map, const geometry_msgs::TransformStamped & transform_stamped,
  autoware_perception_msgs::Shape & shape_output, geometry_msgs::Pose & pose_output,
  bool & orientation_output)
{
  geometry_msgs::Point point;
  tf2::doTransform(pose_output.position, point, transform_stamped);
  double map_coord_vector_yaw;
  if (vector_map.getyaw(point.x, point.y, map_coord_vector_yaw)) {
    // calc yaw from vector map
    tf2::Quaternion tf_quaternion;
    double map_coord_tf_roll, map_coord_tf_pitch, map_coord_tf_yaw;
    tf2::fromMsg(transform_stamped.transform.rotation, tf_quaternion);
    tf2::Matrix3x3(tf_quaternion).getRPY(map_coord_tf_roll, map_coord_tf_pitch, map_coord_tf_yaw);
    const double vector_map_yaw = map_coord_vector_yaw - map_coord_tf_yaw;

    // calc yaw from currrent orientation
    double input_roll, input_pitch, input_yaw;
    tf2::Quaternion input_quaternion;
    tf2::fromMsg(pose_output.orientation, input_quaternion);
    tf2::Matrix3x3(input_quaternion).getRPY(input_roll, input_pitch, input_yaw);

    double corrected_yaw;

    if (shape_output.dimensions.x < shape_output.dimensions.y) {
      double input_yaw_90_rotated = input_yaw + M_PI_2;
      double input_yaw_270_rotated = input_yaw + M_PI_2 * 3.0;
      double diff_90(std::atan2(
        std::sin(vector_map_yaw - input_yaw_90_rotated),
        std::cos(vector_map_yaw - input_yaw_90_rotated)));
      double diff_270(std::atan2(
        std::sin(vector_map_yaw - input_yaw_270_rotated),
        std::cos(vector_map_yaw - input_yaw_270_rotated)));
      if (rad_threshold_ < std::min(std::fabs(diff_90), std::fabs(diff_270))) return false;
      if (std::fabs(diff_90) < std::fabs(diff_270)) {
        corrected_yaw = input_yaw_90_rotated;
      } else {
        corrected_yaw = input_yaw_270_rotated;
      }
      double temp = shape_output.dimensions.y;
      shape_output.dimensions.y = shape_output.dimensions.x;
      shape_output.dimensions.x = temp;
    } else {
      double input_yaw_0_rotated = input_yaw;
      double input_yaw_180_rotated = input_yaw + M_PI_2 * 2.0;
      double diff_0(std::atan2(
        std::sin(vector_map_yaw - input_yaw_0_rotated),
        std::cos(vector_map_yaw - input_yaw_0_rotated)));
      double diff_180(std::atan2(
        std::sin(vector_map_yaw - input_yaw_180_rotated),
        std::cos(vector_map_yaw - input_yaw_180_rotated)));
      if (rad_threshold_ < std::min(std::fabs(diff_0), std::fabs(diff_180))) return false;
      if (std::fabs(diff_0) < std::fabs(diff_180)) {
        corrected_yaw = input_yaw_0_rotated;
      } else {
        corrected_yaw = input_yaw_180_rotated;
      }
    }
    tf2::Quaternion output_quaternion;
    output_quaternion.setRPY(0.0, 0.0, corrected_yaw);
    pose_output.orientation = tf2::toMsg(output_quaternion);
    orientation_output = true;
  } else {
    // quaternion.setRPY(0.0, -M_PI / 2.0, 0.0);
    // pose_output.orientation = tf2::toMsg(quaternion);
  }

  return true;
}
