/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
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
 */

#ifndef UTILS__NMPC_UTILS_HPP_
#define UTILS__NMPC_UTILS_HPP_

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nmpc_utils_eigen.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Converts degree to radians.
namespace ns_utils
{
// ---------------- ROS METHODS -------------------------
inline geometry_msgs::msg::Quaternion createOrientationMsgfromYaw(double const &yaw_angle)
{
	geometry_msgs::msg::Quaternion orientation_msg;
	double roll_angle = 0.0;
	double pitch_angle = 0.0;

	tf2::Quaternion quaternion;
	quaternion.setRPY(roll_angle, pitch_angle, yaw_angle);

	orientation_msg.w = quaternion.getW();
	orientation_msg.x = quaternion.getX();
	orientation_msg.y = quaternion.getY();
	orientation_msg.z = quaternion.getZ();

	return orientation_msg;
}

inline geometry_msgs::msg::Quaternion getQuaternionFromYaw(const double &yaw)
{
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);
	return tf2::toMsg(q);
}

template<typename T>
void computeYawFromXY(
	std::vector<T> const &xvect, std::vector<T> const &yvect, std::vector<T> &yaw_vect);

}  // namespace ns_utils

#endif  // UTILS__NMPC_UTILS_HPP_
