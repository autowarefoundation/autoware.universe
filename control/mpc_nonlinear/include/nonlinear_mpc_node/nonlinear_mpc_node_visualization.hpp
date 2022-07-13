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

#ifndef NONLINEAR_MPC_NODE__NONLINEAR_MPC_NODE_VISUALIZATION_HPP_
#define NONLINEAR_MPC_NODE__NONLINEAR_MPC_NODE_VISUALIZATION_HPP_

#include <string>
#include <vector>
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nonlinear_mpc_core/active_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

inline geometry_msgs::msg::Quaternion createMarkerOrientation(double x, double y, double z, double w)
{
	geometry_msgs::msg::Quaternion quaternion;

	quaternion.x = x;
	quaternion.y = y;
	quaternion.z = z;
	quaternion.w = w;

	return quaternion;
}

inline geometry_msgs::msg::Point createMarkerPosition(double x, double y, double z)
{
	geometry_msgs::msg::Point point;

	point.x = x;
	point.y = y;
	point.z = z;

	return point;
}

inline geometry_msgs::msg::Vector3 createMarkerScale(double x, double y, double z)
{
	geometry_msgs::msg::Vector3 scale;

	scale.x = x;
	scale.y = y;
	scale.z = z;

	return scale;
}

inline std_msgs::msg::ColorRGBA createMarkerColor(float r, float g, float b, float a)
{
	std_msgs::msg::ColorRGBA color;

	color.r = r;
	color.g = g;
	color.b = b;
	color.a = a;

	return color;
}

inline visualization_msgs::msg::Marker createDefaultMarker(const std::string_view &frame_id,
																													 const builtin_interfaces::msg::Time &stamp,
																													 const std::string_view &ns,
																													 const int32_t id,
																													 const int32_t type,
																													 const geometry_msgs::msg::Vector3 &scale,
																													 const std_msgs::msg::ColorRGBA &color)
{
	visualization_msgs::msg::Marker marker;

	marker.header.frame_id = frame_id;
	marker.header.stamp = stamp;
	marker.ns = ns;
	marker.id = id;
	marker.type = type;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.lifetime = rclcpp::Duration::from_seconds(0.5);

	marker.pose.position = createMarkerPosition(0.0, 0.0, 0.0);
	marker.pose.orientation = createMarkerOrientation(0.0, 0.0, 0.0, 1.0);
	marker.scale = scale;
	marker.color = color;
	marker.frame_locked = true;

	return marker;
}

visualization_msgs::msg::Marker createLocationMarker(geometry_msgs::msg::Pose const &waypoint_pose,
																										 const builtin_interfaces::msg::Time &stamp,
																										 std::string const &ns);

/**
 * @brief Create a Autoware Planning Trajectory Msg object
 * @param td Trajectory data computed by optimization algorithms consisting of X and U.
 * @param xy0 All trajectories in the optimization space start from zero. xy0 is added back to move from optimization space to the Autoware space. The states stored in the trajectory_data.X are
 * ['xw', 'yw', 'yaw', 's', 'e_y', 'e_yaw', 'Vx', 'delta']
 * @param autoware_traj  Any trajectory created as an autoware_planning_msg::Trajectory.
 * @return true
 * @return false
 */

bool createAutowarePlanningTrajectoryMsg(Model::trajectory_data_t const &td, std::array<double, 2> const &xy0,
																				 autoware_auto_planning_msgs::msg::Trajectory *autoware_traj);

#endif  // NONLINEAR_MPC_NODE__NONLINEAR_MPC_NODE_VISUALIZATION_HPP_
