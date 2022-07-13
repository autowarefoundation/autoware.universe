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

#include "nonlinear_mpc_node/nonlinear_mpc_node_visualization.hpp"
#include <string>

visualization_msgs::msg::Marker createLocationMarker(geometry_msgs::msg::Pose const &waypoint_pose,
																										 const builtin_interfaces::msg::Time &stamp,
																										 std::string const &ns)
{
	auto marker =
		createDefaultMarker("map", stamp, ns, 0, visualization_msgs::msg::Marker::SPHERE, createMarkerScale(1.2, 1.2, 1.2),
												createMarkerColor(0.0, 1.0, 0.0, 1.0));

	marker.pose.position.x = waypoint_pose.position.x;
	marker.pose.position.y = waypoint_pose.position.y;
	marker.pose.position.z = 0.0;

	return marker;
}

bool createAutowarePlanningTrajectoryMsg(Model::trajectory_data_t const &td, std::array<double, 2> const &xy0,
																				 autoware_auto_planning_msgs::msg::Trajectory *autoware_traj)
{
	if (!autoware_traj)
	{
		return false;
	}

	autoware_traj->points.clear();

	// States stored in the td ['xw', 'yw', 'yaw', 's', 'e_y', 'e_yaw', 'Vx', 'delta'].
	auto size_of_trajectory_data = td.nX();

	for (size_t i = 0; i < size_of_trajectory_data; ++i)
	{
		autoware_auto_planning_msgs::msg::TrajectoryPoint p;

		p.pose.position.x = td.X.at(i)(0) + xy0[0];
		p.pose.position.y = td.X.at(i)(1) + xy0[1];
		p.pose.position.z = 0.0;

		p.pose.orientation = ns_nmpc_utils::getQuaternionFromYaw(ns_nmpc_utils::wrapToPi(td.X.at(i)(2)));
		p.longitudinal_velocity_mps = td.X.at(i)(6);
		autoware_traj->points.push_back(p);
	}
	return true;
}
