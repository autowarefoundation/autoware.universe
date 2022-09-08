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


#include <memory>
#include "nonlinear_mpc_test_node.hpp"

std::shared_ptr<ns_mpc_nonlinear::NonlinearMPCNode> makeNonlinearMPCNode()
{
  // Pass default parameter file to the node
  const auto share_dir = ament_index_cpp::get_package_share_directory("mpc_nonlinear");

  rclcpp::NodeOptions node_options;
  node_options.arguments({"--ros-args", "--params-file",
                          share_dir + "/params/mpc_nonlinear.param.yaml",
                          "--params-file", share_dir + "/params/test_vehicle_info.yaml"});

  auto node = std::make_shared<NonlinearMPCNode>(node_options);

  // Enable all logging in the node
  auto ret = rcutils_logging_set_logger_level(node->get_logger().get_name(),
                                              RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK)
  { std::cout << "Failed to set logging severity to DEBUG\n"; }

  return node;
}

// Helper methods.
std::vector<double> logisticMap(double const &max_y_value,
                                double const &center_x_value,
                                double const &slope,
                                std::vector<double> const &x_coord)
{
  std::vector<double> ycurve_output;

  for (auto &x : x_coord)
  {
    auto yval = max_y_value / (1. + std::exp(-slope * (x - center_x_value)));
    // ns_utils::print("yval : ", yval);
    ycurve_output.emplace_back(yval);
  }

  // Debug
  ns_utils::print_container(ycurve_output);
  // end of debug.

  return ycurve_output;
}

std::vector<double> logisticMapDerivative(double const &max_y_value,
                                          double const &center_x_value,
                                          double const &slope,
                                          std::vector<double> const &x_coord)
{
  std::vector<double> ycurve_output;

  for (auto &x : x_coord)
  {
    auto yval = max_y_value / (1. + std::exp(-slope * (x - center_x_value)));

    // ns_utils::print("yval : ", yval);
    ycurve_output.emplace_back(yval * (1 - yval));
  }

  // Debug
  ns_utils::print_container(ycurve_output);
  // end of debug.

  return ycurve_output;
}

void createTrajectoryMessage(TrajectoryMsg &traj_msg)
{
  // Generate constant speed logistic function.
  double trajectory_time_length = 50.;  // !<-@brief [seconds].

  // !<-@brief [meters], how far the vehicle moves in the x-coordinate
  double trajectory_x_length = 500.;  // !<-@brief [meters].

  // !<-@brief [meters] how far the vehicle moves in the y-direction,
  double trajectory_max_y = 50.;  // !<-@brief [meters].
  double max_speed = 10.;         // !<-@brief [m/s] maximum speed the speed trajectory will reach.

  size_t N = 501;      // !<-@brief number of points in the trajectory.
  double slope = 0.2;  // see the logisticMap method.
  double t0 = trajectory_time_length / 2.;
  double x0 = trajectory_x_length / 2.;

  // Create a speed trajectory.
  std::vector<double> time_vect = ns_utils::linspace(0., trajectory_time_length, N);
  std::vector<double> x_vect = ns_utils::linspace(0., trajectory_x_length, N);
  double dx = x_vect[1] - x_vect[0];

  auto vx_vect = logisticMap(max_speed, t0, slope, time_vect);
  auto y_vect = logisticMap(trajectory_max_y, x0, slope, x_vect);
  auto dy_vect = logisticMapDerivative(trajectory_max_y, x0, slope, x_vect);
  std::vector<double> yaw_vect;

  // Compute yaw angles of the trajectory.
  std::transform(dy_vect.cbegin(), dy_vect.cend(),
                 std::back_inserter(yaw_vect), [&dx](auto &dy)
                 {
                   return std::atan2(dy, dx);
                 });

  // Create the trajectory points and put in the msg container.
  for (size_t k = 0; k < x_vect.size(); ++k)
  {
    TrajectoryPoint tpoint{};
    tpoint.pose.position.x = x_vect[k];
    tpoint.pose.position.y = y_vect[k];
    tpoint.pose.position.z = 0.;

    // Set the orientation.
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_vect[k]);
    tpoint.pose.orientation = tf2::toMsg(q);

    // Set the speeds.
    tpoint.longitudinal_velocity_mps = static_cast<float>(vx_vect[k]);
    traj_msg.points.emplace_back(tpoint);
  }

  // Debug
  ns_utils::print("Generated speed trajectory : ");
  ns_utils::print_container(vx_vect);

  ns_utils::print("Generated y-vect : ");
  ns_utils::print_container(y_vect);

  ns_utils::print("created trajectory points : ");
  for (auto const &point : traj_msg.points)
  {
    auto position = point.pose.position;
    auto yaw = tf2::getYaw(point.pose.orientation);
    ns_utils::print(position.x, position.y, position.z, yaw);
  }

  // end of debug
}
