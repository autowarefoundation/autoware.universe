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

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>
#include "test_nonlinear_mpc_node.hpp"

void NonlinearMPCNodeTestSuit::SetUp()
{
  ASSERT_FALSE(rclcpp::ok());
  rclcpp::init(0, nullptr);
  ASSERT_TRUE(rclcpp::ok());

  // Create the node_options.
  rclcpp::NodeOptions node_options{};

  node_options.append_parameter_override("wheel_radius", 0.39F);
  node_options.append_parameter_override("wheel_width", 0.42F);
  node_options.append_parameter_override("wheel_base", 2.74F);
  node_options.append_parameter_override("wheel_tread", 1.63F);
  node_options.append_parameter_override("front_overhang", 1.0F);
  node_options.append_parameter_override("rear_overhang", 1.03F);
  node_options.append_parameter_override("left_overhang", 0.1F);
  node_options.append_parameter_override("right_overhang", 0.1F);
  node_options.append_parameter_override("vehicle_height", 2.5F);

  // Assign the node being tested.
  nmpc_follower_ = std::make_shared<NonlinearMPCNode>(node_options);

  // Create a fake node for publishing and subscriptions.
  fake_node_ = std::make_shared<FakeNode>("fake_node", rclcpp::NodeOptions());

  /*   // Create fake_node subscriptions.
  sub_nmpc_msgs_ =
    fake_node_->create_subscription<autoware_control_msgs::msg::NonlinearMPCPerformanceStamped>(
      "/debug/nmpc_predicted_performance_vars", rclcpp::QoS{1},
      [this](const autoware_control_msgs::msg::NonlinearMPCPerformanceStamped::SharedPtr msg) {
        this->nmpc_msgs_ = *msg;
      });

  sub_ctrl_cmd_msgs_ =
    fake_node_->create_subscription<autoware_control_msgs::msg::ControlCommandStamped>(
      "/mpc_nonlinear/output/control_steering_raw", rclcpp::QoS{1},
      [this](const autoware_control_msgs::msg::ControlCommandStamped::SharedPtr msg) {
        this->ctrl_cmd_msgs_ = *msg;
      }); */
}

void FakeNode::transformBroadcast()
{
  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    get_logger(), *get_clock(), (1000ms).count(), "In the FakeNode transformBroadcast ");

  /* !!! this should be defined before sendTransform() !!! */
  static std::shared_ptr<tf2_ros::TransformBroadcaster> br =
    std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  geometry_msgs::msg::TransformStamped sent;
  rclcpp::Time current_time = this->now();

  auto point = trajectory_->points.at(ind_pose_);

  sent.header.stamp = current_time;
  sent.header.frame_id = "map";
  sent.child_frame_id = "base_link";
  sent.transform.translation.x = point.pose.position.x;
  sent.transform.translation.y = point.pose.position.y;
  sent.transform.translation.z = 0.0;

  // tf2::Quaternion q;
  // q.setRPY(0, 0, 0.5);
  auto q = point.pose.orientation;
  sent.transform.rotation.x = q.x;
  sent.transform.rotation.y = q.y;
  sent.transform.rotation.z = q.z;
  sent.transform.rotation.w = q.w;

  br->sendTransform(sent);
}

void FakeNode::publishTrajectory()
{
  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    get_logger(), *get_clock(), (1000ms).count(), "In the FakeNode publishTrajectory ");

  // Create a dummy trajectory message.
  // autoware_planning_msgs::msg::Trajectory traj_msg;
  // createTrajectoryMessage(traj_msg);

  // Publish the trajectory.
  auto traj_msg = *trajectory_;
  traj_msg.header.frame_id = "base_link";
  traj_msg.header.stamp = this->now();
  pub_dummy_traj_->publish(traj_msg);

  //  for (size_t k = 0; k < traj_msg.points.size(); ++k) {
  //    auto point = traj_msg.points.at(k);
  //    ns_utils::print(
  //    "Traj point poses : ", point.pose.position.x, point.pose.position.y, point.pose.position.z);
  //    ns_utils::print("Traj point vx : ", point.twist.linear.x);
  //  }
}

void FakeNode::publishVehicleMeasuredSpeed()
{
  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    get_logger(), *get_clock(), (1000ms).count(), "In the FakeNode publishVehicleMeasuredSpeed ");

  geometry_msgs::msg::TwistStamped twist_msg{};

  twist_msg.header.frame_id = "base_link";
  twist_msg.header.stamp = this->now();

  twist_msg.twist.linear.x = initial_speed;

  pub_dummy_velocity_->publish(twist_msg);
}

void FakeNode::publishVehicleMeasuredSteering()
{
  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    get_logger(), *get_clock(), (1000ms).count(),
    "In the FakeNode publishVehicleMeasuredSteering ");

  autoware_vehicle_msgs::msg::Steering steering_msg{};
  steering_msg.header.frame_id = "base_link";
  steering_msg.header.stamp = this->now();

  steering_msg.data = 0.0;

  pub_dummy_steering_->publish(steering_msg);
}

void FakeNode::initTimer(double period_s)
{
  auto timer_callback = std::bind(&FakeNode::onTimer, this);

  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());

  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void FakeNode::onTimer()
{
  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    get_logger(), *get_clock(), (1000ms).count(), "In the FakeNode onTimer ");

  transformBroadcast();
  publishTrajectory();
  publishVehicleMeasuredSpeed();
  publishVehicleMeasuredSteering();
}

void FakeNode::onNMPCPerformance(
  autoware_control_msgs::msg::NonlinearMPCPerformanceStamped::SharedPtr const msg)
{
  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    get_logger(), *get_clock(), (1000ms).count(), "In the FakeNode onNMPCPerformance ");
  nmpc_msgs_ = *msg;
}

void FakeNode::onNMPCControl(autoware_control_msgs::msg::ControlCommandStamped::SharedPtr const msg)
{
  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    get_logger(), *get_clock(), (1000ms).count(), "In the FakeNode onNMPCControl ");
  ctrl_cmd_msgs_ = *msg;
}

// Helper methods.
std::vector<double> logisticMap(
  double const & max_y_value, double const & center_x_value, double const & slope,
  std::vector<double> const & x_coord)
{
  std::vector<double> ycurve_output;

  for (auto & x : x_coord) {
    auto yval = max_y_value / (1. + std::exp(-slope * (x - center_x_value)));
    // ns_utils::print("yval : ", yval);
    ycurve_output.emplace_back(yval);
  }

  // Debug
  // ns_utils::print_container(ycurve_output);
  // end of debug.

  return ycurve_output;
}

std::vector<double> logisticMapDerivative(
  double const & max_y_value, double const & center_x_value, double const & slope,
  std::vector<double> const & x_coord)
{
  std::vector<double> ycurve_output;

  for (auto & x : x_coord) {
    auto yval = max_y_value / (1. + std::exp(-slope * (x - center_x_value)));

    // ns_utils::print("yval : ", yval);
    ycurve_output.emplace_back(yval * (1 - yval));
  }

  // Debug
  // ns_utils::print_container(ycurve_output);
  // end of debug.

  return ycurve_output;
}

void createTrajectoryMessage(autoware_planning_msgs::msg::Trajectory & traj_msg)
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
  std::transform(dy_vect.cbegin(), dy_vect.cend(), std::back_inserter(yaw_vect), [&dx](auto & dy) {
    return std::atan2(dy, dx);
  });

  // Create the trajectory points and put in the msg container.
  for (size_t k = 0; k < x_vect.size(); ++k) {
    autoware_planning_msgs::msg::TrajectoryPoint tpoint{};
    tpoint.pose.position.x = x_vect[k];
    tpoint.pose.position.y = y_vect[k];
    tpoint.pose.position.z = 0.;

    // Set the orientation.
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_vect[k]);
    tpoint.pose.orientation = tf2::toMsg(q);

    // Set the speeds.
    tpoint.twist.linear.x = vx_vect[k];

    traj_msg.points.emplace_back(tpoint);
  }

  // Debug
  // ns_utils::print("Generated speed trajectory : ");
  // ns_utils::print_container(vx_vect);

  // ns_utils::print("Generated y-vect : ");
  // ns_utils::print_container(y_vect);

  //  ns_utils::print("created trajectory points : ");
  //  for (auto & point : traj_msg.points) {
  //    auto position = point.pose.position;
  //    auto yaw = tf2::getYaw(point.pose.orientation);
  //    ns_utils::print(position.x, position.y, position.z, yaw);
  //  }

  // end of debug
}
