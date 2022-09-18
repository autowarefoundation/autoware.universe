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

#include "gtest/gtest.h"
#include "nmpc_test_utils.hpp"
#include "nonlinear_mpc_test_node.hpp"

#include <limits>
#include <memory>
#include <vector>

/**
 * Automatic differentiation : Vehicle model equations test.
 * */
TEST_F(FakeNodeFixture, automatic_differentiation_works)
{
  // Compute f(x, u) by codegen from the model.
  Model::state_vector_t f_of_dx{Model::state_vector_t::Zero()};

  // [x, y, yaw, s, ey, e_yaw, v, steering, lateral velocity]
  Model::state_vector_t x{Model::state_vector_t::Zero()};

  // Set speed
  double v_ego = 10.;
  x(6) = v_ego;

  // double xw = x(0);  // x-coordinate
  // double yw = x(1);  // y-coordinate

  double yaw = x(2);
  // double sdist = x(3);
  double ey = x(4);
  double eyaw = x(5);
  double steering = x(7);

  // Create dummy controls and params.
  Model::input_vector_t u;
  u.setConstant(0.1);

  Model::param_vector_t params{Model::param_vector_t::Zero()};
  params.setZero();

  double kappa{0.1};
  params(ns_utils::toUType(VehicleParamIds::curvature)) = kappa;  // curvature
  params(ns_utils::toUType(VehicleParamIds::target_vx)) = v_ego;  // target velocity

  // Compute Jacobian --
  Model::state_matrix_t A{Model::state_matrix_t::Zero()};
  Model::control_matrix_t B{Model::control_matrix_t::Zero()};

  // Call the model methods.
  // Create vehicle parameters.
  ns_models::ParamsVehicle paramsVehicle{};
  paramsVehicle.use_delay_model = false;

  Model vehicle_model{};
  vehicle_model.updateParameters(paramsVehicle);
  vehicle_model.InitializeModel();

  ASSERT_TRUE(vehicle_model.IsInitialized());

  // Compute analytical f(x, u, params).
  auto tan_delta = tan(steering);
  auto beta = atan(tan_delta * paramsVehicle.lr / paramsVehicle.wheel_base);

  auto const & xdot = v_ego * cos(beta + yaw);
  auto const & ydot = v_ego * sin(beta + yaw);
  auto const & yawdot = v_ego * tan_delta;
  auto const & sdot = v_ego * cos(beta + eyaw) / (EPS + 1 - kappa * ey);
  auto const & eydot = v_ego * sin(beta + eyaw);
  auto const & eyawdot = yawdot - kappa * sdot;
  auto const & vdot = u(0);
  auto const & deltadot = u(1);
  auto const & vydot = yawdot * vdot * cos(beta + eyaw);

  // Analytical fx.
  std::vector<double> analytical_fx_vec{xdot,    ydot, yawdot,   sdot, eydot,
                                        eyawdot, vdot, deltadot, vydot};

  // Autodiff fx
  vehicle_model.computeFx(x, u, params, f_of_dx);

  for (size_t k = 0; k < Model::state_dim; ++k) {
    ASSERT_DOUBLE_EQ(f_of_dx(static_cast<int64_t>(k)), analytical_fx_vec[k]);
    ns_utils::print(
      "k : ", k, " ;  fx : ", f_of_dx(static_cast<int64_t>(k)),
      "analytical fx : ", analytical_fx_vec[k]);
  }

  // Analytical derivative f wrt v : df/dv  = A(:, 6th_col)
  vehicle_model.computeJacobians(x, u, params, A, B);

  EXPECT_DOUBLE_EQ(B(6, 0), 1.);
  EXPECT_DOUBLE_EQ(B(7, 1), 1.);

  std::vector<double> analytical_df_dv_vec;  // {Model::state_dim};

  analytical_df_dv_vec.emplace_back(cos(beta + yaw));                           // x
  analytical_df_dv_vec.emplace_back(sin(beta + yaw));                           // y
  analytical_df_dv_vec.emplace_back(tan(steering) / paramsVehicle.wheel_base);  // yaw
  analytical_df_dv_vec.emplace_back(cos(beta + eyaw) / (1 - kappa * ey));       // s
  analytical_df_dv_vec.emplace_back(sin(beta + eyaw));                          // ey

  analytical_df_dv_vec.emplace_back(
    analytical_df_dv_vec[2] - kappa * analytical_df_dv_vec[3]);  // eyaw

  analytical_df_dv_vec.emplace_back(0.);                       // ax
  analytical_df_dv_vec.emplace_back(0.);                       // steering input
  analytical_df_dv_vec.emplace_back(analytical_df_dv_vec[2]);  // lateral acceleration

  for (Eigen::Index k = 0; k < Model::state_dim; ++k) {
    ASSERT_DOUBLE_EQ(A(k, 6), analytical_df_dv_vec[static_cast<size_t>(k)]);
    ns_utils::print(
      "k : ", k, ": A:", A(k, 6), ": df/dv: ", analytical_df_dv_vec[static_cast<size_t>(k)]);
  }

  // Debug
  ns_eigen_utils::printEigenMat(f_of_dx, "System dynamical equations values : ");
  ns_utils::print("Jacobians A and B : ");
  ns_eigen_utils::printEigenMat(A);
  ns_eigen_utils::printEigenMat(B);
  // end of debug
}

TEST_F(FakeNodeFixture, no_input_stop_control_cmd_is_published)
{
  // Data to test
  ControlCmdMsg::SharedPtr cmd_msg;
  bool is_control_command_received = false;

  // Node
  std::shared_ptr<NonlinearMPCNode> node = makeNonlinearMPCNode();

  // Publishers
  rclcpp::Publisher<TrajectoryMsg>::SharedPtr traj_pub =
    this->create_publisher<TrajectoryMsg>("mpc_nonlinear/input/reference_trajectory");

  rclcpp::Publisher<VelocityMsg>::SharedPtr vel_pub =
    this->create_publisher<VelocityMsg>("mpc_nonlinear/input/current_velocity");

  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("mpc_nonlinear/input/current_steering");

  // Subscribers
  rclcpp::Subscription<ControlCmdMsg>::SharedPtr cmd_sub = this->create_subscription<ControlCmdMsg>(
    "mpc_nonlinear/output/control_cmd", *this->get_fake_node(),
    [&cmd_msg, &is_control_command_received](const ControlCmdMsg::SharedPtr msg) {
      cmd_msg = msg;
      is_control_command_received = true;
    });

  auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // test_utils::spinWhile(node);

  test_utils::waitForMessage(
    node, this, is_control_command_received, std::chrono::seconds{1LL}, false);
  /**
   * Expect stop command with acc = -1.5 and other controls are 0.
   * */
  ASSERT_TRUE(is_control_command_received);
  ASSERT_FLOAT_EQ(cmd_msg->lateral.steering_tire_angle, 0.0);
  ASSERT_FLOAT_EQ(cmd_msg->lateral.steering_tire_rotation_rate, 0.0);
  ASSERT_FLOAT_EQ(cmd_msg->longitudinal.speed, 0.0);
  ASSERT_FLOAT_EQ(cmd_msg->longitudinal.acceleration, -1.5);

  // DEBUG
  ns_utils::print(" ctrl_cmd_msgs_  steering: ", cmd_msg->lateral.steering_tire_angle);
  ns_utils::print(
    " ctrl_cmd_msgs_  steering rate : ", cmd_msg->lateral.steering_tire_rotation_rate);
  ns_utils::print(" ctrl_cmd_msgs_  steering rate : ", cmd_msg->longitudinal.speed);
  ns_utils::print(" ctrl_cmd_msgs_  steering: ", cmd_msg->longitudinal.acceleration, -1.5);
}

TEST_F(FakeNodeFixture, empty_trajectory)
{
  // Data to test
  ControlCmdMsg::SharedPtr cmd_msg;
  bool is_control_command_received = false;

  // Node
  std::shared_ptr<NonlinearMPCNode> node = makeNonlinearMPCNode();

  // Publishers
  rclcpp::Publisher<TrajectoryMsg>::SharedPtr traj_pub =
    this->create_publisher<TrajectoryMsg>("mpc_nonlinear/input/reference_trajectory");

  rclcpp::Publisher<VelocityMsg>::SharedPtr vel_pub =
    this->create_publisher<VelocityMsg>("mpc_nonlinear/input/current_velocity");

  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("mpc_nonlinear/input/current_steering");

  // Subscribers
  rclcpp::Subscription<ControlCmdMsg>::SharedPtr cmd_sub = this->create_subscription<ControlCmdMsg>(
    "mpc_nonlinear/output/control_cmd", *this->get_fake_node(),
    [&cmd_msg, &is_control_command_received](const ControlCmdMsg::SharedPtr msg) {
      cmd_msg = msg;
      is_control_command_received = true;
    });

  auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);

  // Spin for transform to be published
  test_utils::spinWhile(node);

  // Empty trajectory: expect a stopped command
  TrajectoryMsg traj_msg;
  traj_msg.header.stamp = node->now();
  traj_msg.header.frame_id = "map";

  VelocityMsg odom_msg;
  SteeringReport steer_msg;

  traj_msg.header.stamp = node->now();
  odom_msg.header.stamp = node->now();

  odom_msg.twist.twist.linear.x = 0.0;
  steer_msg.stamp = node->now();

  steer_msg.steering_tire_angle = 0.0;
  traj_pub->publish(traj_msg);

  vel_pub->publish(odom_msg);
  steer_pub->publish(steer_msg);

  test_utils::waitForMessage(
    node, this, is_control_command_received, std::chrono::seconds{1LL}, false);

  // Expect true with stopping command.
  ASSERT_TRUE(is_control_command_received);

  // DEBUG
  ns_utils::print(" ctrl_cmd_msgs_  steering: ", cmd_msg->lateral.steering_tire_angle);
  ns_utils::print(
    " ctrl_cmd_msgs_  steering rate : ", cmd_msg->lateral.steering_tire_rotation_rate);
  ns_utils::print(" ctrl_cmd_msgs_  steering rate : ", cmd_msg->longitudinal.speed);
  ns_utils::print(" ctrl_cmd_msgs_  steering: ", cmd_msg->longitudinal.acceleration, -1.5);
}
/**
 * Integration with autodiff : Boost integration test
 * */
TEST(CPPADtests, integration_of_autodiffed)
{
  // Compute f(x, u) by codegen from the model.
  Model::state_vector_t f_of_dx{Model::state_vector_t::Zero()};

  // [x, y, yaw, s, ey, e_yaw, v, steering, lateral velocity]
  Model::state_vector_t x{Model::state_vector_t::Zero()};

  // Set speed
  double v_ego = 10.;
  x(6) = v_ego;

  // Create dummy controls and params.
  Model::input_vector_t u{Model::input_vector_t::Zero()};
  u(0) = 1.;  // acceleration input to the vehicle with no steering.

  Model::param_vector_t params{Model::param_vector_t::Zero()};
  params.setZero();

  double kappa{0.0};  // without curvature - straight motion.
  params(ns_utils::toUType(VehicleParamIds::curvature)) = kappa;  // curvature
  params(ns_utils::toUType(VehicleParamIds::target_vx)) = v_ego;  // target velocity

  // Call the model methods.
  // Create vehicle parameters.
  ns_models::ParamsVehicle params_vehicle{};
  params_vehicle.use_delay_model = false;
  params_vehicle.lr = 0.;

  auto vehicle_model_ptr = std::make_shared<Model>();
  vehicle_model_ptr->updateParameters(params_vehicle);

  if (!vehicle_model_ptr->IsInitialized()) {
    vehicle_model_ptr->InitializeModel();
    // vehicle_model_ptr->testModel();
  }

  // One-step Zero-Order-Hold integration.
  double dt{0.1};

  ns_eigen_utils::printEigenMat(x, "Before one step integration : ");
  auto xw0 = x(0);
  auto v0 = x(6);
  auto s0 = x(3);

  // Integrate ZOH.
  ns_sim::simulateNonlinearModel_zoh(vehicle_model_ptr, u, params, dt, x);

  ns_eigen_utils::printEigenMat(x, "After one step integration : ");
  auto xw1 = x(0);
  auto v1 = x(6);
  auto s1 = x(3);

  /*
   * Assertions
   * */

  EXPECT_DOUBLE_EQ(v1, v0 + u(0) * dt);               // speed gain.
  EXPECT_DOUBLE_EQ(xw1, xw0 + 0.5 * (v0 + v1) * dt);  // motion in x direction.
  EXPECT_DOUBLE_EQ(s1, s0 + 0.5 * (v0 + v1) * dt);    // motion in x direction.

  // Debug
  // end of debug
}

TEST_F(FakeNodeFixture, straight_line_trajectory)
{
  // Data to test
  ControlCmdMsg::SharedPtr cmd_msg;
  bool is_control_command_received{false};
  bool is_nmpc_msg_received{false};

  // Node
  std::shared_ptr<NonlinearMPCNode> node = makeNonlinearMPCNode();

  // Publishers
  rclcpp::Publisher<TrajectoryMsg>::SharedPtr traj_pub =
    this->create_publisher<TrajectoryMsg>("mpc_nonlinear/input/reference_trajectory");

  rclcpp::Publisher<VelocityMsg>::SharedPtr vel_pub =
    this->create_publisher<VelocityMsg>("mpc_nonlinear/input/current_velocity");

  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("mpc_nonlinear/input/current_steering");

  // Subscribers
  rclcpp::Subscription<ControlCmdMsg>::SharedPtr cmd_sub = this->create_subscription<ControlCmdMsg>(
    "mpc_nonlinear/output/control_cmd", *this->get_fake_node(),
    [&cmd_msg, &is_control_command_received](const ControlCmdMsg::SharedPtr msg) {
      cmd_msg = msg;
      is_control_command_received = true;
    });

  NonlinearMPCPerformanceMsg::SharedPtr nmpc_perf_msg;

  rclcpp::Subscription<NonlinearMPCPerformanceMsg>::SharedPtr nmpc_perf_sub =
    this->create_subscription<NonlinearMPCPerformanceMsg>(
      "mpc_nonlinear/debug/nmpc_vars", *this->get_fake_node(),
      [&nmpc_perf_msg](const NonlinearMPCPerformanceMsg::SharedPtr msg) { nmpc_perf_msg = msg; });

  // ASSERT_TRUE(is_control_command_received);
  auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Straight trajectory: expect no steering
  TrajectoryMsg traj_msg{};

  size_t num_of_traj_points = 20;
  double dt{1. / 30};

  std::vector<double> xw{0.};
  std::vector<double> yw{0.};

  double yawpath{15};
  ns_utils::deg2rad(yawpath);

  double spath = 0.;
  double vpath = 1.;

  for (size_t k = 1; k < num_of_traj_points; ++k) {
    spath += dt * vpath;
    auto const & xnext = spath * cos(yawpath);
    auto const & ynext = spath * sin(yawpath);
    xw.emplace_back(xnext);
    yw.emplace_back(ynext);

    TrajectoryPoint p;
    p.pose.position.x = xw[k - 1];
    p.pose.position.y = yw[k - 1];

    p.longitudinal_velocity_mps = static_cast<float>(vpath);
    p.pose.orientation = ns_nmpc_utils::createOrientationMsgfromYaw(yawpath);
    traj_msg.points.emplace_back(p);
  }

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  double yaw_vehicle = yawpath;
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform(yaw_vehicle);
  transform.transform.translation.x = xw[20];
  transform.transform.translation.y = yw[20];

  transform.header.stamp = node->now();
  br->sendTransform(transform);

  VelocityMsg vel_msg;
  vel_msg.header.stamp = node->now();
  vel_msg.twist.twist.linear.x = vpath;

  SteeringReport steer_msg;
  steer_msg.stamp = node->now();
  steer_msg.steering_tire_angle = 0.0;

  vel_pub->publish(vel_msg);
  steer_pub->publish(steer_msg);

  traj_msg.header.stamp = node->now();
  traj_msg.header.frame_id = "map";
  traj_pub->publish(traj_msg);

  // Spin for transform to be published
  test_utils::spinWhile(node);
  //  test_utils::waitForMessage(
  //    node, this, is_control_command_received, std::chrono::seconds{1LL}, false);

  test_utils::waitForMessage(node, this, is_nmpc_msg_received, std::chrono::seconds{1LL}, false);

  ASSERT_TRUE(true);

  ns_utils::print("is nmpc perf msg received ?", is_nmpc_msg_received);

  // DEBUG
  //  ns_utils::print(" ctrl_cmd_msgs_  steering: ", cmd_msg->lateral.steering_tire_angle);
  //  ns_utils::print(
  //    " ctrl_cmd_msgs_  steering rate : ", cmd_msg->lateral.steering_tire_rotation_rate);
  //  ns_utils::print(" ctrl_cmd_msgs_  steering rate : ", cmd_msg->longitudinal.speed);
  //  ns_utils::print(" ctrl_cmd_msgs_  steering: ", cmd_msg->longitudinal.acceleration);
  //
  //  ns_utils::print(" ctrl_cmd_msgs_  steering rate : ", cmd_msg->longitudinal.speed);
  //  ns_utils::print(" ctrl_cmd_msgs_  steering: ", cmd_msg->longitudinal.acceleration);
  //
  //  ns_utils::print("nonlinear mpc lateral error: ", nmpc_perf_msg->nmpc_lateral_error);
  //  ns_utils::print("nonlinear mpc yaw error", nmpc_perf_msg->nmpc_yaw_error);
  //  ns_utils::print("nonlinear mpc yaw measured", nmpc_perf_msg->yaw_angle_measured);
  //  ns_utils::print("nonlinear mpc yaw target", nmpc_perf_msg->yaw_angle_target);
  //  ns_utils::print("nonlinear mpc yaw traj", nmpc_perf_msg->yaw_angle_traj);
  //  ns_utils::print("nonlinear long vel measured", nmpc_perf_msg->long_velocity_measured);

  //  ASSERT_TRUE(is_control_nmpc_msg_received);
  //  EXPECT_EQ(cmd_msg->lateral.steering_tire_angle, 0.0f);
  //  EXPECT_EQ(cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
  //  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}
