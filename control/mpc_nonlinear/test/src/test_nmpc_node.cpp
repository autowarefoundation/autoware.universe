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
TEST_F(FakeNodeFixture, DISABLED_automaticDifferentition)
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

  auto const &xdot = v_ego * cos(beta + yaw);
  auto const &ydot = v_ego * sin(beta + yaw);
  auto const &yawdot = v_ego * tan_delta;
  auto const &sdot = v_ego * cos(beta + eyaw) / (EPS + 1 - kappa * ey);
  auto const &eydot = v_ego * sin(beta + eyaw);
  auto const &eyawdot = yawdot - kappa * sdot;
  auto const &vdot = u(0);
  auto const &deltadot = u(1);
  auto const &vydot = yawdot * vdot * cos(beta + eyaw);

  // Analytical fx.
  std::vector<double> analytical_fx_vec{xdot, ydot, yawdot, sdot, eydot,
                                        eyawdot, vdot, deltadot, vydot};

  // Autodiff fx
  vehicle_model.computeFx(x, u, params, f_of_dx);

  for (size_t k = 0; k < Model::state_dim; ++k)
  {
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

  for (Eigen::Index k = 0; k < Model::state_dim; ++k)
  {
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

/**
 * Integration with autodiff : Boost integration test
 * */
TEST(CPPADtests, DISABLED_integrationOfAutoDiff)
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

  if (!vehicle_model_ptr->IsInitialized())
  {
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

TEST_F(FakeNodeFixture, stopControlPublishedWithNoInput)
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
    [&cmd_msg, &is_control_command_received](const ControlCmdMsg::SharedPtr msg)
    {
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

TEST_F(FakeNodeFixture, emptyTrajectory)
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
    [&cmd_msg, &is_control_command_received](const ControlCmdMsg::SharedPtr msg)
    {
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
 * The trajectory initialization depends on the LPV feedback initializer class in the NMPC and it makes use of the
 * Lyapunov matrices which are used to compute state feedback coefficients. This tests checks whether the feedback
 * controllers are stable.
 *
 * */

TEST_F(FakeNodeFixture, nmpcLPVTrajInitializationStability)
{
  using ns_utils::toUType;

  // Node
  std::shared_ptr<NonlinearMPCNode> node = makeNonlinearMPCNode();

  auto nmpc_core = node->getNMPCcore();
  bool is_core_initialized = nmpc_core.isInitialized();

  ns_utils::print("is nmpc core initialized ", is_core_initialized);
  auto params_lpv = nmpc_core.getLPVparams();

  /**
   * Read the Lyapunov matrices.
   * */
  auto const &num_of_params = params_lpv.num_of_nonlinearities;
  ns_utils::print("num of parameters in LPV container", num_of_params);

  for (size_t k = 0; k < num_of_params; ++k)
  {
    auto Xl = params_lpv.lpvXcontainer[k];
    auto Yl = params_lpv.lpvYcontainer[k];
    ns_eigen_utils::printEigenMat(Xl, "Xl " + std::to_string(k));
    ns_eigen_utils::printEigenMat(Yl, "Yl " + std::to_string(k));
  }

  /**
   * Create a vehicle model
   * */

  // Create vehicle parameters.
  ns_models::ParamsVehicle params_vehicle{};
  params_vehicle.lr = 0.;  // rear-axle center reference

  auto vehicle_model_ptr = std::make_shared<Model>();
  vehicle_model_ptr->updateParameters(params_vehicle);

  if (!vehicle_model_ptr->IsInitialized())
  {
    vehicle_model_ptr->InitializeModel();
    // vehicle_model_ptr->testModel();
  }

  /**
   * Check the stability of A + BK for a given grid points of the parameters.
   * */
  auto kappa_grid = ns_utils::linspace(-0.08, 0.08, 3);  // curvature

  double eyaw_max = ns_utils::deg2rad(20.);

  auto eyaw_grid = ns_utils::linspace(-eyaw_max, eyaw_max, 5);
  auto ey_grid = ns_utils::linspace(-0.8, 0.8, 5);

  // Initialize states and inputs.
  Model::state_vector_t x{Model::state_vector_t::Zero()};
  Model::input_vector_t u{Model::input_vector_t::Zero()};
  Model::param_vector_t params{Model::param_vector_t::Zero()};

  // Define placeholders for the system matrices.
  Model::state_matrix_t Ac{Model::state_matrix_t::Zero()};
  Model::control_matrix_t Bc{Model::control_matrix_t::Zero()};

  // Define Lyapunov matrices placeholders.
  Model::state_matrix_X_t Xr{Model::state_matrix_X_t::Zero()};
  Model::input_matrix_Y_t Yr{Model::input_matrix_Y_t::Zero()};

  // starting speed
  // Although, it is stable for a range of velocity, we use the LPV when starting only.
  auto const &ntheta_ = params_lpv.num_of_nonlinearities;

  double vx{2.};
  auto const &Id = Eigen::MatrixXd::Identity(4, 4);  // Model::state_matrix_t::Identity();
  double dt_mpc = 0.1;

  for (double const &ey : ey_grid)
  {
    for (double const &eyaw : eyaw_grid)
    {
      for (double const &k : kappa_grid)
      {
        // for computing the state and control matrices, set the states
        x.setZero();
        u.setZero();

        x(toUType(VehicleStateIds::vx)) = vx;
        x(toUType(VehicleStateIds::ey)) = ey;
        x(toUType(VehicleStateIds::eyaw)) = eyaw;

        u.setRandom();
        params(toUType(VehicleParamIds::curvature)) = k;

        Ac.setZero();
        Bc.setZero();

        vehicle_model_ptr->computeJacobians(x, u, params, Ac, Bc);

        // ns_eigen_utils::printEigenMat(Ac, "Ac");
        // ns_eigen_utils::printEigenMat(Ac.block<4, 4>(4, 4), "Ace");

        // ns_eigen_utils::printEigenMat(Bc, "Bc");
        // ns_eigen_utils::printEigenMat(Bc.block<4, 2>(4, 0), "Bce");

        auto const &Ac_error_block = Ac.block<4, 4>(4, 4);

        auto const &th1 = Ac_error_block(0, 1);
        auto const &th2 = Ac_error_block(0, 2);

        auto const &th3 = Ac_error_block(1, 0);
        auto const &th4 = Ac_error_block(1, 1);
        auto const &th5 = Ac_error_block(1, 2);
        auto const &th6 = Ac_error_block(1, 3);

        // ns_utils::print("Nonlinearities in Error Block", th1, th2, th3, th4);

        auto thetas_ = std::vector<double>{th1, th2, th3, th4, th5, th6};

        // Extract the first X0, Y0, we save the first X0 and Y0 at the end.
        Xr = params_lpv.lpvXcontainer.back();  // We keep the first X0, Y0 at the end of the
        Yr = params_lpv.lpvYcontainer.back();

        for (size_t j = 0; j < ntheta_ - 1; j++)
        {
          Xr += thetas_[j] * params_lpv.lpvXcontainer[j];
          Yr += thetas_[j] * params_lpv.lpvYcontainer[j];
        }

        // Compute Feedback coefficients.
        auto const &Pr = Xr.inverse();  // Cost matrix P.
        auto const &Kfb = Yr * Pr;      // State feedback coefficients matrix.

        // ns_eigen_utils::printEigenMat(Kfb.eval(), "\nComputed Feedback Gains");

        /**
         * Discretisize the system matrices
         * */
        auto const &I_At2_inv = (Id - Ac.block<4, 4>(4, 4) * dt_mpc / 2).inverse();

        auto const &Ad = I_At2_inv * (Id + Ac.block<4, 4>(4, 4) * dt_mpc / 2);
        auto const &Bd = I_At2_inv * Bc.block<4, 2>(4, 0) * dt_mpc;

        // Closed loop transfer matrix
        // ['xw', 'yw', 'psi', 's', 'e_y', 'e_yaw', 'Vx', 'delta', 'ay']
        auto Aclosed_loop = Ad + Bd * Kfb;
        auto const &eig_vals = Aclosed_loop.eigenvalues();

        ns_utils::print("Operating states, vx, ey, eyaw :", vx, ey, eyaw);
        // ns_eigen_utils::printEigenMat(eig_vals, "\nEigen values of the closed loop system matrix :");

        // ns_utils::print("Magnitude of Eigenvalues ");
        // for (auto ke = 0; ke < eig_vals.size(); ++ke)
        // {
        //  ASSERT_LE(std::abs(eig_vals(ke)), 1.);
        //  ns_utils::print(std::abs(eig_vals(ke)));
        // }
      }
    }
  }

  ASSERT_TRUE(true);
}

/**
 * A straight trajectory path is give to the NMPC and it is expected to produce zero-steering angle control command.
 * */

TEST_F(FakeNodeFixture, straightTrajectoryTest)
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
    [&cmd_msg, &is_control_command_received](const ControlCmdMsg::SharedPtr msg)
    {
      cmd_msg = msg;
      is_control_command_received = true;
    });

  NonlinearMPCPerformanceMsg::SharedPtr nmpcperf_msg;

  rclcpp::Subscription<NonlinearMPCPerformanceMsg>::SharedPtr nmpc_perfsub =
    this->create_subscription<NonlinearMPCPerformanceMsg>(
      "mpc_nonlinear/debug/nmpc_vars", *this->get_fake_node(),
      [&nmpcperf_msg, &is_nmpc_msg_received](const NonlinearMPCPerformanceMsg::SharedPtr msg)
      {
        nmpcperf_msg = msg;
        is_nmpc_msg_received = true;
      });

  // ASSERT_TRUE(is_control_command_received);
  auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Straight trajectory: expect no steering
  TrajectoryMsg traj_msg{};

  size_t num_of_traj_points = 50;
  double dt{1. / 10};

  std::vector<double> xw{0.};
  std::vector<double> yw{0.};

  double yawpath = ns_utils::deg2rad(15.);

  double spath = 0.;
  double vpath = 10.;

  for (size_t k = 1; k < num_of_traj_points; ++k)
  {
    spath += dt * vpath;
    auto const &xnext = spath * cos(yawpath);
    auto const &ynext = spath * sin(yawpath);
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
  transform.transform.translation.x = xw[0];
  transform.transform.translation.y = yw[0];

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
  test_utils::waitForMessage(
    node, this, is_control_command_received, std::chrono::seconds{1LL}, false);

  test_utils::waitForMessage(node, this, is_nmpc_msg_received, std::chrono::seconds{1LL}, false);
  test_utils::spinWhile(node);

  // DEBUG
  ns_utils::print("is nmpc perf msg received ?", is_nmpc_msg_received);
  ns_utils::print("ctrl_cmd_msgs_ steering: ", cmd_msg->lateral.steering_tire_angle);
  ns_utils::print("ctrl_cmd_msgs_ steering rate : ", cmd_msg->lateral.steering_tire_rotation_rate);

  ns_utils::print("ctrl_cmd_msgs_ longitudinal speed: ", cmd_msg->longitudinal.speed);
  ns_utils::print("ctrl_cmd_msgs_ longitudinal acceleration: ", cmd_msg->longitudinal.acceleration);

  ns_utils::print("nonlinear mpc lateral error: ", nmpcperf_msg->nmpc_lateral_error);
  ns_utils::print("nonlinear mpc yaw error", nmpcperf_msg->nmpc_yaw_error);
  ns_utils::print("nonlinear mpc yaw measured", nmpcperf_msg->yaw_angle_measured);
  ns_utils::print("nonlinear mpc yaw target", nmpcperf_msg->yaw_angle_target);
  ns_utils::print("nonlinear mpc yaw traj", nmpcperf_msg->yaw_angle_traj);
  ns_utils::print("nonlinear long vel target", nmpcperf_msg->long_velocity_target);

  ASSERT_TRUE(is_control_command_received);
  ASSERT_TRUE(is_nmpc_msg_received);

  EXPECT_LE(std::fabs(cmd_msg->lateral.steering_tire_angle), 1e-6f);
  EXPECT_GE(cmd_msg->longitudinal.acceleration,
            static_cast<decltype(cmd_msg->longitudinal.acceleration)>(0));

  EXPECT_LE(std::fabs(nmpcperf_msg->nmpc_lateral_error), 1e-4);
  EXPECT_LE(std::fabs(nmpcperf_msg->nmpc_yaw_error), 1e-4);
  // EXPECT_DOUBLE_EQ(nmpcperf_msg->long_velocity_target, vpath);

  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}


/**
 * The NMPC controller should produce a steering signal that move the vehicle to the right.
 * */
TEST_F(FakeNodeFixture, turnRight)
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
    [&cmd_msg, &is_control_command_received](const ControlCmdMsg::SharedPtr msg)
    {
      cmd_msg = msg;
      is_control_command_received = true;
    });

  NonlinearMPCPerformanceMsg::SharedPtr nmpcperf_msg;

  rclcpp::Subscription<NonlinearMPCPerformanceMsg>::SharedPtr nmpc_perfsub =
    this->create_subscription<NonlinearMPCPerformanceMsg>(
      "mpc_nonlinear/debug/nmpc_vars", *this->get_fake_node(),
      [&nmpcperf_msg, &is_nmpc_msg_received](const NonlinearMPCPerformanceMsg::SharedPtr msg)
      {
        nmpcperf_msg = msg;
        is_nmpc_msg_received = true;
      });

  // ASSERT_TRUE(is_control_command_received);
  auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Straight trajectory: expect no steering
  TrajectoryMsg traj_msg{};

  size_t num_of_traj_points = 50;
  double dt{1. / 10};

  std::vector<double> xw{0.};
  std::vector<double> yw{0.};

  double yawpath = ns_utils::deg2rad(15.);
  double yawerror = ns_utils::deg2rad(1.);
  double ey = 0.2; // lateral error of the vehicle w.r.t path

  double spath = 0.;
  double vpath = 10.;

  for (size_t k = 1; k < num_of_traj_points; ++k)
  {
    spath += dt * vpath;
    auto const &xnext = spath * cos(yawpath);
    auto const &ynext = spath * sin(yawpath);
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
  double yaw_vehicle = yawpath + yawerror;
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform(yaw_vehicle);
  transform.transform.translation.x = xw[0];
  transform.transform.translation.y = yw[0] + ey;

  transform.header.stamp = node->now();
  br->sendTransform(transform);

  VelocityMsg vel_msg;
  vel_msg.header.stamp = node->now();
  vel_msg.twist.twist.linear.x = vpath + 2.;

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
  test_utils::waitForMessage(
    node, this, is_control_command_received, std::chrono::seconds{1LL}, false);

  test_utils::waitForMessage(node, this, false, std::chrono::seconds{1LL}, false);
  test_utils::spinWhile(node);

  // DEBUG
  ns_utils::print("is nmpc perf msg received ?", is_nmpc_msg_received);
  ns_utils::print("ctrl_cmd_msgs_ steering: ", cmd_msg->lateral.steering_tire_angle);
  ns_utils::print("ctrl_cmd_msgs_ steering rate : ", cmd_msg->lateral.steering_tire_rotation_rate);

  ns_utils::print("ctrl_cmd_msgs_ longitudinal speed: ", cmd_msg->longitudinal.speed);
  ns_utils::print("ctrl_cmd_msgs_ longitudinal acceleration: ", cmd_msg->longitudinal.acceleration);

  ns_utils::print("nonlinear mpc lateral error: ", nmpcperf_msg->nmpc_lateral_error);
  ns_utils::print("nonlinear mpc yaw error", nmpcperf_msg->nmpc_yaw_error);
  ns_utils::print("nonlinear mpc yaw measured", nmpcperf_msg->yaw_angle_measured);
  ns_utils::print("nonlinear mpc yaw target", nmpcperf_msg->yaw_angle_target);
  ns_utils::print("nonlinear mpc yaw traj", nmpcperf_msg->yaw_angle_traj);
  ns_utils::print("nonlinear long vel target", nmpcperf_msg->long_velocity_target);

  ASSERT_TRUE(is_control_command_received);
  ASSERT_TRUE(is_nmpc_msg_received);


  // EXPECT_DOUBLE_EQ(nmpcperf_msg->long_velocity_target, vpath);

  EXPECT_LE(cmd_msg->lateral.steering_tire_angle,
            static_cast<decltype(cmd_msg->lateral.steering_tire_angle)>(0));

  EXPECT_LE(cmd_msg->longitudinal.acceleration,
            static_cast<decltype(cmd_msg->lateral.steering_tire_angle)>(0));

  EXPECT_DOUBLE_EQ(nmpcperf_msg->long_velocity_target, vpath);

  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));

}

TEST_F(FakeNodeFixture, turnLeft)
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
    [&cmd_msg, &is_control_command_received](const ControlCmdMsg::SharedPtr msg)
    {
      cmd_msg = msg;
      is_control_command_received = true;
    });

  NonlinearMPCPerformanceMsg::SharedPtr nmpcperf_msg;

  rclcpp::Subscription<NonlinearMPCPerformanceMsg>::SharedPtr nmpc_perfsub =
    this->create_subscription<NonlinearMPCPerformanceMsg>(
      "mpc_nonlinear/debug/nmpc_vars", *this->get_fake_node(),
      [&nmpcperf_msg, &is_nmpc_msg_received](const NonlinearMPCPerformanceMsg::SharedPtr msg)
      {
        nmpcperf_msg = msg;
        is_nmpc_msg_received = true;
      });

  // ASSERT_TRUE(is_control_command_received);
  auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Straight trajectory: expect no steering
  TrajectoryMsg traj_msg{};

  size_t const num_of_traj_points = 50;
  double const dt{1. / 10};

  std::vector<double> xw{0.};
  std::vector<double> yw{0.};

  double yawpath = ns_utils::deg2rad(15.);
  double yawerror = ns_utils::deg2rad(-1.);
  double ey = 0.2; // lateral error of the vehicle w.r.t path

  double spath = 0.;
  double vpath = 10.;

  for (size_t k = 1; k < num_of_traj_points; ++k)
  {
    spath += dt * vpath;
    auto const &xnext = spath * cos(yawpath);
    auto const &ynext = spath * sin(yawpath);
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
  double yaw_vehicle = yawpath + yawerror;
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform(yaw_vehicle);
  transform.transform.translation.x = xw[1];
  transform.transform.translation.y = yw[0] - ey;

  transform.header.stamp = node->now();
  br->sendTransform(transform);

  VelocityMsg vel_msg;
  vel_msg.header.stamp = node->now();
  vel_msg.twist.twist.linear.x = vpath - 2.;

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
  test_utils::waitForMessage(
    node, this, is_control_command_received, std::chrono::seconds{1LL}, false);

  test_utils::waitForMessage(node, this, false, std::chrono::seconds{1LL}, false);
  test_utils::spinWhile(node);

  // DEBUG
  ns_utils::print("is nmpc perf msg received ?", is_nmpc_msg_received);
  ns_utils::print("ctrl_cmd_msgs_ steering: ", cmd_msg->lateral.steering_tire_angle);
  ns_utils::print("ctrl_cmd_msgs_ steering rate : ", cmd_msg->lateral.steering_tire_rotation_rate);

  ns_utils::print("ctrl_cmd_msgs_ longitudinal speed: ", cmd_msg->longitudinal.speed);
  ns_utils::print("ctrl_cmd_msgs_ longitudinal acceleration: ", cmd_msg->longitudinal.acceleration);

  ns_utils::print("nonlinear mpc lateral error: ", nmpcperf_msg->nmpc_lateral_error);
  ns_utils::print("nonlinear mpc yaw error", nmpcperf_msg->nmpc_yaw_error);
  ns_utils::print("nonlinear mpc yaw measured", nmpcperf_msg->yaw_angle_measured);
  ns_utils::print("nonlinear mpc yaw target", nmpcperf_msg->yaw_angle_target);
  ns_utils::print("nonlinear mpc yaw traj", nmpcperf_msg->yaw_angle_traj);
  ns_utils::print("nonlinear long vel target", nmpcperf_msg->long_velocity_target);

  ASSERT_TRUE(is_control_command_received);
  ASSERT_TRUE(is_nmpc_msg_received);


  // EXPECT_DOUBLE_EQ(nmpcperf_msg->long_velocity_target, vpath);

  EXPECT_GE(cmd_msg->lateral.steering_tire_angle,
            static_cast<decltype(cmd_msg->lateral.steering_tire_angle)>(0));

  EXPECT_GE(cmd_msg->longitudinal.acceleration,
            static_cast<decltype(cmd_msg->lateral.steering_tire_angle)>(0));

  EXPECT_DOUBLE_EQ(nmpcperf_msg->long_velocity_target, vpath);

  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));

}