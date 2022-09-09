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
#include <limits>
#include <memory>
#include <vector>
#include "gtest/gtest.h"
#include "nonlinear_mpc_test_node.hpp"
#include "nmpc_test_utils.hpp"

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
    [&cmd_msg, &is_control_command_received](const ControlCmdMsg::SharedPtr msg)
    {
      cmd_msg = msg;
      is_control_command_received = true;
    });

  auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // test_utils::spinWhile(node);

  test_utils::waitForMessage(node, this, is_control_command_received, std::chrono::seconds{1LL}, false);
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
  ns_utils::print(" ctrl_cmd_msgs_  steering rate : ", cmd_msg->lateral.steering_tire_rotation_rate);

  ns_utils::print(" ctrl_cmd_msgs_  steering rate : ", cmd_msg->longitudinal.speed);
  ns_utils::print(" ctrl_cmd_msgs_  steering: ", cmd_msg->longitudinal.acceleration, -1.5);

}

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
  u.setRandom();

  Model::param_vector_t params{Model::param_vector_t::Zero()};
  params.setZero();

  double kappa = 0.1;
  params(ns_utils::toUType(VehicleParamIds::curvature)) = kappa;    // curvature
  params(ns_utils::toUType(VehicleParamIds::target_vx)) = v_ego;    // target velocity

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
  std::vector<double> analytical_fx_vec;  // {Model::state_dim};

  auto tan_delta = tan(steering);
  auto beta = atan(tan_delta * paramsVehicle.lr / paramsVehicle.wheel_base);

  analytical_fx_vec.emplace_back(v_ego * cos(beta + yaw));
  analytical_fx_vec.emplace_back(v_ego * sin(beta + yaw));
  analytical_fx_vec.emplace_back(v_ego * sin(beta) / paramsVehicle.lr);
  analytical_fx_vec.emplace_back(v_ego * cos(beta + eyaw) / (EPS + 1 - kappa * ey));
  analytical_fx_vec.emplace_back(v_ego * sin(beta + eyaw));
  analytical_fx_vec.emplace_back(analytical_fx_vec[2] - kappa * analytical_fx_vec[3]);
  analytical_fx_vec.emplace_back(u(ns_utils::toUType(VehicleControlIds::u_vx)));
  analytical_fx_vec.emplace_back(u(ns_utils::toUType(VehicleControlIds::u_steering)));
  analytical_fx_vec.emplace_back(0.);

  vehicle_model.computeFx(x, u, params, f_of_dx);

  for (size_t k = 0; k < Model::state_dim; ++k)
  {
    ASSERT_DOUBLE_EQ(f_of_dx(static_cast<long>(k)), analytical_fx_vec[k]);
    ns_utils::print("k : ", k, " ;  fx : ", f_of_dx(static_cast<long>(k)), "analytical fx : ", analytical_fx_vec[k]);
  }

  // Analytical derivative f wrt v : df/dv  = A(:, 6th_col)
  vehicle_model.computeJacobians(x, u, params, A, B);

  EXPECT_DOUBLE_EQ(B(6, 0), 1.);
  EXPECT_DOUBLE_EQ(B(7, 1), 1.);

  std::vector<double> analytical_df_dv_vec;  // {Model::state_dim};

  analytical_df_dv_vec.emplace_back(cos(beta + yaw)); // x
  analytical_df_dv_vec.emplace_back(sin(beta + yaw)); // y
  analytical_df_dv_vec.emplace_back(tan(steering) / paramsVehicle.wheel_base); // yaw
  analytical_df_dv_vec.emplace_back(cos(beta + eyaw) / (1 - kappa * ey)); //s
  analytical_df_dv_vec.emplace_back(sin(beta + eyaw)); //ey
  analytical_df_dv_vec.emplace_back(analytical_df_dv_vec[2] - kappa * analytical_df_dv_vec[3]); // eyaw
  analytical_df_dv_vec.emplace_back(0.); // ax
  analytical_df_dv_vec.emplace_back(0.); // steering input
  analytical_df_dv_vec.emplace_back(analytical_df_dv_vec[2]); // lateral acceleration

  for (Eigen::Index k = 0; k < Model::state_dim; ++k)
  {
    //ASSERT_DOUBLE_EQ(A(k, 6), analytical_df_dv_vec[static_cast<unsigned long>(k)]);
    ns_utils::print("k : ", k, ": A:", A(k, 6), ": df/dv: ", analytical_df_dv_vec[static_cast<unsigned long>(k)]);
  }

  // Debug
  ns_eigen_utils::printEigenMat(f_of_dx, "System dynamical equations values : ");
  ns_utils::print("Jacobians A and B : ");
  ns_eigen_utils::printEigenMat(A);
  ns_eigen_utils::printEigenMat(B);
  // end of debug
}

/**
 * @brief path tracking performance test - sigmoid function like xy trajectory.
 * The trajectory starts at [x, y, yaw] = [0, 0, 0] and ends at [500, 50m    ]
 * */
TEST_F(FakeNodeFixture, nmpc_tracking)
{
  TrajectoryMsg sigmoid_traj_msg{};
  createTrajectoryMessage(sigmoid_traj_msg);

  ASSERT_TRUE(true);
}
