// Copyright 2022 The Autoware Foundation
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

#include "cdc_test_node.hpp"
#include "cdc_test_utils.hpp"
#include "gtest/gtest.h"

TEST_F(FakeNodeFixture, doesPublishCorrectiveRefs)
{
  // Data to test
  DelayCompensationRefs::SharedPtr compensation_msg{nullptr};
  bool is_comp_msg_received = false;

  // Node
  std::shared_ptr<CommDelayNode> node = makeComDelayComNode();

  // Publishers
  rclcpp::Publisher<VelocityMsg>::SharedPtr vel_pub =
    this->create_publisher<VelocityMsg>("communication_delay_compensator/input/current_odometry");

  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("communication_delay_compensator/input/steering_state");

  //  rclcpp::Publisher<ErrorReportMsg>::SharedPtr long_error_pub_ =
  //    this->create_publisher<ErrorReportMsg>("communication_delay_compensator/input/long_errors");

  rclcpp::Publisher<ErrorReportMsg>::SharedPtr lat_error_pub_ =
    this->create_publisher<ErrorReportMsg>("communication_delay_compensator/input/lat_errors");

  rclcpp::Publisher<ControlCmdMsg>::SharedPtr control_pub_ =
    this->create_publisher<ControlCmdMsg>("communication_delay_compensator/input/control_cmd");

  // Subscribers
  rclcpp::Subscription<DelayCompensationRefs>::SharedPtr comm_ref_sub_ =
    this->create_subscription<DelayCompensationRefs>(
      "communication_delay_compensator/output/communication_delay_compensation_refs",
      *this->get_fake_node(),
      [&compensation_msg, &is_comp_msg_received](const DelayCompensationRefs::SharedPtr msg) {
        compensation_msg = msg;
        is_comp_msg_received = true;
      });

  // Broadcast transform
  auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  /**
   * Publish dummy msgs
   * */

  VelocityMsg odom_msg;
  SteeringReport steer_msg;
  ControlCmdMsg control_msg;
  ErrorReportMsg error_msg;

  odom_msg.header.stamp = node->now();
  odom_msg.twist.twist.linear.x = 0.0;

  steer_msg.steering_tire_angle = 0.0;
  steer_msg.stamp = node->now();

  control_msg.lateral.steering_tire_angle = 0.0;
  control_msg.stamp = node->now();

  error_msg.lateral_deviation_read = 0.0;
  error_msg.steering_read = 0.0;
  error_msg.heading_angle_error_read = 0.0;
  error_msg.curvature_read = 0.0;
  error_msg.stamp = node->now();

  // Broadcast transform
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();

  transform.header.stamp = node->now();
  br->sendTransform(transform);

  vel_pub->publish(odom_msg);
  steer_pub->publish(steer_msg);
  control_pub_->publish(control_msg);
  lat_error_pub_->publish(error_msg);

  test_utils::spinWhile(node);
  test_utils::waitForMessage(node, this, is_comp_msg_received, std::chrono::seconds{2LL}, false);
  test_utils::spinWhile(node);
  //  auto time_passed{std::chrono::milliseconds{0LL}};
  //  const auto dt{std::chrono::milliseconds{30LL}};
  //
  //  while (time_passed < std::chrono::seconds{2LL})
  //  {
  //    rclcpp::spin_some(node);
  //    rclcpp::spin_some(this->get_fake_node());
  //    std::this_thread::sleep_for(dt);
  //    time_passed += dt;
  //  }

  ns_utils::print("is compensation_msg received ? ", is_comp_msg_received);

  ASSERT_TRUE(is_comp_msg_received);
}

TEST_F(FakeNodeFixture, isLPVobserverStable)
{
  using ns_utils::toUType;

  // Node
  std::shared_ptr<CommDelayNode> node = makeComDelayComNode();
  auto dob_observer_model_ptr = node->getDOBmodel();

  auto const & lyap_mats = node->getLyapMatrices();
  auto const & num_of_mats = lyap_mats.vXs.size();

  for (size_t k = 0; k < num_of_mats; ++k) {
    auto Xl = lyap_mats.vXs[k];
    auto Yl = lyap_mats.vYs[k];
    ns_eigen_utils::printEigenMat(Xl, "Xl " + std::to_string(k));
    ns_eigen_utils::printEigenMat(Yl, "Yl " + std::to_string(k));
  }

  // Generate grids for ey, eyaw and steering angle.
  auto const & ey_grid = ns_utils::linspace(-0.8, 0.8, 5);

  double const eyaw_max = ns_utils::deg2rad(30.);
  auto const & eyaw_grid = ns_utils::linspace(-eyaw_max, eyaw_max, 5);

  auto const & kappa_grid = ns_utils::linspace(-0.08, 0.08, 3);  // curvature
  double const steering{};

  double const vmax{20.};                          // m/s
  auto vx_grid = ns_utils::linspace(1., vmax, 4);  // curvature

  for (double const & ey : ey_grid) {
    for (double const & eyaw : eyaw_grid) {
      for (double const & k : kappa_grid) {
        for (double const & vx : vx_grid) {
          observers::state_vector_vehicle_t y_current_measurements;
          y_current_measurements << ey, eyaw, steering;

          // Update the observer model.
          dob_observer_model_ptr->updateStateSpace(vx, steering);

          // Update the initial states
          dob_observer_model_ptr->updateInitialStates(ey, eyaw, steering, vx, k);

          // Evaluate the LPV parameters.
          observers::state_vector_observer_t thetas{observers::state_vector_observer_t::Zero()};
          dob_observer_model_ptr->evaluateNonlinearTermsForLyap(thetas, y_current_measurements);

          ns_eigen_utils::printEigenMat(thetas, "Nonlinear terms of the LPV model : ");

          // Compute the observer gains.
          // Compute the parametric lyapunov matrices.
          auto Xc = lyap_mats.vXs.back();  // X0, Y0 are stored at the end.
          auto Yc = lyap_mats.vYs.back();

          observers::measurement_matrix_observer_t Lobs_;  // observer gain matrix

          // P(th) = P0 + th1*P1 + ...
          for (size_t j = 0; j < lyap_mats.vXs.size() - 1; ++j) {
            Xc += thetas(static_cast<Eigen::Index>(j)) * lyap_mats.vXs[j];
            Yc += thetas(static_cast<Eigen::Index>(j)) * lyap_mats.vYs[j];
          }

          // Compute the observer  gain matrix.
          Lobs_ = Yc * Xc.inverse();
          ns_eigen_utils::printEigenMat(Lobs_, "Observer gain matrix : ");

          // Get Ad and Cd to compute a closed loop system matrix.
          auto Ad = dob_observer_model_ptr->Ad();
          auto Cd = dob_observer_model_ptr->Cd();

          ns_eigen_utils::printEigenMat(Ad, "Ad matrix : ");
          ns_eigen_utils::printEigenMat(Cd, "Cd matrix : ");

          // Compute the closed loop system matrix
          auto Aclp = Ad + Lobs_.transpose() * Cd;

          // Get the eigenvalues
          auto const & eig_vals = Aclp.eigenvalues();
          ns_eigen_utils::printEigenMat(
            eig_vals, "\nEigen values of the closed loop system matrix :");

          ns_utils::print("Magnitude of Eigenvalues ");
          for (auto ke = 0; ke < eig_vals.size(); ++ke) {
            ASSERT_LE(std::abs(eig_vals(ke)), 1.);
            ns_utils::print(std::abs(eig_vals(ke)));
          }
        }
      }
    }
  }
}