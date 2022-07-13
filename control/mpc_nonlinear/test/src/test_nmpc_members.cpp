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
#include "test_nonlinear_mpc_node.hpp"

// Util functions testing.
TEST_F(NMPCTestSuiteMembers, angleDistance)
{
  double reference_angle = -0.1;
  double target_angle = 0.1;

  double difference_beetween_angles = ns_utils::angleDistance(target_angle, reference_angle);
  EXPECT_DOUBLE_EQ(reference_angle + difference_beetween_angles, target_angle);

  ns_utils::print("Angle diff : ", difference_beetween_angles);
}

// Unit tests of the auxiliary libraries : SPLINES_PCG.
/**
 *  y = a*sin(b*t)  spline fitting test.
 * */
TEST_F(NMPCTestSuiteMembers, splinePCG)
{
  size_t const base_size = 50;  // the base coordinate and data size.
  size_t const new_size = 50;   // original signal is expanded to this size.

  // Prepare the base signal.
  double a = 10.0;
  double b = 8.0;

  std::vector<double> t_base = ns_utils::linspace<double>(0.0, b, base_size);

  std::vector<double> y_base;
  std::transform(
    t_base.cbegin(), t_base.cend(), std::back_inserter(y_base), [&](auto const & x) {
      return a * sin(x);
    });

  // Generate spline interpolator.
  auto interpolating_spline_pcg = ns_splines::InterpolatingSplinePCG(3);

  // Test time vector in the new_size.
  auto t_new = ns_utils::linspace(t_base[0], t_base.back(), new_size);
  std::vector<double> y_new;

  // Initialize with the coordinates and the base data.
  auto is_updated = interpolating_spline_pcg.Initialize(t_base, y_base);
  ASSERT_TRUE(is_updated);

  bool could_interpolate = interpolating_spline_pcg.Interpolate(t_new, y_new);

  // Compare with the analytical function.
  for (size_t k = 0; k < y_new.size(); ++k) {
    ASSERT_LE(std::abs(y_new[k] - a * std::sin(t_new[k])), 1e-5);
  }

  // Test scalar interpolation.
  double y_scalar;
  double t_scalar = t_new[2];
  could_interpolate = interpolating_spline_pcg.Interpolate(t_scalar, y_scalar);
  EXPECT_DOUBLE_EQ(y_scalar, y_new[2]);

  // Test out of range.
  auto tnew_vec_out_of_range = ns_utils::linspace<double>(-0.5, b + 1.0, base_size);
  could_interpolate =
    interpolating_spline_pcg.Interpolate(t_base, y_base, tnew_vec_out_of_range, y_new);

  ASSERT_FALSE(could_interpolate);

  // test constructor and interpolation (without storing tbase, ybase).
  could_interpolate = interpolating_spline_pcg.Interpolate(t_base, y_base, t_new, y_new);
  ASSERT_TRUE(could_interpolate);
}

// Test Bspline interpolator, yaw and curvature computation test.
/**
 *  Note that, this is also a smoother.
 *  x = R*sin(theta)          Curvature test k ~= 1 / R
 *  y = R*cos(theta)
 *
 *  Analytical values to compare.
 *  yaw = theta, and curvature ~= 1
 * */
TEST_F(NMPCTestSuiteMembers, splineBSpline)
{
  double Radius = 20.;          // circle radius
  size_t const base_size = 50;  // the base coordinate and data size.
  size_t const new_size = 50;   // original signal is expanded to this size.

  // Prepare the base signal.
  std::vector<double> theta_base = ns_utils::linspace<double>(0.0, 2 * M_PI, base_size);

  // EIGEN IMPLEMENTATION.
  Eigen::MatrixXd th_base_eigen =
    Eigen::Map<Eigen::Matrix<double, base_size, 1>>(theta_base.data());

  Eigen::MatrixXd x_base_eigen(
    th_base_eigen.unaryExpr([&Radius](auto const & t) {return Radius * std::cos(t);}));
  Eigen::MatrixXd y_base_eigen(
    th_base_eigen.unaryExpr([&Radius](auto const & t) {return Radius * std::sin(t);}));

  // Concatenate xy into a matrix.
  auto base_matrix_xy = ns_eigen_utils::hstack<double>(x_base_eigen, y_base_eigen);

  // Create a new BSpline object (true for derivative computations).
  ns_splines::BSplineInterpolatorTemplated<base_size, new_size> bspline_interpolator(0.95, true);

  Eigen::MatrixXd interpolated_xy_mat;   // [x, y]
  Eigen::MatrixXd xy_dot(new_size, 2);   // [x', y']
  Eigen::MatrixXd xy_ddot(new_size, 2);  // [x'', y'']

  // Interpolate xy matrix into the interpolated_xy matrix.
  bspline_interpolator.InterpolateImplicitCoordinates(base_matrix_xy, interpolated_xy_mat);
  bspline_interpolator.getFirstDerivative(base_matrix_xy, xy_dot);
  bspline_interpolator.getSecondDerivative(base_matrix_xy, xy_ddot);

  // Curvature computation given the derivatives.
  auto curvature = ns_eigen_utils::Curvature(xy_dot, xy_ddot);

  EXPECT_LE(std::fabs(curvature.mean() - 1. / Radius), 1e-3);

  // Print
  ns_utils::print("Mean Curvature ", curvature.mean());
  // auto yaw_matrix = ns_eigen_utils::hstack<double>(yaw_angles, th_base_eigen);

  //    ns_utils::print("base mat xy ");
  //    ns_eigen_utils::printEigenMat(base_matrix_xy);
  //
  //    ns_utils::print("interpolated mat xy ");
  //    ns_eigen_utils::printEigenMat(interpolated_xy_mat);
  //
  //    ns_utils::print("interpolated mat xy_dot ");
  //    ns_eigen_utils::printEigenMat(xy_dot);

  //    ns_utils::print("Yaw angles original ");
  //    ns_eigen_utils::printEigenMat(yaw_matrix);
  //
  //    ns_utils::print("Yaw error");
  //    ns_eigen_utils::printEigenMat(yaw_angles - th_base_eigen);

  //    ns_utils::print("interpolated mat xy_ddot ");
  //    ns_eigen_utils::printEigenMat(xy_ddot);
  //
  //    ns_utils::print("curvature ");
  //    ns_eigen_utils::printEigenMat(curvature);

  // Eigen::MatrixXd y_to_be_interpolated ;
}

/**
 * Automatic differentiation : Vehicle model equations test.
 * */
TEST_F(NMPCTestSuiteMembers, automaticDifferentiation)
{
  auto constexpr EPS = std::numeric_limits<double>::epsilon();

  // Compute f(x, u) by codegen from the model.
  Model::state_vector_t f_of_dx;
  f_of_dx.setZero();

  Model::state_vector_t x;  // [x, y, yaw, s, ey, e_yaw, v, steering, virtual_car_dist]
  x.setZero();

  // Set speed
  double vtarget = 15.;
  double v_ego = 10.;

  x(6) = v_ego;

  // double xw = x(0);  // x-coordinate
  // double yw = x(1);  // y-coordinate
  double yaw = x(2);
  // double sdist = x(3);
  double ey = x(4);
  double eyaw = x(5);
  double steering = x(7);
  double vdist = v_ego - vtarget;

  // Create dummy controls and params.
  Model::input_vector_t u;
  u.setRandom();

  Model::param_vector_t params;
  params.setZero();

  double kappa = 0.1;
  params(0) = kappa;    // curvature
  params(1) = vtarget;  // virtual car speed.

  // Compute Jacobian --
  Model::state_matrix_t A;
  A.setZero();

  Model::control_matrix_t B;
  B.setZero();

  // Call the model methods.
  // Create vehicle parameters.
  ns_models::ParamsVehicle paramsVehicle{};
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
  analytical_fx_vec.emplace_back(u(0));
  analytical_fx_vec.emplace_back(u(1));
  analytical_fx_vec.emplace_back(vdist);

  vehicle_model.computeFx(x, u, params, f_of_dx);

  for (size_t k = 0; k < Model::state_dim; ++k) {
    ASSERT_DOUBLE_EQ(f_of_dx(k), analytical_fx_vec[k]);
    ns_utils::print("Analytical comparison f(x,u, param): ", f_of_dx(k), analytical_fx_vec[k]);
  }

  // Analytical derivative f wrt v : df/dv  = A(:, 6th_col)
  vehicle_model.computeJacobians(x, u, params, A, B);

  EXPECT_DOUBLE_EQ(B(6, 0), 1.);
  EXPECT_DOUBLE_EQ(B(7, 1), 1.);

  std::vector<double> analytical_df_dv_vec;  // {Model::state_dim};

  analytical_df_dv_vec.emplace_back(cos(beta + yaw));
  analytical_df_dv_vec.emplace_back(sin(beta + yaw));
  analytical_df_dv_vec.emplace_back(sin(beta) / paramsVehicle.lr);
  analytical_df_dv_vec.emplace_back(cos(beta + eyaw) / (EPS + 1 - kappa * ey));
  analytical_df_dv_vec.emplace_back(sin(beta + eyaw));
  analytical_df_dv_vec.emplace_back(analytical_df_dv_vec[2] - kappa * analytical_df_dv_vec[3]);
  analytical_df_dv_vec.emplace_back(0.);
  analytical_df_dv_vec.emplace_back(0.);
  analytical_df_dv_vec.emplace_back(1.);

  for (size_t k = 0; k < Model::state_dim; ++k) {
    ASSERT_DOUBLE_EQ(A(k, 6), analytical_df_dv_vec[k]);
    ns_utils::print("Analytical df/dv : ", A(k, 6), analytical_df_dv_vec[k]);
  }

  //  // Debug
  //  ns_utils::print("System dynamical equations values : ");
  //  ns_eigen_utils::printEigenMat(f_of_dx);
  //
  //  ns_utils::print("Jacobians A and B : ");
  //  ns_eigen_utils::printEigenMat(A);
  //  ns_eigen_utils::printEigenMat(B);
  //  // end of debug
}
