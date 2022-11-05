// Copyright 2022 The Autoware Foundation.
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

#include "act_test_suite.hpp"
#include "gtest/gtest.h"

TEST(ACTspline, bSplineInterpolatorDownSampling)
{
  // Generate a noisy sinusoidal signal with arc-length parametrization. This is our test signal.
  size_t const base_size = 100;  // number of points in the signal.
  size_t const new_size = 50;    // data will be expanded to this size.

  // Generate x.
  double const kx = 8;

  // Generate y = sin(x).
  double const cy = 10;

  auto tbase = Eigen::VectorXd::LinSpaced(base_size, 0.0, kx);
  auto tnew = Eigen::VectorXd::LinSpaced(new_size, 0.0, kx);

  Eigen::MatrixXd ye(tbase.unaryExpr([&cy](auto const & t) { return cy * sin(t); }));

  Eigen::MatrixXd ze(tbase.unaryExpr([](auto const & t) { return 2 * cos(t) - 3 * sin(t); }));

  // Create a matrix to be interpolated into a new size.
  auto const & yze = ns_eigen_utils::hstack<double>(ye, ze);
  Eigen::MatrixXd yz_interp_newsize;

  // Create a new smoothing spline.
  ns_splines::BSplineInterpolator interpolating_bspline(base_size, new_size, 0.3);

  // Different size multi-column interpolation. Expanding the data points.
  interpolating_bspline.InterpolateInCoordinates(yze, yz_interp_newsize);

  /**
   * Create validating sets for y and z vectors.
   * */
  Eigen::MatrixXd ye_validation(tnew.unaryExpr([&cy](auto const & t) { return cy * sin(t); }));

  Eigen::MatrixXd ze_validation(tnew.unaryExpr([&](auto x) { return 2 * cos(x) - 3 * sin(x); }));

  auto error_y = yz_interp_newsize.col(0) - ye_validation;
  auto error_y_rms = std::sqrt(error_y.array().square().sum() / new_size) / cy;  // normalized to 1.

  ns_utils::print("ye_validation size ", ye_validation.rows(), ye_validation.cols());
  ns_utils::print("yze rows cols ", yz_interp_newsize.rows(), yz_interp_newsize.cols());
  ns_utils::print("Approximation error for y :", error_y_rms);
  ASSERT_LE(error_y_rms, 1e-2);  // this is a global curve fitting with down-sampling test.
}

TEST(ACTspline, bSplineCurvature)
{
  const size_t Nin = 120;
  const size_t Nout = 80;

  // Generate theta.
  std::vector<double> theta = ns_utils::linspace<double>(-M_PI / 2, M_PI / 2, Nin);

  Eigen::MatrixXd theta_eg;
  theta_eg = Eigen::Map<Eigen::Matrix<double, Nin, 1>>(theta.data());

  // Define a curve radius
  const double R = 100;

  // EIGEN IMPLEMENTATION.
  Eigen::MatrixXd xe(theta_eg.unaryExpr([&R](auto x) { return R * cos(x); }));

  Eigen::MatrixXd ye(theta_eg.unaryExpr([&R](auto x) { return R * sin(x); }));

  // Create a new smoothing spline.
  double knot_number_ratio = 0.3;
  ns_splines::BSplineInterpolatorTemplated<Nin, Nout> interpolating_bspline(
    knot_number_ratio, true);

  // Get xdot, ydot
  Eigen::MatrixXd rdot_interp(Nout, 2);  // [xdot, ydot]

  // Get xddot, yddot
  Eigen::MatrixXd rddot_interp(Nout, 2);  // [xddot, yddot]

  auto xy_data = ns_eigen_utils::hstack<double>(xe, ye);
  interpolating_bspline.getFirstDerivative(xy_data, rdot_interp);
  interpolating_bspline.getSecondDerivative(xy_data, rddot_interp);

  // Curvature from the B-spline
  Eigen::MatrixXd curvature_bspline_smoother;
  curvature_bspline_smoother = ns_eigen_utils::Curvature(rdot_interp, rddot_interp);

  // Compute the curvature approximation error.
  Eigen::MatrixXd error_curvature(
    curvature_bspline_smoother.unaryExpr([&R](auto & x) { return x - 1 / R; }));

  auto error_curvature_rms = std::sqrt(error_curvature.array().square().sum() / Nout);
  ns_utils::print("Error Curvature RMS :", error_curvature_rms);

  ASSERT_LE(error_curvature_rms, 1e-3);
}
