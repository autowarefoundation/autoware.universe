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

#include "gtest/gtest.h"
#include "act_test_suite.hpp"

/**
 * Test cubic spline interpolator which uses Preconditioned Conjugate Gradient optimization method.
 * */
TEST(ACTspline, splinePCapproximationVectorwise)
{

  // Generate a sinusoidal signal with time parametrization.
  size_t const num_of_points = 100;  // number of points in the signal.

  double const &omega = 0.3;  // sin-wave speed.

  // Generate x.
  std::vector<double> tbase = ns_utils::linspace(0.0, 10.0, num_of_points);

  // Generate y = sin(t).
  std::vector<double> ybase;
  std::transform(
    tbase.cbegin(), tbase.cend(), std::back_inserter(ybase), [&omega](auto const &t)
    { return sin(omega * t); });

  // Create new interpolating coordinates
  size_t const &new_num_of_points = 50;
  auto tnew = ns_utils::linspace(tbase[0], tbase.back(), new_num_of_points);

  std::vector<double> yvalidation;
  std::transform(
    tnew.cbegin(), tnew.cend(), std::back_inserter(yvalidation), [&omega](auto const &t)
    { return sin(omega * t); });

  // Create a spline object from the x
  // Default spline type is 'spline'. We can choose 'line' as an additional implementation.

  size_t const &interpolating_type = 3;  // 3 for spline, 1 for line.
  auto interpolating_spline_aw = ns_splines::InterpolatingSplinePCG(interpolating_type);

  /**
   * Interpolate and store the solutions in yinterp. The interpolator does not keep the computed spline coefficients.
   * This construction is memoryless.
   * */
  std::vector<double> yinterp;
  auto is_interpolated = interpolating_spline_aw.Interpolate(tbase, ybase, tnew, yinterp);
  ns_utils::print("Is interpolated :", is_interpolated);

  ASSERT_TRUE(is_interpolated);

  /**
   * Compute the interpolation error
   * */

  std::vector<double> interpolation_error;
  std::transform(yvalidation.cbegin(), yvalidation.cend(), yinterp.cbegin(),
                 std::back_inserter(interpolation_error),
                 [](auto const &x0, auto const &x1)
                 {
                   return (x1 - x0) * (x1 - x0);
                 });

  auto const &sum_of_squares = std::accumulate(interpolation_error.cbegin(), interpolation_error.cend(), 0.);
  auto norm_of_error = std::sqrt(sum_of_squares);

  ns_utils::print("Norm of spline interpolation error : ", norm_of_error);

  double error_tol = 1e-5; // more than enough for spline approximation.

  double mean_error = norm_of_error / static_cast<double>( new_num_of_points);
  ns_utils::print("Mean value of the approximation : ", mean_error);

  ASSERT_LE(mean_error, error_tol);
}

/**
 * Test cubic spline interpolator point wise
 * */
TEST(ACTspline, splinePCapproximationPointwise)
{

  // Generate a sinusoidal signal with time parametrization.
  size_t const num_of_points = 100;  // number of points in the signal.

  double const &omega = 0.3;  // sin wave speed.

  // Generate x.
  std::vector<double> tbase = ns_utils::linspace(0.0, 10.0, num_of_points);

  // Generate y = sin(t).
  std::vector<double> ybase;
  std::transform(
    tbase.cbegin(), tbase.cend(), std::back_inserter(ybase), [&omega](auto const &t)
    { return sin(omega * t); });


  // Create new interpolating coordinates
  size_t const &new_num_of_points = 50;
  auto tnew = ns_utils::linspace(tbase[0], tbase.back(), new_num_of_points);

  std::vector<double> yvalidation;
  std::transform(
    tnew.cbegin(), tnew.cend(), std::back_inserter(yvalidation), [&omega](auto const &t)
    { return sin(omega * t); });

  // Create a spline object from the x
  // Default spline type is 'spline'. We can choose 'line' as an additional implementation.

  size_t const &interpolating_type = 3;  // 3 for spline, 1 for line.
  auto interpolating_spline_aw = ns_splines::InterpolatingSplinePCG(interpolating_type);


  /**
   * Initialize the spline and use it many times as long as the base data do not change.
   * */
  interpolating_spline_aw.Initialize(tbase, ybase);

  /**
   * Interpolate and store the solutions in yinterp. The interpolator does keep the computed spline coefficients.
   * */
  std::vector<double> yinterp;

  for (auto const &t : tnew)
  {
    double ypoint{};
    interpolating_spline_aw.Interpolate(t, ypoint);
    yinterp.emplace_back(ypoint);
  }

  /**
   * Compute the interpolation error
   * */

  std::vector<double> interpolation_error;
  std::transform(yvalidation.cbegin(), yvalidation.cend(), yinterp.cbegin(),
                 std::back_inserter(interpolation_error),
                 [](auto const &x0, auto const &x1)
                 {
                   return (x1 - x0) * (x1 - x0);
                 });

  auto const &sum_of_squares = std::accumulate(interpolation_error.cbegin(), interpolation_error.cend(), 0.);
  auto norm_of_error = std::sqrt(sum_of_squares);

  ns_utils::print("Norm of spline interpolation error : ", norm_of_error);

  double error_tol = 1e-5; // more than enough for spline approximation.

  double mean_error = norm_of_error / static_cast<double>( new_num_of_points);
  ns_utils::print("Mean value of the approximation : ", mean_error);

  ASSERT_LE(mean_error, error_tol);
}


/**
 * Test line interpolator which uses Preconditioned Conjugate Gradient optimization method.
 * */
TEST(ACTspline, linePCGapproximation)
{

  // Generate a sinusoidal signal with time parametrization.
  size_t const num_of_points = 100;  // number of points in the signal.

  // Generate x.
  std::vector<double> tbase = ns_utils::linspace(0.0, 10.0, num_of_points);

  // Generate y = ac*t + b.
  double const ac = 1.8;
  double const bc = 20.;

  std::vector<double> ybase;
  std::transform(
    tbase.cbegin(), tbase.cend(), std::back_inserter(ybase), [&ac, &bc](auto const &t)
    { return ac * t + bc; });

  // Create new interpolating coordinates
  size_t const &new_num_of_points = 50;
  auto tnew = ns_utils::linspace(tbase[0], tbase.back(), new_num_of_points);

  std::vector<double> yvalidation;
  std::transform(
    tnew.cbegin(), tnew.cend(), std::back_inserter(yvalidation), [&ac, &bc](auto const &t)
    { return ac * t + bc; });
  // Create a spline object from the x
  // Default spline type is 'spline'. We can choose 'line' as an additional implementation.

  size_t const &interpolating_type = 1;  // 3 for spline, 1 for line.
  auto interpolating_line_aw = ns_splines::InterpolatingSplinePCG(interpolating_type);

  /**
   * Interpolate and store the solutions in yinterp. The interpolator does not keep the computed spline coefficients.
   * This construction is memoryless.
   * */
  std::vector<double> yinterp;
  auto is_interpolated = interpolating_line_aw.Interpolate(tbase, ybase, tnew, yinterp);
  ns_utils::print("Is interpolated :", is_interpolated);

  ASSERT_TRUE(is_interpolated);

  /**
   * Compute the interpolation error
   * */

  std::vector<double> interpolation_error;
  std::transform(yvalidation.cbegin(), yvalidation.cend(), yinterp.cbegin(),
                 std::back_inserter(interpolation_error),
                 [](auto const &x0, auto const &x1)
                 {
                   return (x1 - x0) * (x1 - x0);
                 });

  auto const &sum_of_squares = std::accumulate(interpolation_error.cbegin(), interpolation_error.cend(), 0.);
  auto norm_of_error = std::sqrt(sum_of_squares);

  ns_utils::print("Norm of line interpolation error : ", norm_of_error);

  double error_tol = 1e-5; // more than enough for spline approximation.

  double mean_error = norm_of_error / static_cast<double>( new_num_of_points);
  ns_utils::print("Mean value of the approximation : ", mean_error);

  ASSERT_LE(mean_error, error_tol);
}

/**
 * Test extrapolation
 * */
TEST(ACTspline, linePCGextrapolation)
{

  // Generate a sinusoidal signal with time parametrization.
  size_t const num_of_points = 100;  // number of points in the signal.

  // Generate x.
  std::vector<double> tbase = ns_utils::linspace(0.0, 10.0, num_of_points);

  // Generate y = ac*t + b.
  double const ac = 1.8;
  double const bc = 20.;

  std::vector<double> ybase;
  std::transform(
    tbase.cbegin(), tbase.cend(), std::back_inserter(ybase), [&ac, &bc](auto const &t)
    { return ac * t + bc; });

  // Create new interpolating coordinates
  size_t const &new_num_of_points = 50;

  auto tnew = ns_utils::linspace(tbase[0] - 1., tbase.back() + 1., new_num_of_points);

  std::vector<double> yvalidation;
  std::transform(
    tnew.cbegin(), tnew.cend(), std::back_inserter(yvalidation), [&ac, &bc](auto const &t)
    { return ac * t + bc; });
  // Create a spline object from the x
  // Default spline type is 'spline'. We can choose 'line' as an additional implementation.

  size_t const &interpolating_type = 1;  // 3 for spline, 1 for line.
  auto interpolating_line_aw = ns_splines::InterpolatingSplinePCG(interpolating_type);

  /**
   * Interpolate and store the solutions in yinterp. The interpolator does not keep the computed spline coefficients.
   * This construction is memoryless.
   * */
  std::vector<double> yinterp;
  auto is_interpolated = interpolating_line_aw.Interpolate(tbase, ybase, tnew, yinterp);
  ns_utils::print("Is interpolated :", is_interpolated);

  ASSERT_TRUE(is_interpolated);

  /**
   * Compute the exptrapolation error only for the terms that are out of the range.
   * */

  std::vector<double> interpolation_error;
  std::transform(yvalidation.cbegin(), yvalidation.cend(), yinterp.cbegin(),
                 std::back_inserter(interpolation_error),
                 [&ac, &bc, &tbase](auto const &y0, auto const &y1)
                 {
                   if ((y0 - bc) / ac < tbase[0] || (y0 - bc) / ac > tbase.back())
                   {
                     return (y1 - y0) * (y1 - y0);
                   }
                   return 0.;
                 });

  ns_utils::print_container(interpolation_error);

  auto const &sum_of_squares = std::accumulate(interpolation_error.cbegin(), interpolation_error.cend(), 0.);
  auto norm_of_error = std::sqrt(sum_of_squares);

  ns_utils::print("Norm of line extrapolation error : ", norm_of_error);

  double error_tol = 1e-5; // more than enough for spline approximation.

  double mean_error = norm_of_error / static_cast<double>( new_num_of_points);
  ns_utils::print("Mean value of the approximation : ", mean_error);

  ASSERT_LE(mean_error, error_tol);
}

/**
 * Test BSpline Interpolator and Smoothers and Interpolators.
 * */

TEST(ACTspline, bSplineInterpolatorDownSampling)
{
  // Generate a noisy sinusoidal signal with arc-length parametrization. This is our test signal.
  size_t const base_size = 100; // number of points in the signal.
  size_t const new_size = 50; // data will be expanded to this size.


  // Generate x.
  double const kx = 8;

  // Generate y = sin(x).
  double const cy = 10;

  auto tbase = Eigen::VectorXd::LinSpaced(base_size, 0.0, kx);
  auto tnew = Eigen::VectorXd::LinSpaced(new_size, 0.0, kx);

  Eigen::MatrixXd ye(tbase.unaryExpr([&cy](auto const &t)
                                     { return cy * sin(t); }));

  Eigen::MatrixXd ze(tbase.unaryExpr([](auto const &t)
                                     { return 2 * cos(t) - 3 * sin(t); }));

  // Create a matrix to be interpolated into a new size.
  auto const &yze = ns_eigen_utils::hstack<double>(ye, ze);
  Eigen::MatrixXd yz_interp_newsize;

  // Create a new smoothing spline.
  ns_splines::BSplineInterpolator interpolating_bspline(base_size, new_size, 0.3);

  // Different size multi-column interpolation. Expanding the data points.
  interpolating_bspline.InterpolateInCoordinates(yze, yz_interp_newsize);

  /**
   * Create validating sets for y and z vectors.
   * */
  Eigen::MatrixXd ye_validation(tnew.unaryExpr([&cy](auto const &t)
                                               { return cy * sin(t); }));

  Eigen::MatrixXd ze_validation(tnew.unaryExpr([&](auto x)
                                               { return 2 * cos(x) - 3 * sin(x); }));

  auto error_y = yz_interp_newsize.col(0) - ye_validation;
  auto error_y_rms = std::sqrt(error_y.array().square().sum() / new_size) / cy; // normalized to 1.

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
  theta_eg = Eigen::Map<Eigen::Matrix<double, Nin, 1 >>(theta.data());

  // Define a curve radius
  const double R = 100;

  // EIGEN IMPLEMENTATION.
  Eigen::MatrixXd xe(theta_eg.unaryExpr([&R](auto x)
                                        { return R * cos(x); }));

  Eigen::MatrixXd ye(theta_eg.unaryExpr([&R](auto x)
                                        { return R * sin(x); }));

  // Create a new smoothing spline.
  double know_number_ratio = 0.3;
  ns_splines::BSplineInterpolatorTemplated<Nin, Nout> interpolating_bspline(know_number_ratio, true);

  // Get xdot, ydot
  Eigen::MatrixXd rdot_interp(Nout, 2); // [xdot, ydot]

  // Get xddot, yddot
  Eigen::MatrixXd rddot_interp(Nout, 2); // [xddot, yddot]

  auto xy_data = ns_eigen_utils::hstack<double>(xe, ye);
  interpolating_bspline.getFirstDerivative(xy_data, rdot_interp);
  interpolating_bspline.getSecondDerivative(xy_data, rddot_interp);

  // Curvature from the B-spline
  Eigen::MatrixXd curvature_bspline_smoother;
  curvature_bspline_smoother = ns_eigen_utils::Curvature(rdot_interp, rddot_interp);

  // Compute the curvature approximation error.
  Eigen::MatrixXd error_curvature(curvature_bspline_smoother.unaryExpr([&R](auto &x)
                                                                       { return x - 1 / R; }));

  auto error_curvature_rms = std::sqrt(error_curvature.array().square().sum() / Nout);
  ns_utils::print("Error Curvature RMS :", error_curvature_rms);

  ASSERT_LE(error_curvature_rms, 1e-3);
}