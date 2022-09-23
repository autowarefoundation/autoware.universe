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
TEST(ACTspline, splinePCapproximation)
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
                   } else
                   { return 0.; }
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