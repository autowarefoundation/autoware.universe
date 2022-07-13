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

#include "utils_act/writetopath.hpp"
#include "utils_act/act_utils.hpp"
#include <vector>
#include <algorithm>
#include <numeric>
#include <random>
#include <fmt/core.h>
#include "utils_act/timekeep.hpp"
#include "splines/bsplines_smoother.hpp"

int main()
{

  std::random_device rd;
  std::default_random_engine generator{rd()};
  std::normal_distribution<double> distribution(0.0, 0.3);
  auto log_path = getOutputPath() / "bspline_smooth";

  {
    // Generate a noisy sinusoidal signal with arc-length parametrization. This is our test signal.
    size_t const num_of_points = 100; // number of points in the signal.

    // Generate x.
    double kx = 8;
    std::vector<double> xvec = ns_utils::linspace<double>(0.0, kx, num_of_points);

    // Generate y = sin(x).
    double cy = 10;
    std::vector<double> yvec;
    std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [&](auto const &x)
    {
      return cy * sin(x) + distribution(generator) * 0;
    });

    std::vector<double> zvec;
    std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(zvec), [&](auto const &x)
    {
      return 2 * cos(x) - 3 * sin(x);
    });


    // Arc-length parametrization.
    std::vector<double> dx; //{1, 0.0};
    std::vector<double> dy; //{1, 0.0};

    std::adjacent_difference(xvec.begin(), xvec.end(), std::back_inserter(dx));
    std::adjacent_difference(yvec.begin(), yvec.end(), std::back_inserter(dy));

    // Define arc-length cumsum()
    std::vector<double> svec;
    std::transform(dx.cbegin(), dx.cend(), dy.cbegin(), std::back_inserter(svec),
                   [](auto dxi, auto dyi)
                   {
                     static double ds = 0.0;
                     ds += std::hypot(dxi, dyi);

                     return ds;
                   });

    writeToFile(log_path, xvec, "xvec");
    writeToFile(log_path, yvec, "yvec");
    writeToFile(log_path, zvec, "zvec");
    writeToFile(log_path, svec, "svec");

    /*
     In case we use random noise.
    // Generate noise.
      std::random_device rd;
      std::default_random_engine generator{rd()};
      std::normal_distribution<double> distribution(0.0, 0.3);
     */


    // Create new interpolating coordinates
    auto snew = ns_utils::linspace(svec[0], svec.back(), 101);


    // Create a spline object from the x
    // Default spline type is 'spline'. We can choose 'line' as an additional implementation.

    // < -------------------------- FULL EIGEN IMPLEMENTATION --------------------------------->

    Eigen::MatrixXd xe;
    xe = Eigen::Map<Eigen::Matrix<double, num_of_points, 1 >>(xvec.data());

    Eigen::MatrixXd ye(xe.unaryExpr([&](auto x)
                                    { return cy * sin(x) + distribution(generator); }));

    Eigen::MatrixXd ze(xe.unaryExpr([&](auto x)
                                    { return 2 * cos(x) - 3 * sin(x) + distribution(generator); }));

    Eigen::MatrixXd se;
    se = Eigen::Map<Eigen::Matrix<double, num_of_points, 1 >>(svec.data());

    Eigen::Index new_ssize = num_of_points;
    auto se_new = Eigen::VectorXd::LinSpaced(new_ssize, 0.0, se(se.rows() - 1));

//  ns_eigen_utils::printEigenMat(se_new.bottomRows(se_new.rows() - 1));

    writeToFile(log_path, xe, "xe");
    writeToFile(log_path, ye, "ye");
    writeToFile(log_path, ze, "ze");
    writeToFile(log_path, se, "se");
    writeToFile(log_path, se_new, "snew");

    // Load model_ parameters from its info file and set initialized true
    const double timer_interpolation = tic();
    ns_splines::BSplineSmoother smoothing_spline(se.rows(), 0.3);
    fmt::print("{:<{}}{:.2f}ms\n", "Time: spline initialization:", 50, toc(timer_interpolation));

    // Evaluate Interpolator
    Eigen::MatrixXd yinterp(ye.rows(), ye.cols());
    const double timer_evaluation = tic();

    smoothing_spline.InterpolateInCoordinates(ye, yinterp);

    fmt::print("{:<{}}{:.2f}ms\n", "Time: spline evaluation:", 50, toc(timer_evaluation));
    writeToFile(log_path, yinterp, "yinterp");

    // Evaluate another variable using the same Interpolator
    Eigen::MatrixXd zinterp(ye.rows(), ye.cols());
    const double timer_evaluation2 = tic();

    smoothing_spline.InterpolateInCoordinates(ze, zinterp);

    fmt::print("{:<{}}{:.2f}ms\n", "Time: spline evaluation:", 50, toc(timer_evaluation2));
    writeToFile(log_path, zinterp, "zinterp");

    // Multi Dimensional Interpolation
    Eigen::MatrixXd yze(ye.rows(), 2);
    yze.col(0) = ye;
    yze.col(1) = ze;
    writeToFile(log_path, yze, "yze");

    Eigen::MatrixXd yzinterp(ye.rows(), 2);

    const double timer_evaluation3 = tic();
    smoothing_spline.InterpolateInCoordinates(yze, yzinterp);
    fmt::print("{:<{}}{:.2f}ms\n", "Time: spline evaluation:", 50, toc(timer_evaluation3));
    writeToFile(log_path, yzinterp, "yzinterp");
  }

  // < ------------------- Given X, Y compute curvature by smoothing Bsplines ----------- >
  {
    // Generate a noisy sinusoidal signal with arc-length parametrization. This is our test signal.
    size_t const num_of_points = 100; // number of points in the signal.


    // Generate x.
    double kx = 8;
    std::vector<double> xvec = ns_utils::linspace<double>(0.0, kx, num_of_points);

    // Generate y = sin(x).
    double cy = 10;
    std::vector<double> yvec;
    std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [&](auto const &x)
    {
      return cy * sin(x) + distribution(generator) * 0;
    });

    // Arc-length parametrization.
    std::vector<double> dx; //{1, 0.0};
    std::vector<double> dy; //{1, 0.0};

    std::adjacent_difference(xvec.begin(), xvec.end(), std::back_inserter(dx));
    std::adjacent_difference(yvec.begin(), yvec.end(), std::back_inserter(dy));

    // Define arc-length cumsum()
    std::vector<double> svec;
    std::transform(dx.cbegin(), dx.cend(), dy.cbegin(), std::back_inserter(svec),
                   [](auto dxi, auto dyi)
                   {
                     static double ds = 0.0;
                     ds += std::hypot(dxi, dyi);

                     return ds;
                   });

    // EIGEN IMPLEMENTATION.
    Eigen::MatrixXd xe;
    xe = Eigen::Map<Eigen::Matrix<double, num_of_points, 1 >>(xvec.data());

    Eigen::MatrixXd ye(xe.unaryExpr([&](auto x)
                                    { return cy * sin(x) + distribution(generator); }));

    Eigen::MatrixXd ze(xe.unaryExpr([&](auto x)
                                    { return 2 * cos(x) - 3 * sin(x) + distribution(generator); }));
    // Generate arc length
    Eigen::MatrixXd se;
    se = Eigen::Map<Eigen::Matrix<double, num_of_points, 1 >>(svec.data());

    // Compute original curvature.
    Eigen::MatrixXd dxdt(xe.rows(), 1);

    // First derivative
    dxdt.setConstant(kx); // dxdt = k*x and kx is k of x
//  std::cout << " dxdt " << std::endl;
//  ns_eigen_utils::printEigenMat(dxdt);

    Eigen::MatrixXd dydt(xe.unaryExpr([&](auto x)
                                      { return cy * kx * cos(x); }));

//  std::cout << "dydt " << std::endl;
//  ns_eigen_utils::printEigenMat(dydt);

    // Second derivative
    Eigen::MatrixXd dxdt2(xe.rows(), 1);
    dxdt2.setZero();

    Eigen::MatrixXd dydt2(xe.unaryExpr([&](auto x)
                                       { return -cy * kx * kx * sin(x); }));

    // compute r0, r1 as r0 = [dxdt, dydt] and r1[dxdt2, dydt2]
    auto rdt = ns_eigen_utils::hstack<double>(dxdt, dydt);
    auto rdt2 = ns_eigen_utils::hstack<double>(dxdt2, dydt2);

    // Cross product example.
    //  Eigen::MatrixXd cross_product(xe.rows(), 1);
    //    auto cross_product = ns_eigen_utils::crossProduct<double>(rdt, rdt2);

    // Curvature example.
    Eigen::MatrixXd curvature_orginal; //(xe.rows(), 1);
    curvature_orginal = ns_eigen_utils::Curvature(rdt, rdt2);
//  ns_eigen_utils::printEigenMat(curvature_orginal);
    writeToFile(log_path, curvature_orginal, "curvature_original");

    // Create a new smoothing spline.
    ns_splines::BSplineSmoother smoothing_spline(num_of_points, 0.3);

    // Get xdot, ydot
    Eigen::MatrixXd rdot_interp(num_of_points, 2); // [xdot, ydot]

    // Get xddot, yddot
    Eigen::MatrixXd rddot_interp(num_of_points, 2); // [xdot, ydot]

    auto xy_data = ns_eigen_utils::hstack<double>(xe, ye);
    smoothing_spline.getFirstDerivative(xy_data, rdot_interp);
    smoothing_spline.getSecondDerivative(xy_data, rddot_interp);

    writeToFile(log_path, rdot_interp, "rdot_interp");
    writeToFile(log_path, rddot_interp, "rddot_interp");

    // Curvature from the B-spline
    Eigen::MatrixXd curvature_bspline_smoother;
    curvature_bspline_smoother = ns_eigen_utils::Curvature(rdot_interp, rddot_interp);
    writeToFile(log_path, curvature_bspline_smoother, "curvature_bspline_interpolator");

  }

  return 0;
}