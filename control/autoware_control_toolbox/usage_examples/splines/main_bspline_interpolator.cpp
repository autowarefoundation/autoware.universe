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
#include "splines/bsplines_interpolator.hpp"

int main()
{

//  std::random_device rd;
//  std::default_random_engine generator{rd()};
//  std::normal_distribution<double> distribution(0.0, 0.3);
  auto log_path = getOutputPath() / "bspline_interp";


  // <--------------- Dimension Expansion -------------------------------->

  {
    // Generate a noisy sinusoidal signal with arc-length parametrization. This is our test signal.
    size_t const base_size = 100; // number of points in the signal.
    size_t const new_size = 50; // data will be expanded to this size.


    // Generate x.
    double kx = 8;
    std::vector<double> xvec = ns_utils::linspace<double>(0.0, kx, base_size);

    // Generate y = sin(x).
    double cy = 10;
    std::vector<double> yvec;
    std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [&](auto const &x)
    {
      return cy * sin(x);
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
    Eigen::MatrixXd
      xe = Eigen::Map<Eigen::Matrix<double, base_size, 1 >>(xvec.data());

    Eigen::MatrixXd ye(xe.unaryExpr([&](auto x)
                                    { return cy * sin(x); }));

    Eigen::MatrixXd ze(xe.unaryExpr([&](auto x)
                                    { return 2 * cos(x) - 3 * sin(x); }));

    Eigen::MatrixXd se;
    se = Eigen::Map<Eigen::Matrix<double, base_size, 1 >>(svec.data());

    // Create a matrix to be interpolated into a new size.
    auto yze = ns_eigen_utils::hstack<double>(ye, ze);
    Eigen::MatrixXd yz_interp_newsize;
    writeToFile(log_path, yze, "yze");

    // Create a new smoothing spline.
    ns_splines::BSplineInterpolator interpolating_bspline(base_size, new_size, 0.3);

    // Different size multi-column interpolation. Expanding the data points.
    interpolating_bspline.InterpolateInCoordinates(yze, yz_interp_newsize);

    writeToFile(log_path, yz_interp_newsize, "yz_interp_newsize");

    // Save coordinates to plot on the same scale
    auto tvec_base = Eigen::VectorXd::LinSpaced(base_size, 0.0, 1.);
    auto tvec_new = Eigen::VectorXd::LinSpaced(new_size, 0.0, 1.0);

    writeToFile(log_path, tvec_base, "tvec_base");
    writeToFile(log_path, tvec_new, "tvec_new");

    // TEST with New Constructors, base coordinates and new coordinates are given.
    // Save coordinates to plot on the same scale
    size_t const new_size_1 = 60; //static_cast<size_t>(base_size / 2);

    auto tvec_base_1 = Eigen::VectorXd::LinSpaced(base_size, 10.0, 1.);
    auto tvec_new_1 = Eigen::VectorXd::LinSpaced(new_size_1, 8.0, 2.0);

    ns_splines::BSplineInterpolator interpolating_bspline_1(tvec_base_1, tvec_new_1,
                                                            0.3);

    Eigen::MatrixXd yz_interp_new_coord;
    interpolating_bspline_1.InterpolateInCoordinates(yze, yz_interp_new_coord);

    writeToFile(log_path, tvec_base_1, "tvec_base_1");
    writeToFile(log_path, tvec_new_1, "tvec_new_1");
    writeToFile(log_path, yz_interp_new_coord, "yz_interp_new_coord");

//    ns_eigen_utils::printEigenMat(tvec_base_1);

  }

  // For curvature computations. The derivatives option is set true.
  {

    // Generate a noisy sinusoidal signal with arc-length parametrization. This is our test signal.
    size_t const base_size = 122; // number of points in the signal.
    size_t const new_size = 50; // data will be expanded into this size.

    // Generate x.
    double kx = 8;
    std::vector<double> xvec = ns_utils::linspace<double>(0.0, kx, base_size);

    // Generate y = sin(x).
    double cy = 10;
    std::vector<double> yvec;
    std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [&](auto const &x)
    {
      return cy * sin(x);
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
    Eigen::MatrixXd
      xe = Eigen::Map<Eigen::Matrix<double, base_size, 1 >>(xvec.data());

    Eigen::MatrixXd ye(xe.unaryExpr([&](auto x)
                                    { return cy * sin(x); }));

    Eigen::MatrixXd ze(xe.unaryExpr([&](auto x)
                                    { return 2 * cos(x) - 3 * sin(x); }));

    Eigen::MatrixXd se;
    se = Eigen::Map<Eigen::Matrix<double, base_size, 1 >>(svec.data());

    // Create a matrix to be interpolated into a new size.
    auto yze = ns_eigen_utils::hstack<double>(ye, ze);
    Eigen::MatrixXd yz_interp_newsize;
    writeToFile(log_path, yze, "yze");

    // Create a new smoothing spline with compute derivatives option set true. Giving only the lengths.
    const double timer_interpolation = tic();
    ns_splines::BSplineInterpolator interpolating_bspline(base_size, new_size, 0.3, true);
    fmt::print("{:<{}}{:.2f}ms\n", "Time: Bspline interpolator with its derivatives construction time :", 50, toc
      (timer_interpolation));

    // Different size multi-column interpolation. Expanding the data points.
    interpolating_bspline.InterpolateInCoordinates(yze, yz_interp_newsize);

    writeToFile(log_path, yz_interp_newsize, "yz_interp_newsize");

    // Save coordinates to plot on the same scale
    auto tvec_base = Eigen::VectorXd::LinSpaced(base_size, 0.0, 1.);
    auto tvec_new = Eigen::VectorXd::LinSpaced(new_size, 0.0, 1.0);

    writeToFile(log_path, tvec_base, "tvec_base");
    writeToFile(log_path, tvec_new, "tvec_new");

    // TEST with New Constructors, base coordinates and new coordinates are given.
    // Save coordinates to plot on the same scale
    size_t const new_size_1 = 50; //static_cast<size_t>(base_size / 2);

    auto tvec_base_1 = Eigen::VectorXd::LinSpaced(base_size, 1.0, 10.);
    auto tvec_new_1 = Eigen::VectorXd::LinSpaced(new_size_1, 2.0, 8.0);

    const double timer_interpolation2 = tic();
    ns_splines::BSplineInterpolator interpolating_bspline_1(tvec_base_1, tvec_new_1,
                                                            0.3, true);

    fmt::print("{:<{}}{:.2f}ms\n", "Time: Bspline interpolator with its derivatives construction time :", 50, toc
      (timer_interpolation2));

    Eigen::MatrixXd yz_interp_new_coord;
    interpolating_bspline_1.InterpolateInCoordinates(yze, yz_interp_new_coord);

    writeToFile(log_path, tvec_base_1, "tvec_base_1");
    writeToFile(log_path, tvec_new_1, "tvec_new_1");
    writeToFile(log_path, yz_interp_new_coord, "yz_interp_new_coord");

    // COMPUTE DERIVATIVES of GIVEN vector or matrix (cols are interpreted).
    // Get xdot, ydot
    /*
     *  Note, all the constructors are independent of base data itself. Only the base length is used for the
     *  construction. The same interpolator can be used for different data as we do here. We constructed with yze but
     *  we will take derivative of xy vectors from the previous lines.
     *
     * */
    Eigen::MatrixXd rdot_interp(new_size_1, 2); // [xdot, ydot]
    Eigen::MatrixXd rddot_interp(new_size_1, 2); // [xdot, ydot]
    auto xy_data = ns_eigen_utils::hstack<double>(xe, ye);

    interpolating_bspline_1.getFirstDerivative(xy_data, rdot_interp);
    interpolating_bspline_1.getSecondDerivative(xy_data, rddot_interp);

    writeToFile(log_path, rdot_interp, "rdot_interp");
    writeToFile(log_path, rddot_interp, "rddot_interp");

    // Curvature from the spline
    Eigen::MatrixXd curvature_bspline_interpolator;
    curvature_bspline_interpolator = ns_eigen_utils::Curvature(rdot_interp, rddot_interp);

    writeToFile(log_path, curvature_bspline_interpolator, "curvature_bspline_interpolator");

    // To compare the derivatives with the original derivatives, we compute them analytically.
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

  }

  // POSSIBLE IMPLEMENTATION in ROS
  {
    // Generate a noisy sinusoidal signal with arc-length parametrization. This is our test signal.
    size_t const base_size = 122; // number of points in the signal.
    size_t const new_size = 60; // data will be downsampled into this size.

    // Generate x.
    double kx = 8;
    std::vector<double> xvec = ns_utils::linspace<double>(0.0, kx, base_size);

    // Generate y = sin(x).
    double cy = 10;
    std::vector<double> yvec;
    std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [&](auto const &x)
    {
      return cy * sin(x);
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
    Eigen::MatrixXd
      xe = Eigen::Map<Eigen::Matrix<double, base_size, 1 >>(xvec.data());

    Eigen::MatrixXd ye(xe.unaryExpr([&cy](auto x)
                                    { return cy * sin(x); }));

    Eigen::MatrixXd ze(xe.unaryExpr([](auto x)
                                    { return 2 * cos(x) - 3 * sin(x); }));

    Eigen::MatrixXd se;
    se = Eigen::Map<Eigen::Matrix<double, base_size, 1 >>(svec.data());


    // Combine se, xe, ye, ze --> these are assumed to be given or taken from the trajectory.
    auto sxyze_ref = ns_eigen_utils::hstack<double>(se, xe, ye, ze);
    writeToFile(log_path, sxyze_ref, "sxyz_ref");

    // Now we want to downsample these variables into the new size.
    // Create a new smoothing spline with compute derivatives option set true. Giving only the lengths.
    const double timer_interpolation = tic();

    ns_splines::BSplineInterpolator interpolating_bspline_ROS(base_size, new_size, 0.3, true);

    fmt::print("{:<{}}{:.2f}ms\n", "Time: Bspline ROS interpolator with its derivatives construction time :", 50, toc
      (timer_interpolation));

    // The downsampled and smoothed map is obtained by;
    Eigen::MatrixXd smoothed_map_ROS;
    interpolating_bspline_ROS.InterpolateInCoordinates(sxyze_ref, smoothed_map_ROS);

    writeToFile(log_path, smoothed_map_ROS, "smoothed_map_ROS");


    // CURVATURE ROS example.
    Eigen::MatrixXd rdot_interp(new_size, 2); // [xdot, ydot]
    Eigen::MatrixXd rddot_interp(new_size, 2); // [xdot, ydot]
    auto xy_data = ns_eigen_utils::hstack<double>(xe, ye);

    interpolating_bspline_ROS.getFirstDerivative(xy_data, rdot_interp);
    interpolating_bspline_ROS.getSecondDerivative(xy_data, rddot_interp);

    // Get [xdot, ydot] and [xddot, yddot].
    interpolating_bspline_ROS.getFirstDerivative(xy_data, rdot_interp);
    interpolating_bspline_ROS.getSecondDerivative(xy_data, rddot_interp);

    writeToFile(log_path, rdot_interp, "rdot_interp");
    writeToFile(log_path, rddot_interp, "rddot_interp");

    // Curvature from the spline
    Eigen::MatrixXd curvature_bspline_interpolator_ROS;
    curvature_bspline_interpolator_ROS = ns_eigen_utils::Curvature(rdot_interp, rddot_interp);

    writeToFile(log_path, curvature_bspline_interpolator_ROS, "curvature_bspline_interpolator_ROS");

    // Curvature Original
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
    Eigen::MatrixXd curvature_orginal_ROS; //(xe.rows(), 1);
    curvature_orginal_ROS = ns_eigen_utils::Curvature(rdt, rdt2);

    //  ns_eigen_utils::printEigenMat(curvature_orginal);
    writeToFile(log_path, curvature_orginal_ROS, "curvature_original_ROS");

  }

  return 0;
}