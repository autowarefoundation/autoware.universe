// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_CONTROL_TOOLBOX_INCLUDE_SPLINES_BSPLINES_SMOOTHER_HPP_
#define AUTOWARE_CONTROL_TOOLBOX_INCLUDE_SPLINES_BSPLINES_SMOOTHER_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>
#include <algorithm>
#include <functional>
#include <iostream>
#include <numeric>
#include <vector>

#include "utils_act/act_utils.hpp"
#include "utils_act/act_utils_eigen.hpp"

namespace ns_splines
{
/*
 *  Main References : Semiparametric Regression by David Rupert et al.
 *                    The Elements of Statistical Learning
 *
 * sbase a monotonic distance curve, ybase represents data to be interpolated.
 * The end-point of the trajectory is interpolated as line. The curvature must not be used at the last interval of
 * the trajectory interpolation.
 *
 * We compute a projection matrix once and can use this projection matrix with other data. No need to compute the
 * projection matrix for the new data.
 *
 *  -Heading angle should not be interpolated by the splines as they are periodic and needed to be treated  differently.
 *  - All the vectors must be column vector.
 *  - We use same size for interpolated vector.
 *   y(t) = f(t) =  [1, t, N0, N1, N2, .... ] where N0:Nk ~= (t - ti)**3 with basis
 * */
class BSplineSmoother
{
 public:
  BSplineSmoother() = default;

  explicit BSplineSmoother(size_t base_signal_length, double num_of_knots_ratio = 0.3);

  // Multicolumn matrix can also be interpolated. MatrixBase and Interpolated matrix must have
  // the same size.
  void InterpolateInCoordinates(Eigen::MatrixXd const &ybase, Eigen::MatrixXd &data_tobe_interpolated);

  void getFirstDerivative(Eigen::MatrixXd const &ybase, Eigen::MatrixXd &ybase_dot) const;

  void getSecondDerivative(Eigen::MatrixXd const &ybase, Eigen::MatrixXd &ybase_dot_dot) const;

 private:
  // Pre-settings. Increasing lambda yield more smooth and flattened curve.
  // smoothing factor used in normal form of LS; B*B + (lambda**2)*D*D, D is f''(x).
  double lambda_ = 0.001;

  // Initialized during instantiation.
  size_t npoints_{};      // !<-@brief number of points in the signal to be smoothed.
  double knots_ratio_{};  // !<-@brief ratio of the number of knots to num of interpolation points.
  size_t nknots_{};
  std::vector<double> knots_vec_;  // knot points of the global smoother.

  // To be computed in the constructor.
  /*
   *   y = Ax --> At*y = At*x -- > x = coeffs = inv(At*A)*A*ybase_data and A = A(t parameter)
   *   -- > ynew = Anew(t) * x  = Anew(t) * coeffs  = Anew(t) * inv(At*A)*A * ydata
   *   -- > projection matrix p = inv(At*A)*A
   *   ---> projection with base included  Anew(t) * inv(At*A)*A
   * */
  Eigen::MatrixXd projection_mat_base_;

  // !<-@brief coeffs = ProjectionMat @ ydata and ysmooth = Basis_mat*ydata
  Eigen::MatrixXd projection_mat_wb_;

  // !<-@brief ysmooth_dot = basis_dot * projection_mat_dot*ydata
  Eigen::MatrixXd projection_mat_dot_wb_;

  // !<-@brief ysmooth_ddot = basis_ddot * projection_mat_ddot*ydata - second derivative
  Eigen::MatrixXd projection_mat_ddot_wb_;

  // Inner Methods.
  void createBasesMatrix(std::vector<double> const &tvec,
                         Eigen::MatrixXd &basis_mat,
                         Eigen::MatrixXd &basis_dmat,
                         Eigen::MatrixXd &basis_ddmat);

  std::vector<std::vector<double>> basisRowsWithDerivatives(double const &ti);

  std::vector<double> fPlusCube(double const &ti, double const &ki) const;  // returns max(0, t);

  void solveByDemmlerReisch(Eigen::MatrixXd &basis_mat,
                            Eigen::MatrixXd &basis_dmat,
                            Eigen::MatrixXd &basis_ddmat);  // set projection mat.

  void solveByQR(Eigen::MatrixXd &basis_mat,
                 Eigen::MatrixXd &basis_dmat,
                 Eigen::MatrixXd &basis_ddmat);

  void createBasesMatrix(const Eigen::MatrixXd &tvec,
                         Eigen::MatrixXd &basis_mat,
                         Eigen::MatrixXd &basis_dmat,
                         Eigen::MatrixXd &basis_ddmat);
};
}  // namespace ns_splines

#endif //AUTOWARE_CONTROL_TOOLBOX_INCLUDE_SPLINES_BSPLINES_SMOOTHER_HPP_
