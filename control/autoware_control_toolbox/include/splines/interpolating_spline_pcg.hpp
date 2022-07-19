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


#ifndef AUTOWARE_CONTROL_TOOLBOX_INCLUDE_SPLINES_INTERPOLATING_SPLINE_PCG_HPP_
#define AUTOWARE_CONTROL_TOOLBOX_INCLUDE_SPLINES_INTERPOLATING_SPLINE_PCG_HPP_

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
// Preconditioned Conjugate Gradients.
class PCG
{
 public:
  PCG() = default;

  /*
   *  Solves A x = b.
   *  diag_lower = diag(A, k=-11) upper diagonal
   *  diag       = diag(A, k=0)
   *  diag_upper = diag(A, k=1) upper diagonal
   * */

  bool solve(Eigen::SparseMatrix<double> const &Asparse,
             Eigen::VectorXd const &b, std::vector<double> &solution_coeffs_c) const;

 private:
  size_t maxiter_{50}; // maximum number of iterations
  double eps_{1e-5}; // convergence threshold

  // Methods.
  [[nodiscard]] bool isConvergedL1(Eigen::VectorXd const &residuals) const;

  static double pApnorm(Eigen::MatrixXd const &p);

};

class InterpolatingSplinePCG
{
 public:

  InterpolatingSplinePCG() = default;

  explicit InterpolatingSplinePCG(size_t interpolating_type);


  // Interpolation type 1 for line 3 for spline.
  /*
   * Interpolate a vector at the given coordinates given a base coordinates and base vector.
   *
   * @param tbase : base coordinates
   * @param ybase : base vector
   * @param tnew : new coordinates
   * @param ynew : new vectors to be computed.
   * @param reuse_coeffs: use the same coefficients previously computed.
   * */

  // For vector interpolation.

  [[nodiscard]] bool Interpolate(std::vector<double> const &tbase,
                                 std::vector<double> const &ybase,
                                 std::vector<double> const &tnew,
                                 std::vector<double> &ynew);

  // For scalar point interpolation.

  bool Interpolate(std::vector<double> const &tbase,
                   std::vector<double> const &ybase,
                   double const &tnew,
                   double &ynew);

  bool Interpolate(double const &tnew, double &ynew) const;

  bool Interpolate(std::vector<double> const &tnew, std::vector<double> &ynew) const;

  // Initialize to eliminate settings frequently reusing var.

  bool Initialize(std::vector<double> const &tbase, std::vector<double> const &ybase);

 private:
  size_t interp_type_{3}; // 1 or 3 for linear and spline interpolation, respectively.
  PCG pcg_{}; // Preconditioned Conjugate Gradient object.

  // base coordinate and base data container.
  std::vector<double> tbase_{};  // base coordinate, assigned when re-used.
  std::vector<double> ybase_{};  // base data, assigned when re-used.

  std::vector<std::vector<double>> coefficients_; // if line [a, b] and spline [a, b, c, d]
  bool initialized_{false};

  // METHODS.
  static bool checkIfMonotonic(const std::vector<double> &tbase);

  // Checks if the coefficients to be reused, checks monotonicity.
  bool prepareCoefficients(std::vector<double> const &tbase,
                           std::vector<double> const &ybase);

  bool compute_coefficients(std::vector<double> const &ybase); // Computes coefficients for the given base data.

  // set remaining coeffs b and d
  void set_bd_coeffs_(std::vector<double> const &ydata, std::vector<double> const &c_coeffs);

  // Polynomial evaluation function.
  static double evaluatePolynomial(double ti, std::vector<double> coeffs);

};

} // namespace ns_splines

#endif //AUTOWARE_CONTROL_TOOLBOX_INCLUDE_SPLINES_INTERPOLATING_SPLINE_PCG_HPP_
