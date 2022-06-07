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

#ifndef AUTOWARE_CONTROL_TOOLBOX_STATE_SPACE_HPP
#define AUTOWARE_CONTROL_TOOLBOX_STATE_SPACE_HPP

#include "act_definitions.hpp"
#include "act_utils.hpp"
#include "act_utils_eigen.hpp"
#include "balance.hpp"
#include "transfer_functions.hpp"

#include <algorithm>
#include <vector>

namespace ns_control_toolbox
{

/**
 * @brief tf2ss Converts a transfer function representation in to a state-space form.
 * We assume the system is SISO type.
 *
 * */

class tf2ss
{
public:
  // Constructors
  tf2ss() = default;

  explicit tf2ss(tf const & sys_tf, const double & Ts = 0.1);

  tf2ss(std::vector<double> const & num, std::vector<double> const & den, const double & Ts = 0.1);

  // Public methods
  // Currently only Tustin - Bilinear discretization is implemented.
  void discretisize(double const & Ts);

  // Update state-space once constructed and in case of a parameter change.
  void updateStateSpace(tf const & sys_tf);

  // Getters for the system matrices.
  // Discrete time state-space matrices.
  [[nodiscard]] Eigen::MatrixXd Ad() const;

  [[nodiscard]] Eigen::MatrixXd Bd() const;

  [[nodiscard]] Eigen::MatrixXd Cd() const;

  [[nodiscard]] Eigen::MatrixXd Dd() const;

  // Continuous time state-space matrices.
  [[nodiscard]] Eigen::MatrixXd A() const;

  [[nodiscard]] Eigen::MatrixXd B() const;

  [[nodiscard]] Eigen::MatrixXd C() const;

  [[nodiscard]] Eigen::MatrixXd D() const;

  [[nodiscard]] Eigen::MatrixXd T() const;  // Similarity transformation matrix T^inv * A * T

  /**
   * @brief simulated the discrete system matrices [Ad, Bd:Cd, Dd] for one step. Its state matrix as
   * an input is a column matrix [x;u]. This state matrix returns as [x; y] which is in the form of
   * xy = [A B;C D]xu.
   * */

  double simulateOneStep(Eigen::MatrixXd & x0, double const & u) const;

  /**
   * @brief Compute the system continuous time system matrices
   * */
  void computeSystemMatrices(std::vector<double> const & num, std::vector<double> const & den);

  void print() const;

  void print_discrete_system() const;

private:
  double Ts_{};
  Eigen::Index N_{};  // system size (A.rows+1).

  // Data members

  // system matrices  in a single matrix form of [A, B;C, D]
  /**
   *	A_ = ss_system.topLeftCorner(nx, nx);
   *	B_ = ss_system.topRightCorner(nx, 1);
   *	C_ = ss_system.bottomLeftCorner(1, nx);
   *	D_ = ss_system.bottomRightCorner(1, 1);
   * */
  Eigen::MatrixXd Tsimilarity_mat_{};  // Similarity mat of A, Aprime = Tinv * A *T.

  // Data members
  // Continuous time state-space model
  Eigen::MatrixXd A_{};
  Eigen::MatrixXd B_{};
  Eigen::MatrixXd C_{};
  Eigen::MatrixXd D_{};

  // Discrete time state-space model
  Eigen::MatrixXd Ad_{};
  Eigen::MatrixXd Bd_{};
  Eigen::MatrixXd Cd_{};
  Eigen::MatrixXd Dd_{};
};

// Type definitions.
template <int nx, int ny>
using mat_type_t = Eigen::Matrix<double, nx, ny>;

template <int N>
using state_type_t = Eigen::Matrix<double, N, 1>;

}  // namespace ns_control_toolbox
#endif  // AUTOWARE_CONTROL_TOOLBOX_STATE_SPACE_HPP
