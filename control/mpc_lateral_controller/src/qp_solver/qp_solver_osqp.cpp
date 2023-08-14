// Copyright 2018-2021 The Autoware Foundation
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

#include "mpc_lateral_controller/qp_solver/qp_solver_osqp.hpp"

#include <string>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{
QPSolverOSQP::QPSolverOSQP(const rclcpp::Logger & logger) : logger_{logger}
{
}
bool QPSolverOSQP::solve(
  const Eigen::MatrixXd & h_mat, const Eigen::MatrixXd & f_vec, const Eigen::MatrixXd & a,
  const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lb_a,
  const Eigen::VectorXd & ub_a, Eigen::VectorXd & u)
{
  const Eigen::Index raw_a = a.rows();
  const Eigen::Index col_a = a.cols();
  const Eigen::Index dim_u = ub.size();
  Eigen::MatrixXd Identity = Eigen::MatrixXd::Identity(dim_u, dim_u);

  Eigen::VectorXd lower_bound = Eigen::VectorXd::Zero(lb.size() + lb_a.size());
  Eigen::VectorXd upper_bound = Eigen::VectorXd::Zero(lb.size() + lb_a.size());

  lower_bound.segment(0, lb.size()) = lb;
  lower_bound.segment(lb.size(), lb_a.size()) = lb_a;

  upper_bound.segment(0, ub.size()) = ub;
  upper_bound.segment(ub.size(), ub_a.size()) = ub_a;

  Eigen::MatrixXd osqpA = Eigen::MatrixXd(dim_u + raw_a, col_a);
  osqpA.block(0, 0, dim_u, col_a) = Identity;
  osqpA.block(dim_u, 0, raw_a, col_a) = a;

  return solve(h_mat, f_vec, osqpA, lower_bound, upper_bound, u);
}

bool QPSolverOSQP::solve(
  const Eigen::MatrixXd & h_mat, const Eigen::MatrixXd & f_vec, const Eigen::MatrixXd & a,
  const Eigen::VectorXd & lb_a, const Eigen::VectorXd & ub_a, Eigen::VectorXd & u)
{
  // convert matrix to vector for osqpsolver
  std::vector<double> f(&f_vec(0), f_vec.data() + f_vec.cols() * f_vec.rows());

  std::vector<double> lower_bound;
  std::vector<double> upper_bound;
  for (int i = 0; i < lb_a.size(); ++i) {
    lower_bound.push_back(lb_a(i));
    upper_bound.push_back(ub_a(i));
  }

  /* execute optimization */
  auto result = osqpsolver_.optimize(h_mat, a, f, lower_bound, upper_bound);

  std::vector<double> U_osqp = std::get<0>(result);
  u = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(
    &U_osqp[0], static_cast<Eigen::Index>(U_osqp.size()), 1);

  const int status_val = std::get<3>(result);
  if (status_val != 1) {
    RCLCPP_WARN(logger_, "optimization failed : %s", osqpsolver_.getStatusMessage().c_str());
    return false;
  }
  const auto has_nan =
    std::any_of(U_osqp.begin(), U_osqp.end(), [](const auto v) { return std::isnan(v); });
  if (has_nan) {
    RCLCPP_WARN(logger_, "optimization failed: result contains NaN values");
    return false;
  }

  // polish status: successful (1), unperformed (0), (-1) unsuccessful
  int status_polish = std::get<2>(result);
  if (status_polish == -1) {
    RCLCPP_WARN(logger_, "osqp status_polish = %d (unsuccessful)", status_polish);
    return false;
  }
  if (status_polish == 0) {
    RCLCPP_WARN(logger_, "osqp status_polish = %d (unperformed)", status_polish);
    return true;
  }
  return true;
}
}  // namespace autoware::motion::control::mpc_lateral_controller
