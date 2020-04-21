/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
#include "mpc_follower/qp_solver/qp_solver_osqp.h"
#include <ros/ros.h>

QPSolverOSQP::QPSolverOSQP() {}
bool QPSolverOSQP::solve(
  const Eigen::MatrixXd & Hmat, const Eigen::MatrixXd & fvec, const Eigen::MatrixXd & A,
  const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lbA,
  const Eigen::VectorXd & ubA, Eigen::VectorXd & U)
{
  const int raw_a = A.rows();
  const int col_a = A.cols();
  const int DIM_U = ub.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(DIM_U, DIM_U);

  // convert matrix to vector for osqpsolver
  std::vector<double> f(&fvec(0), fvec.data() + fvec.cols() * fvec.rows());

  std::vector<double> lower_bound;
  std::vector<double> upper_bound;

  for (int i = 0; i < DIM_U; ++i) {
    lower_bound.push_back(lb(i));
    upper_bound.push_back(ub(i));
  }

  for (int i = 0; i < col_a; ++i) {
    lower_bound.push_back(lbA(i));
    upper_bound.push_back(ubA(i));
  }

  Eigen::MatrixXd osqpA = Eigen::MatrixXd(DIM_U + col_a, raw_a);
  osqpA << I, A;

  /* execute optimization */
  auto result = osqpsolver.optimize(Hmat, osqpA, f, lower_bound, upper_bound);

  std::vector<double> U_osqp = std::get<0>(result);
  U = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1> >(&U_osqp[0], U_osqp.size(), 1);

  // polish status: successful (1), unperformed (0), (-1) unsuccessful
  int status_polish = std::get<2>(result);
  if (status_polish == -1) {
    ROS_WARN("osqp status_polish = %d (unsuccessful)", status_polish);
    return false;
  }
  if (status_polish == 0) {
    ROS_WARN("osqp status_polish = %d (unperformed)", status_polish);
    return true;
  }
  return true;
}
