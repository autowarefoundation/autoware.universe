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

#include "mpc_follower/qp_solver/qp_solver_qpoases.h"

QPSolverQpoasesHotstart::QPSolverQpoasesHotstart(const int max_iter)
: is_solver_initialized_(false), max_iter_(max_iter)
{
}

bool QPSolverQpoasesHotstart::solve(
  const Eigen::MatrixXd & Hmat, const Eigen::MatrixXd & fvec, const Eigen::MatrixXd & A,
  const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lbA,
  const Eigen::VectorXd & ubA, Eigen::VectorXd & U)
{
  int max_iter = max_iter_;  // redeclaration to give a non-const value to solver

  const int kNumOfMatrixElements = Hmat.rows() * Hmat.cols();
  double h_matrix[kNumOfMatrixElements];

  const int kNumOfOffsetRows = fvec.rows();
  double g_matrix[kNumOfOffsetRows];

  double lower_bound[kNumOfOffsetRows];
  double upper_bound[kNumOfOffsetRows];

  double result[kNumOfOffsetRows];
  U = Eigen::VectorXd::Zero(kNumOfOffsetRows);

  Eigen::MatrixXd Aconstraint = Eigen::MatrixXd::Identity(kNumOfOffsetRows, kNumOfOffsetRows);
  double a_constraint_matirx[kNumOfMatrixElements];

  int index = 0;

  for (int r = 0; r < Hmat.rows(); ++r) {
    g_matrix[r] = fvec(r, 0);
    for (int c = 0; c < Hmat.cols(); ++c) {
      h_matrix[index] = Hmat(r, c);
      a_constraint_matirx[index] = Aconstraint(r, c);
      index++;
    }
  }

  for (int i = 0; i < kNumOfOffsetRows; ++i) {
    lower_bound[i] = lb[i];
    upper_bound[i] = ub[i];
  }

  solver_.setPrintLevel(
    qpOASES::PL_NONE);  // options: PL_DEBUG_ITER, PL_TABULAR, PL_NONE, PL_LOW, PL_MEDIUM, PL_HIGH

  if (!is_solver_initialized_) {
    solver_ = qpOASES::SQProblem(kNumOfOffsetRows, kNumOfOffsetRows);
    auto ret = solver_.init(
      h_matrix, g_matrix, a_constraint_matirx, lower_bound, upper_bound, lower_bound, upper_bound,
      max_iter);
    if (ret != qpOASES::SUCCESSFUL_RETURN) {
      std::cerr << "[QPOASES] not successfully solved in init()" << std::endl;
      return false;
    }

    is_solver_initialized_ = true;
  } else {
    auto ret = solver_.hotstart(
      h_matrix, g_matrix, a_constraint_matirx, lower_bound, upper_bound, lower_bound, upper_bound,
      max_iter);
    if (ret != qpOASES::SUCCESSFUL_RETURN) {
      std::cerr << "[QPOASES] not successfully solved in hotstart()" << std::endl;
      return false;
    }
  }

  solver_.getPrimalSolution(result);

  for (int i = 0; i < kNumOfOffsetRows; ++i) {
    U(i) = result[i];
  }

  return true;
}
