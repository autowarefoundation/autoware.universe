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

/**
 * @file qp_solver_qpoases.h
 * @brief qp solver with QPOASES
 * @author Takamasa Horibe, Yutaka Shimizu
 * @date 2019.05.01
 */

#pragma once

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <iostream>
#include <qpoases_vendor/qpOASES.hpp>

#include "mpc_follower/qp_solver/qp_solver_interface.h"

class QPSolverQpoasesHotstart : public QPSolverInterface
{
private:
  bool is_solver_initialized_;  //!< @brief flag to check initialization
  const int max_iter_;          //!< @brief max iteration number
  qpOASES::SQProblem solver_;   //!< @brief solver for QP

public:
  /**
   * @brief constructor
   * @param max_iter max iteration for QP
   */
  QPSolverQpoasesHotstart(const int max_iter);

  /**
   * @brief destructor
   */
  ~QPSolverQpoasesHotstart() = default;

  /**
   * @brief solve QP problem : minimize J = U' * Hmat * U + fvec' * U with constraint
   * @param [in] Hmat parameter matrix in object function
   * @param [in] fvec parameter matrix in object function
   * @param [in] A parameter matrix for constraint lbA < A*U < ubA
   * @param [in] lb parameter matrix for constraint lb < U < ub
   * @param [in] up parameter matrix for constraint lb < U < ub
   * @param [in] lbA parameter matrix for constraint lbA < A*U < ubA
   * @param [in] ubA parameter matrix for constraint lbA < A*U < ubA
   * @param [out] U optimal variable vector
   * @return bool to check the problem is solved
   */
  bool solve(
    const Eigen::MatrixXd & Hmat, const Eigen::MatrixXd & fvec, const Eigen::MatrixXd & A,
    const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lbA,
    const Eigen::VectorXd & ubA, Eigen::VectorXd & U) override;
};