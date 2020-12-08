/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 *
 * Author: Robin Karlsson
 */
#include <chrono>
#include <iostream>
#include <vector>

#include "osqp.h"
#include "osqp_interface/csc_matrix_conv.hpp"
#include "osqp_interface/osqp_interface.hpp"

namespace osqp
{
OSQPInterface::OSQPInterface(const c_float eps_abs, const bool polish)
{
  /************************
   * INITIALIZE WORKSPACE
   ************************/
  settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  data = (OSQPData *)c_malloc(sizeof(OSQPData));

  /*******************
   * SOLVER SETTINGS
   *******************/
  if (settings) {
    osqp_set_default_settings(settings);
    settings->alpha = 1.6;  // Change alpha parameter
    settings->eps_rel = 1.0E-4;
    settings->eps_abs = eps_abs;
    settings->eps_prim_inf = 1.0E-4;
    settings->eps_dual_inf = 1.0E-4;
    settings->warm_start = true;
    settings->max_iter = 4000;
    settings->verbose = false;
    settings->polish = polish;
  }
  // Set flag for successful initialization
  exitflag = 0;
}

OSQPInterface::OSQPInterface(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u, const c_float eps_abs)
: OSQPInterface(eps_abs)
{
  initializeProblem(P, A, q, l, u);
}

c_int OSQPInterface::initializeProblem(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u)
{
  /*******************
   * SET UP MATRICES
   *******************/
  CSC_Matrix P_csc = calCSCMatrixTrapesoidal(P);
  CSC_Matrix A_csc = calCSCMatrix(A);
  // Dynamic float arrays
  std::vector<double> q_tmp(q.begin(), q.end());
  std::vector<double> l_tmp(l.begin(), l.end());
  std::vector<double> u_tmp(u.begin(), u.end());
  double * q_dyn = q_tmp.data();
  double * l_dyn = l_tmp.data();
  double * u_dyn = u_tmp.data();

  /**********************
   * OBJECTIVE FUNCTION
   **********************/
  // Number of constraints
  c_int constr_m = A.rows();
  // Number of parameters
  param_n = P.rows();

  /*****************
   * POLULATE DATA
   *****************/
  data->m = constr_m;
  data->n = param_n;
  data->P = csc_matrix(
    data->n, data->n, P_csc.vals.size(), P_csc.vals.data(), P_csc.row_idxs.data(),
    P_csc.col_idxs.data());
  data->q = q_dyn;
  data->A = csc_matrix(
    data->m, data->n, A_csc.vals.size(), A_csc.vals.data(), A_csc.row_idxs.data(),
    A_csc.col_idxs.data());
  data->l = l_dyn;
  data->u = u_dyn;

  // For deconstructor
  problem_in_memory = true;

  // Setup workspace
  exitflag = osqp_setup(&work, data, settings);

  return exitflag;
}

OSQPInterface::~OSQPInterface()
{
  // Cleanup dynamic OSQP memory
  if (work) {
    osqp_cleanup(work);
  }
  if (data) {
    if (problem_in_memory) {
      c_free(data->A);
      c_free(data->P);
    }
    c_free(data);
  }
  if (settings) {c_free(settings);}
}

void OSQPInterface::updateP(const Eigen::MatrixXd & P_new)
{
  /*
  // Transform 'P' into an 'upper trapesoidal matrix'
  Eigen::MatrixXd P_trap = P_new.triangularView<Eigen::Upper>();
  // Transform 'P' into a sparse matrix and extract data as dynamic arrays
  Eigen::SparseMatrix<double> P_sparse = P_trap.sparseView();
  double *P_val_ptr = P_sparse.valuePtr();
  // Convert dynamic 'int' arrays to 'c_int' arrays (OSQP input type)
  c_int P_elem_N = P_sparse.nonZeros();
  */
  CSC_Matrix P_csc = calCSCMatrixTrapesoidal(P_new);
  osqp_update_P(work, P_csc.vals.data(), OSQP_NULL, P_csc.vals.size());
}

void OSQPInterface::updateA(const Eigen::MatrixXd & A_new)
{
  /*
  // Transform 'A' into a sparse matrix and extract data as dynamic arrays
  Eigen::SparseMatrix<double> A_sparse = A_new.sparseView();
  double *A_val_ptr = A_sparse.valuePtr();
  // Convert dynamic 'int' arrays to 'c_int' arrays (OSQP input type)
  c_int A_elem_N = A_sparse.nonZeros();
  */
  CSC_Matrix A_csc = calCSCMatrix(A_new);
  osqp_update_A(work, A_csc.vals.data(), OSQP_NULL, A_csc.vals.size());
}

void OSQPInterface::updateQ(const std::vector<double> & q_new)
{
  std::vector<double> q_tmp(q_new.begin(), q_new.end());
  double * q_dyn = q_tmp.data();
  osqp_update_lin_cost(work, q_dyn);
}

void OSQPInterface::updateL(const std::vector<double> & l_new)
{
  std::vector<double> l_tmp(l_new.begin(), l_new.end());
  double * l_dyn = l_tmp.data();
  osqp_update_lower_bound(work, l_dyn);
}

void OSQPInterface::updateU(const std::vector<double> & u_new)
{
  std::vector<double> u_tmp(u_new.begin(), u_new.end());
  double * u_dyn = u_tmp.data();
  osqp_update_upper_bound(work, u_dyn);
}

void OSQPInterface::updateBounds(
  const std::vector<double> & l_new, const std::vector<double> & u_new)
{
  std::vector<double> l_tmp(l_new.begin(), l_new.end());
  std::vector<double> u_tmp(u_new.begin(), u_new.end());
  double * l_dyn = l_tmp.data();
  double * u_dyn = u_tmp.data();
  osqp_update_bounds(work, l_dyn, u_dyn);
}

void OSQPInterface::updateEpsAbs(const double eps_abs) {osqp_update_eps_abs(work, eps_abs);}

void OSQPInterface::updateEpsRel(const double eps_rel) {osqp_update_eps_rel(work, eps_rel);}

void OSQPInterface::updateMaxIter(const int max_iter) {osqp_update_max_iter(work, max_iter);}

void OSQPInterface::updateVerbose(const bool is_verbose) {osqp_update_verbose(work, is_verbose);}

int OSQPInterface::getTakenIter() {return work->info->iter;}

std::tuple<std::vector<double>, std::vector<double>, int, int> OSQPInterface::solve()
{
  // Solve Problem
  osqp_solve(work);

  /********************
   * EXTRACT SOLUTION
   ********************/
  double * sol_x = work->solution->x;
  double * sol_y = work->solution->y;
  std::vector<double> sol_primal(sol_x, sol_x + static_cast<int>(param_n));
  std::vector<double> sol_lagrange_multiplier(sol_y, sol_y + static_cast<int>(param_n));
  // Solver polish status
  int status_polish = work->info->status_polish;
  // Solver solution status
  int status_solution = work->info->status_val;
  // Result tuple
  std::tuple<std::vector<double>, std::vector<double>, int, int> result =
    std::make_tuple(sol_primal, sol_lagrange_multiplier, status_polish, status_solution);

  return result;
}

std::tuple<std::vector<double>, std::vector<double>, int, int> OSQPInterface::optimize()
{
  // Run the solver on the stored problem representation.
  std::tuple<std::vector<double>, std::vector<double>, int, int> result = solve();
  return result;
}

std::tuple<std::vector<double>, std::vector<double>, int, int> OSQPInterface::optimize(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u)
{
  // Allocate memory for problem
  initializeProblem(P, A, q, l, u);

  // Run the solver on the stored problem representation.
  std::tuple<std::vector<double>, std::vector<double>, int, int> result = solve();

  // Free allocated memory for problem
  osqp_cleanup(work);
  return result;
}

inline bool OSQPInterface::isEqual(double x, double y)
{
  const double epsilon = 1e-6;
  return std::abs(x - y) <= epsilon * std::abs(x);
}

c_int OSQPInterface::getExitFlag(void) {return exitflag;}

}  // namespace osqp
