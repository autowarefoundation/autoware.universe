// Copyright 2021 The Autoware Foundation
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

#include "osqp_interface/osqp_interface.hpp"

#include "osqp/osqp.h"
#include "osqp_interface/csc_matrix_conv.hpp"

#include <glog/logging.h>

#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

static bool check_has_solution(OSQPInfo * info) { return (info->status_val == OSQP_SOLVED); }

namespace autoware
{
namespace common
{
namespace osqp
{
OSQPInterface::OSQPInterface(const c_float eps_abs, const bool polish, const bool warm_start)
: m_work{nullptr, OSQPWorkspaceDeleter}, m_warm_start(warm_start)
{
  OSQPSettings * settings_ptr = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  m_settings = std::unique_ptr<OSQPSettings, decltype(&OSQPSettingsDeleter)>(
    settings_ptr, OSQPSettingsDeleter);
  if (m_settings) {
    osqp_set_default_settings(m_settings.get());
    m_settings->alpha = 1.6;  // Change alpha parameter
    m_settings->eps_rel = 1.0E-4;
    m_settings->eps_abs = eps_abs;
    m_settings->eps_prim_inf = 1.0E-4;
    m_settings->eps_dual_inf = 1.0E-4;
    m_settings->warm_start = warm_start;
    m_settings->max_iter = 4000;
    m_settings->verbose = false;
    m_settings->polish = polish;
  }
  m_exitflag = 0;
}

OSQPInterface::OSQPInterface(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u, const c_float eps_abs,
  const bool polish, const bool warm_start)
: OSQPInterface(eps_abs, polish, warm_start)
{
  initializeProblem(P, A, q, l, u, warm_start);
}

OSQPInterface::OSQPInterface(
  CSC_Matrix && P, CSC_Matrix && A, const std::vector<double> & q, const std::vector<double> & l,
  const std::vector<double> & u, const c_float eps_abs, const bool polish, const bool warm_start)
: OSQPInterface(eps_abs, polish, warm_start)
{
  initializeProblem(std::forward<CSC_Matrix>(P), std::forward<CSC_Matrix>(A), q, l, u, warm_start);
}

void OSQPInterface::OSQPWorkspaceDeleter(OSQPWorkspace * ptr) noexcept
{
  if (ptr) {
    osqp_cleanup(ptr);
  }
}

void OSQPInterface::OSQPSettingsDeleter(OSQPSettings * ptr) noexcept
{
  if (ptr) {
    c_free(ptr);
  }
}

void OSQPInterface::OSQPDataDeleter(OSQPData * ptr) noexcept
{
  if (ptr) {
    if (ptr->A) c_free(ptr->A);
    if (ptr->P) c_free(ptr->P);
    c_free(ptr);
  }
}

void OSQPInterface::updateP(const Eigen::MatrixXd & P_new)
{
  /*
  // Transform 'P' into an 'upper trapezoidal matrix'
  Eigen::MatrixXd P_trap = P_new.triangularView<Eigen::Upper>();
  // Transform 'P' into a sparse matrix and extract data as dynamic arrays
  Eigen::SparseMatrix<double> P_sparse = P_trap.sparseView();
  double *P_val_ptr = P_sparse.valuePtr();
  // Convert dynamic 'int' arrays to 'c_int' arrays (OSQP input type)
  c_int P_elem_N = P_sparse.nonZeros();
  */
  CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P_new);
  osqp_update_P(
    m_work.get(), P_csc.m_vals.data(), OSQP_NULL, static_cast<c_int>(P_csc.m_vals.size()));
}

void OSQPInterface::updateCscP(const CSC_Matrix & P_csc)
{
  osqp_update_P(
    m_work.get(), P_csc.m_vals.data(), OSQP_NULL, static_cast<c_int>(P_csc.m_vals.size()));
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
  osqp_update_A(
    m_work.get(), A_csc.m_vals.data(), OSQP_NULL, static_cast<c_int>(A_csc.m_vals.size()));
  return;
}

void OSQPInterface::updateCscA(const CSC_Matrix & A_csc)
{
  osqp_update_A(
    m_work.get(), A_csc.m_vals.data(), OSQP_NULL, static_cast<c_int>(A_csc.m_vals.size()));
}

void OSQPInterface::updateQ(const std::vector<double> & q_new)
{
  osqp_update_lin_cost(m_work.get(), q_new.data());
}

void OSQPInterface::updateL(const std::vector<double> & l_new)
{
  osqp_update_lower_bound(m_work.get(), l_new.data());
}

void OSQPInterface::updateU(const std::vector<double> & u_new)
{
  osqp_update_upper_bound(m_work.get(), u_new.data());
}

void OSQPInterface::updateBounds(
  const std::vector<double> & l_new, const std::vector<double> & u_new)
{
  osqp_update_bounds(m_work.get(), l_new.data(), u_new.data());
}

void OSQPInterface::updateEpsAbs(const double eps_abs)
{
  m_settings->eps_abs = eps_abs;  // for default setting
  if (m_work) {
    osqp_update_eps_abs(m_work.get(), eps_abs);  // for current work
  }
}

void OSQPInterface::updateEpsRel(const double eps_rel)
{
  m_settings->eps_rel = eps_rel;  // for default setting
  if (m_work) {
    osqp_update_eps_rel(m_work.get(), eps_rel);  // for current work
  }
}

void OSQPInterface::updateMaxIter(const int max_iter)
{
  m_settings->max_iter = max_iter;  // for default setting
  if (m_work) {
    osqp_update_max_iter(m_work.get(), max_iter);  // for current work
  }
}

void OSQPInterface::updateVerbose(const bool is_verbose)
{
  m_settings->verbose = is_verbose;  // for default setting
  if (m_work) {
    osqp_update_verbose(m_work.get(), is_verbose);  // for current work
  }
}

void OSQPInterface::updateRhoInterval(const int rho_interval)
{
  m_settings->adaptive_rho_interval = rho_interval;  // for default setting
}

void OSQPInterface::updateRho(const double rho)
{
  m_settings->rho = rho;
  if (m_work) {
    osqp_update_rho(m_work.get(), rho);
  }
}

void OSQPInterface::updateAlpha(const double alpha)
{
  m_settings->alpha = alpha;
  if (m_work) {
    osqp_update_alpha(m_work.get(), alpha);
  }
}

void OSQPInterface::updateScaling(const int scaling) { m_settings->scaling = scaling; }

void OSQPInterface::updatePolish(const bool polish)
{
  m_settings->polish = polish;
  if (m_work) {
    osqp_update_polish(m_work.get(), polish);
  }
}

void OSQPInterface::updatePolishRefinementIteration(const int polish_refine_iter)
{
  if (polish_refine_iter < 0) {
    std::cerr << "Polish refinement iterations must be positive" << std::endl;
    return;
  }

  m_settings->polish_refine_iter = polish_refine_iter;
  if (m_work) {
    osqp_update_polish_refine_iter(m_work.get(), polish_refine_iter);
  }
}

void OSQPInterface::updateCheckTermination(const int check_termination)
{
  if (check_termination < 0) {
    std::cerr << "Check termination must be positive" << std::endl;
    return;
  }

  m_settings->check_termination = check_termination;
  if (m_work) {
    osqp_update_check_termination(m_work.get(), check_termination);
  }
}

bool OSQPInterface::setWarmStart(
  const std::vector<double> & primal_variables, const std::vector<double> & dual_variables)
{
  return setPrimalVariables(primal_variables) && setDualVariables(dual_variables);
}

bool OSQPInterface::setPrimalVariables(const std::vector<double> & primal_variables)
{
  const auto result = osqp_warm_start_x(m_work.get(), primal_variables.data());
  if (result != 0) {
    std::cerr << "Failed to set primal variables for warm start" << std::endl;
    return false;
  }

  return true;
}

bool OSQPInterface::setDualVariables(const std::vector<double> & dual_variables)
{
  const auto result = osqp_warm_start_y(m_work.get(), dual_variables.data());
  if (result != 0) {
    std::cerr << "Failed to set dual variables for warm start" << std::endl;
    return false;
  }

  return true;
}

int64_t OSQPInterface::initializeProblem(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u, const bool do_warm_start)
{
  // check if arguments are valid
  std::stringstream ss;
  if (P.rows() != P.cols()) {
    ss << "P.rows() and P.cols() are not the same. P.rows() = " << P.rows()
       << ", P.cols() = " << P.cols();
    throw std::invalid_argument(ss.str());
  }
  if (P.rows() != static_cast<int>(q.size())) {
    ss << "P.rows() and q.size() are not the same. P.rows() = " << P.rows()
       << ", q.size() = " << q.size();
    throw std::invalid_argument(ss.str());
  }
  if (P.rows() != A.cols()) {
    ss << "P.rows() and A.cols() are not the same. P.rows() = " << P.rows()
       << ", A.cols() = " << A.cols();
    throw std::invalid_argument(ss.str());
  }
  if (A.rows() != static_cast<int>(l.size())) {
    ss << "A.rows() and l.size() are not the same. A.rows() = " << A.rows()
       << ", l.size() = " << l.size();
    throw std::invalid_argument(ss.str());
  }
  if (A.rows() != static_cast<int>(u.size())) {
    ss << "A.rows() and u.size() are not the same. A.rows() = " << A.rows()
       << ", u.size() = " << u.size();
    throw std::invalid_argument(ss.str());
  }

  CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
  CSC_Matrix A_csc = calCSCMatrix(A);
  return initializeProblem(std::move(P_csc), std::move(A_csc), q, l, u, do_warm_start);
}

int64_t OSQPInterface::initializeProblem(
  CSC_Matrix && P_csc, CSC_Matrix && A_csc, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u, const bool do_warm_start)
{
  m_param_n = static_cast<int>(q.size());
  m_param_m = static_cast<int>(l.size());

  // Setup workspace
  if (do_warm_start && warmStartReady()) {
    // if (1) warm_start is enabled AND
    // (2) problem size is invariant AND
    // (3) previous solution is valid
    updateCscP(std::forward<CSC_Matrix>(P_csc));
    updateCscA(std::forward<CSC_Matrix>(A_csc));
    updateQ(q);
    updateBounds(l, u);
    return true;
  } else {
    // (1) this is the first initialization OR
    // (2) warm_start is disabled OR
    // (3) dimension changed from previous OR
    // (4) previous solution was invalid
    OSQPWorkspace * workspace;
    /*****************
     * POPULATE DATA
     *****************/
    OSQPData * data_ptr = (OSQPData *)c_malloc(sizeof(OSQPData));
    auto data = std::unique_ptr<OSQPData, decltype(&OSQPDataDeleter)>(data_ptr, OSQPDataDeleter);
    // size
    data->n = m_param_n;
    data->m = m_param_m;
    // P
    data->P = csc_matrix(
      data->n, data->n, static_cast<c_int>(P_csc.m_vals.size()), P_csc.m_vals.data(),
      P_csc.m_row_idxs.data(), P_csc.m_col_idxs.data());
    // q
    data->q = const_cast<double *>(q.data());
    // A
    data->A = csc_matrix(
      data->m, data->n, static_cast<c_int>(A_csc.m_vals.size()), A_csc.m_vals.data(),
      A_csc.m_row_idxs.data(), A_csc.m_col_idxs.data());
    // bounds
    data->l = const_cast<double *>(l.data());
    data->u = const_cast<double *>(u.data());
    // NOTE(Mamoru Sobue): osqp_setup() copies everything from `data` and `settings`
    // so `q, l, u` can be mutably referenced and `data` can expire here
    m_exitflag = osqp_setup(&workspace, data.get(), m_settings.get());
    m_work.reset(workspace);
    return m_exitflag;
  }
}

bool OSQPInterface::warmStartReady() const
{
  return m_param_n_prev && m_param_n_prev.value() == m_param_n &&  // dim of m
         m_param_m_prev && m_param_m_prev.value() == m_param_m &&  // dim of n
         m_sol_prev && m_lagrange_multiplier_prev;
}

std::tuple<std::vector<double>, std::vector<double>, int64_t, int64_t, int64_t>
OSQPInterface::solve(const bool do_warm_start)
{
  if (m_warm_start && do_warm_start && warmStartReady()) {
    setWarmStart(m_sol_prev.value(), m_lagrange_multiplier_prev.value());
  }
  // Solve Problem
  osqp_solve(m_work.get());

  /********************
   * EXTRACT SOLUTION
   ********************/
  double * sol_x = m_work->solution->x;
  double * sol_y = m_work->solution->y;
  const std::vector<double> sol_primal(sol_x, sol_x + m_param_n);
  const std::vector<double> sol_lagrange_multiplier(sol_y, sol_y + m_param_m);

  const int64_t status_polish = m_work->info->status_polish;
  const auto status_val = m_work->info->status_val;
  const bool has_solution = check_has_solution(m_work->info);
  const int64_t status_iteration = m_work->info->iter;

  std::cout << "has_solution = " << has_solution << std::endl;
  std::cout << "iteration = " << status_iteration << std::endl;

  // save solution
  m_param_n_prev = m_param_n;
  m_param_m_prev = m_param_m;
  m_latest_work_info = *(m_work->info);
  // do not save invalid solution for warm-start
  if (m_warm_start && has_solution && status_polish == 1) {
    m_sol_prev = sol_primal;
    m_lagrange_multiplier_prev = sol_lagrange_multiplier;
  } else {
    m_sol_prev = std::nullopt;
    m_lagrange_multiplier_prev = std::nullopt;
    m_work.reset();
  }

  return {sol_primal, sol_lagrange_multiplier, status_polish, status_val, status_iteration};
}

std::tuple<std::vector<double>, std::vector<double>, int64_t, int64_t, int64_t>
OSQPInterface::optimize(const bool do_warm_start)
{
  // Run the solver on the stored problem representation.
  return solve(m_warm_start && do_warm_start);
}

std::tuple<std::vector<double>, std::vector<double>, int64_t, int64_t, int64_t>
OSQPInterface::optimize(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u, const bool do_warm_start)
{
  // Allocate memory for problem
  initializeProblem(P, A, q, l, u, m_warm_start && do_warm_start);

  // Run the solver on the stored problem representation.
  return solve(m_warm_start && do_warm_start);
}

void OSQPInterface::logUnsolvedStatus(const std::string & prefix_message) const
{
  const int status = getStatus();
  if (status == 1) {
    // No need to log since optimization was solved.
    return;
  }

  // create message
  std::string output_message = "";
  if (prefix_message != "") {
    output_message = prefix_message + " ";
  }

  const auto status_message = getStatusMessage();
  output_message += "Optimization failed due to " + status_message;

  // log with warning
  RCLCPP_WARN(rclcpp::get_logger("osqp_interface"), output_message.c_str());
}
}  // namespace osqp
}  // namespace common
}  // namespace autoware
