// Copyright 2023 TIER IV, Inc.
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

#include "qp_interface/proxqp_interface.hpp"

#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace qp
{
void ProxQPInterface::initializeProblemImpl(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u)
{
  auto dim = static_cast<int>(q.size());
  auto n_in = static_cast<int>(l.size());

  const bool use_warm_start = false;
  /*
  const bool use_warm_start = [&]() {
    if (!qp_ptr_) return false;
    if (!m_variables_num || static_cast<long>(*m_variables_num) != dim) return false;
    if (!m_constraints_num || static_cast<long>(*m_constraints_num) != n_in) return false;
    return true;
  }();
  */
  std::cerr << use_warm_start << std::endl;

  if (!use_warm_start) {
    qp_ptr_ = std::make_shared<proxsuite::proxqp::sparse::QP<double, int>>(dim, 0, n_in);
  }

  Eigen::SparseMatrix<double> H_spa(dim, n_in);
  H_spa = P.sparseView();
  qp_ptr_->settings.eps_abs = 1e-9;
  qp_ptr_->settings.initial_guess =
    use_warm_start ? proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT
                   : proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
  qp_ptr_->settings.verbose = true;

  std::vector<double> g_std_vec = q;
  Eigen::VectorXd ei_g =
    Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(g_std_vec.data(), g_std_vec.size());
  std::vector<double> l_std_vec = l;
  Eigen::VectorXd ei_l =
    Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(l_std_vec.data(), l_std_vec.size());
  std::vector<double> u_std_vec = u;
  Eigen::VectorXd ei_u =
    Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(u_std_vec.data(), u_std_vec.size());

  if (use_warm_start) {
    qp_ptr_->update(
      H_spa, ei_g, proxsuite::nullopt, proxsuite::nullopt, A.sparseView(), ei_l, ei_u);
  } else {
    qp_ptr_->init(H_spa, ei_g, proxsuite::nullopt, proxsuite::nullopt, A.sparseView(), ei_l, ei_u);
  }
}

void ProxQPInterface::updateEpsAbs(const double eps_abs)
{
  if (qp_ptr_) {
    qp_ptr_->settings.eps_abs = eps_abs;
  }
}

void ProxQPInterface::updateEpsRel(const double eps_rel)
{
  if (qp_ptr_) {
    qp_ptr_->settings.eps_rel = eps_rel;
  }
}

void ProxQPInterface::updateVerbose(const bool is_verbose)
{
  if (qp_ptr_) {
    qp_ptr_->settings.verbose = is_verbose;
  }
}

int ProxQPInterface::getIteration() const
{
  return 0;
}

int ProxQPInterface::getStatus() const
{
  return 0;
}

std::vector<double> ProxQPInterface::optimizeImpl()
{
  qp_ptr_->solve();

  std::vector<double> result;
  for (Eigen::Index i = 0; i < qp_ptr_->results.x.size(); ++i) {
    result.push_back(qp_ptr_->results.x[i]);
  }
  return result;
}
}  // namespace qp
