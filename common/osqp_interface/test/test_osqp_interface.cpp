// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <tuple>
#include <vector>

#include "eigen3/Eigen/Core"
#include "gtest/gtest.h"
#include "osqp_interface/osqp_interface.hpp"


namespace
{
using autoware::common::osqp::float64_t;
// Problem taken from https://github.com/osqp/osqp/blob/master/tests/basic_qp/generate_problem.py
//
// min  1/2 * x' * P * x  + q' * x
// s.t. lb <= A * x <= ub
//
// P = [4, 1], q = [1], A = [1, 1], lb = [   1], ub = [1.0]
//     [1, 2]      [1]      [1, 0]       [   0]       [0.7]
//                          [0, 1]       [   0]       [0.7]
//                          [0, 1]       [-inf]       [inf]
//
// The optimal solution is
// x = [0.3, 0.7]'
// y = [-2.9, 0.0, 0.2, 0.0]`
// obj = 1.88

// cppcheck-suppress syntaxError
TEST(TestOsqpInterface, BasicQp) {
  using autoware::common::osqp::CSC_Matrix;
  using autoware::common::osqp::calCSCMatrix;
  using autoware::common::osqp::calCSCMatrixTrapezoidal;

  auto check_result =
    [](const std::tuple<std::vector<float64_t>, std::vector<float64_t>, int, int, int> & result) {
      EXPECT_EQ(std::get<2>(result), 1);  // polish succeeded
      EXPECT_EQ(std::get<3>(result), 1);  // solution succeeded

      static const auto ep = 1.0e-8;

      const auto prime_val = std::get<0>(result);
      ASSERT_EQ(prime_val.size(), size_t(2));
      EXPECT_NEAR(prime_val[0], 0.3, ep);
      EXPECT_NEAR(prime_val[1], 0.7, ep);

      const auto dual_val = std::get<1>(result);
      ASSERT_EQ(dual_val.size(), size_t(4));
      EXPECT_NEAR(dual_val[0], -2.9, ep);
      EXPECT_NEAR(dual_val[1], 0.0, ep);
      EXPECT_NEAR(dual_val[2], 0.2, ep);
      EXPECT_NEAR(dual_val[3], 0.0, ep);
    };

  {
    // Define problem during optimization
    autoware::common::osqp::OSQPInterface osqp;
    Eigen::MatrixXd P(2, 2);
    P << 4, 1, 1, 2;
    Eigen::MatrixXd A(4, 2);
    A << 1, 1, 1, 0, 0, 1, 0, 1;
    std::vector<float64_t> q = {1.0, 1.0};
    std::vector<float64_t> l = {1.0, 0.0, 0.0, -autoware::common::osqp::INF};
    std::vector<float64_t> u = {1.0, 0.7, 0.7, autoware::common::osqp::INF};
    std::tuple<std::vector<float64_t>, std::vector<float64_t>, int, int, int> result = osqp.optimize(
      P, A, q, l, u);
    check_result(result);
  }
  {
    // Define problem during initialization
    Eigen::MatrixXd P(2, 2);
    P << 4, 1, 1, 2;
    Eigen::MatrixXd A(4, 2);
    A << 1, 1, 1, 0, 0, 1, 0, 1;
    std::vector<float64_t> q = {1.0, 1.0};
    std::vector<float64_t> l = {1.0, 0.0, 0.0, -autoware::common::osqp::INF};
    std::vector<float64_t> u = {1.0, 0.7, 0.7, autoware::common::osqp::INF};
    autoware::common::osqp::OSQPInterface osqp(P, A, q, l, u, 1e-6);
    std::tuple<std::vector<float64_t>, std::vector<float64_t>, int, int, int> result = osqp.optimize();
    check_result(result);
  }
  {
    std::tuple<std::vector<float64_t>, std::vector<float64_t>, int, int, int> result;
    // Dummy initial problem
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 2);
    std::vector<float64_t> q(2, 0.0);
    std::vector<float64_t> l(4, 0.0);
    std::vector<float64_t> u(4, 0.0);
    autoware::common::osqp::OSQPInterface osqp(P, A, q, l, u, 1e-6);
    osqp.optimize();
    // Redefine problem before optimization
    Eigen::MatrixXd P_new(2, 2);
    P_new << 4, 1, 1, 2;
    Eigen::MatrixXd A_new(4, 2);
    A_new << 1, 1, 1, 0, 0, 1, 0, 1;
    std::vector<float64_t> q_new = {1.0, 1.0};
    std::vector<float64_t> l_new = {1.0, 0.0, 0.0, -autoware::common::osqp::INF};
    std::vector<float64_t> u_new = {1.0, 0.7, 0.7, autoware::common::osqp::INF};
    osqp.initializeProblem(P_new, A_new, q_new, l_new, u_new);
    result = osqp.optimize();
    check_result(result);
  }
  {
    // Define problem during initialization with csc matrix
    Eigen::MatrixXd P(2, 2);
    P << 4, 1, 1, 2;
    CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
    Eigen::MatrixXd A(4, 2);
    A << 1, 1, 1, 0, 0, 1, 0, 1;
    CSC_Matrix A_csc = calCSCMatrix(A);
    std::vector<float64_t> q = {1.0, 1.0};
    std::vector<float64_t> l = {1.0, 0.0, 0.0, -autoware::common::osqp::INF};
    std::vector<float64_t> u = {1.0, 0.7, 0.7, autoware::common::osqp::INF};
    autoware::common::osqp::OSQPInterface osqp(P_csc, A_csc, q, l, u, 1e-6);
    std::tuple<std::vector<float64_t>, std::vector<float64_t>, int, int, int> result = osqp.optimize();
    check_result(result);
  }
  {
    std::tuple<std::vector<float64_t>, std::vector<float64_t>, int, int, int> result;
    // Dummy initial problem with csc matrix
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2, 2);
    CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 2);
    CSC_Matrix A_csc = calCSCMatrix(A);
    std::vector<float64_t> q(2, 0.0);
    std::vector<float64_t> l(4, 0.0);
    std::vector<float64_t> u(4, 0.0);
    autoware::common::osqp::OSQPInterface osqp(P_csc, A_csc, q, l, u, 1e-6);
    osqp.optimize();
    // Redefine problem before optimization
    Eigen::MatrixXd P_new(2, 2);
    P_new << 4, 1, 1, 2;
    CSC_Matrix P_new_csc = calCSCMatrixTrapezoidal(P_new);
    Eigen::MatrixXd A_new(4, 2);
    A_new << 1, 1, 1, 0, 0, 1, 0, 1;
    CSC_Matrix A_new_csc = calCSCMatrix(A_new);
    std::vector<float64_t> q_new = {1.0, 1.0};
    std::vector<float64_t> l_new = {1.0, 0.0, 0.0, -autoware::common::osqp::INF};
    std::vector<float64_t> u_new = {1.0, 0.7, 0.7, autoware::common::osqp::INF};
    osqp.initializeProblem(P_new_csc, A_new_csc, q_new, l_new, u_new);
    result = osqp.optimize();
    check_result(result);
  }
}
}  // namespace
