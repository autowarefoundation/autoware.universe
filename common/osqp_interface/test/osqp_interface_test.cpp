// Copyright 2021 Tier IV, Inc.
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

#include <vector>

#include "gtest/gtest.h"

#include "osqp_interface/osqp_interface.hpp"

constexpr double tolerance = 0.01;

void generateOSQPSolver()
{
  osqp::OSQPInterface solver;
}

TEST(OSQPInterface, Instance)
{
  EXPECT_NO_THROW(generateOSQPSolver());
}

TEST(OSQPInterface, Solve)
{
  // minimize x'x + [1 1]x
  // subject to 0 <= x <= 1
  // The answer is expected to [0 0]'
  osqp::OSQPInterface solver;
  constexpr int num_vars = 2;
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(num_vars, num_vars);
  std::vector<double> q(num_vars, 1.0);
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_vars, num_vars);
  std::vector<double> l(num_vars, 0.0);
  std::vector<double> u(num_vars, 1.0);
  const auto result = solver.optimize(P, A, q, l, u);
  std::vector<double> x_optimal = std::get<0>(result);
  EXPECT_LE(std::fabs(x_optimal[0]), tolerance);
  EXPECT_LE(std::fabs(x_optimal[1]), tolerance);
}

TEST(OSQPInterface, Exception)
{
  constexpr int num_vars = 2;
  c_float eps_abs = 0.1;

  {
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(num_vars, num_vars + 1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_vars, num_vars);
    std::vector<double> q(num_vars, 1.0);
    std::vector<double> l(num_vars, 0.0);
    std::vector<double> u(num_vars, 1.0);
    EXPECT_THROW(osqp::OSQPInterface solver(P, A, q, l, u, eps_abs), std::invalid_argument);
  }

  {
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(num_vars, num_vars);
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_vars, num_vars);
    std::vector<double> q(num_vars + 1, 1.0);
    std::vector<double> l(num_vars, 0.0);
    std::vector<double> u(num_vars, 1.0);
    EXPECT_THROW(osqp::OSQPInterface solver(P, A, q, l, u, eps_abs), std::invalid_argument);
  }

  {
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(num_vars, num_vars);
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_vars, num_vars + 1);
    std::vector<double> q(num_vars, 1.0);
    std::vector<double> l(num_vars, 0.0);
    std::vector<double> u(num_vars, 1.0);
    EXPECT_THROW(osqp::OSQPInterface solver(P, A, q, l, u, eps_abs), std::invalid_argument);
  }

  {
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(num_vars, num_vars);
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_vars, num_vars);
    std::vector<double> q(num_vars, 1.0);
    std::vector<double> l(num_vars + 1, 0.0);
    std::vector<double> u(num_vars, 1.0);
    EXPECT_THROW(osqp::OSQPInterface solver(P, A, q, l, u, eps_abs), std::invalid_argument);
  }

  {
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(num_vars, num_vars);
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_vars, num_vars);
    std::vector<double> q(num_vars, 1.0);
    std::vector<double> l(num_vars, 0.0);
    std::vector<double> u(num_vars + 1, 1.0);
    EXPECT_THROW(osqp::OSQPInterface solver(P, A, q, l, u, eps_abs), std::invalid_argument);
  }
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
