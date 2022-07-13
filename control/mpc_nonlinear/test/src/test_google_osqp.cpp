// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iterator>
#include <limits>
#include <vector>
#include "Eigen/SparseCore"
#include "gtest/gtest.h"
#include "osqp_google/osqp++.hpp"
#include "utils/nmpc_utils.hpp"

namespace osqp
{
namespace
{
constexpr double kTolerance = 1e-5;
constexpr double kInfinity = std::numeric_limits<double>::infinity();
using ::Eigen::SparseMatrix;
using ::Eigen::Triplet;
using ::Eigen::VectorXd;

void ExpectElementsAre(
  Eigen::Map<const Eigen::VectorXd> vec, std::vector<double> expected, const double tolerance)
{
  EXPECT_EQ(vec.size(), expected.size());
  for (int i = 0; i < vec.size(); ++i) {
    EXPECT_NEAR(vec[i], expected[i], tolerance);
  }
}

TEST(OsqpTest, DimensionMismatchObjectiveMatrix)
{
  OsqpInstance instance;
  instance.objective_matrix = SparseMatrix<double>(1, 1);
  instance.objective_vector.resize(1);
  instance.objective_vector << -1.0;
  instance.constraint_matrix = SparseMatrix<double>(2, 2);
  instance.lower_bounds.resize(2);
  instance.lower_bounds << 0.0, 0.0;
  instance.upper_bounds.resize(2);
  instance.upper_bounds << 1.0, 1.0;
  OsqpSolver solver;
  OsqpSettings settings;

  // ns_utils::print("\nStatus", StatusCodeToString(solver.Init(instance, settings).code()));
  EXPECT_EQ(solver.Init(instance, settings).code(), StatusCode::kInvalidArgument);
  EXPECT_FALSE(solver.IsInitialized());
}

TEST(OsqpTest, DimensionMismatchObjectiveVector)
{
  OsqpInstance instance;
  instance.objective_matrix = SparseMatrix<double>(1, 1);
  instance.objective_vector.resize(2);
  instance.objective_vector << 1.0, 1.0;
  instance.constraint_matrix = SparseMatrix<double>(0, 1);

  // instance.lower_bounds not set.
  // instance.upper_bounds not set.

  OsqpSolver solver;
  OsqpSettings settings;
  EXPECT_EQ(solver.Init(instance, settings).code(), StatusCode::kInvalidArgument);
  EXPECT_FALSE(solver.IsInitialized());
}

TEST(OsqpTest, DimensionMismatchLowerBounds)
{
  OsqpInstance instance;
  instance.objective_matrix = SparseMatrix<double>(1, 1);
  instance.objective_vector.resize(1);
  instance.objective_vector << 1.0;
  instance.constraint_matrix = SparseMatrix<double>(0, 1);
  instance.lower_bounds.resize(1);
  instance.lower_bounds << 1.0;

  // instance.upper_bounds not set.

  OsqpSolver solver;
  OsqpSettings settings;
  EXPECT_EQ(solver.Init(instance, settings).code(), StatusCode::kInvalidArgument);
  EXPECT_FALSE(solver.IsInitialized());
}

TEST(OsqpTest, DimensionMismatchUpperBounds)
{
  OsqpInstance instance;
  instance.objective_matrix = SparseMatrix<double>(1, 1);
  instance.objective_vector.resize(1);
  instance.objective_vector << 1.0;
  instance.constraint_matrix = SparseMatrix<double>(0, 1);
  // instance.lower_bounds not set.
  instance.upper_bounds.resize(1);
  instance.upper_bounds << 1.0;
  OsqpSolver solver;
  OsqpSettings settings;
  EXPECT_EQ(solver.Init(instance, settings).code(), StatusCode::kInvalidArgument);
  EXPECT_FALSE(solver.IsInitialized());
}

// A lower bound greater than an upper bound triggers an OSQP initialization
// error.
TEST(OsqpTest, InvalidData)
{
  OsqpInstance instance;
  instance.objective_matrix = SparseMatrix<double>(1, 1);
  instance.objective_vector.resize(1);
  instance.objective_vector << 1.0;
  instance.constraint_matrix = SparseMatrix<double>(1, 1);
  instance.lower_bounds.resize(1);
  instance.lower_bounds << 1.0;
  instance.upper_bounds.resize(1);
  instance.upper_bounds << 0.0;
  OsqpSolver solver;
  OsqpSettings settings;
  EXPECT_EQ(solver.Init(instance, settings).code(), StatusCode::kInvalidArgument);
  EXPECT_FALSE(solver.IsInitialized());
}

TEST(OsqpTest, TwoDimLP)
{
  // Minimize -x subject to:
  // x + y <= 1, x >= 0, y >= 0.

  SparseMatrix<double> constraint_matrix(3, 2);
  const Triplet<double> kTripletsA[] = {{0, 0, 1.0}, {0, 1, 1.0}, {1, 0, 1.0}, {2, 1, 1.0}};
  constraint_matrix.setFromTriplets(std::begin(kTripletsA), std::end(kTripletsA));

  OsqpInstance instance;
  instance.objective_matrix = SparseMatrix<double>(2, 2);
  instance.objective_vector.resize(2);
  instance.objective_vector << -1.0, 0.0;
  instance.constraint_matrix = constraint_matrix;
  instance.lower_bounds.resize(3);
  instance.lower_bounds << -kInfinity, 0.0, 0.0;
  instance.upper_bounds.resize(3);
  instance.upper_bounds << 1.0, kInfinity, kInfinity;

  OsqpSolver solver;
  OsqpSettings settings;
  // absolute_convergence_tolerance (eps_abs) is an l_2 tolerance on the
  // residual vector, so this is safe given we use kTolerance
  // as an l_infty tolerance.
  settings.eps_abs = kTolerance;
  settings.eps_rel = 0.0;
  EXPECT_FALSE(solver.IsInitialized());
  ASSERT_TRUE(solver.Init(instance, settings).ok());
  EXPECT_TRUE(solver.IsInitialized());
  ASSERT_EQ(solver.Solve(), OsqpExitCode::kOptimal);

  EXPECT_NEAR(solver.objective_value(), -1.0, kTolerance);
  ExpectElementsAre(solver.primal_solution(), {1.0, 0.0}, kTolerance);
  ExpectElementsAre(solver.dual_solution(), {1.0, 0.0, -1.0}, kTolerance);
}

TEST(OsqpTest, TwoDimLPWithEquality)
{
  // Minimize -x subject to:
  // x + y <= 1, x >= 0, y == 0.5.
  SparseMatrix<double> constraint_matrix(3, 2);
  const Triplet<double> kTripletsA[] = {{0, 0, 1.0}, {0, 1, 1.0}, {1, 0, 1.0}, {2, 1, 1.0}};
  constraint_matrix.setFromTriplets(std::begin(kTripletsA), std::end(kTripletsA));

  OsqpInstance instance;
  instance.objective_matrix = SparseMatrix<double>(2, 2);
  instance.objective_vector.resize(2);
  instance.objective_vector << -1.0, 0.0;
  instance.constraint_matrix = constraint_matrix;
  instance.lower_bounds.resize(3);
  instance.lower_bounds << -kInfinity, 0.0, 0.5;
  instance.upper_bounds.resize(3);
  instance.upper_bounds << 1.0, kInfinity, 0.5;

  OsqpSolver solver;
  OsqpSettings settings;
  settings.eps_abs = kTolerance;
  settings.eps_rel = 0.0;
  ASSERT_TRUE(solver.Init(instance, settings).ok());
  ASSERT_EQ(solver.Solve(), OsqpExitCode::kOptimal);

  EXPECT_NEAR(solver.objective_value(), -0.5, kTolerance);
  ExpectElementsAre(solver.primal_solution(), {0.5, 0.5}, kTolerance);
  ExpectElementsAre(solver.dual_solution(), {1.0, 0.0, -1.0}, kTolerance);
}

TEST(OsqpTest, DetectsPrimalInfeasible)
{
  // Minimize 0 subject to:
  // x == 2, x >= 3.
  SparseMatrix<double> objective_matrix(1, 1);

  SparseMatrix<double> constraint_matrix(2, 1);
  const Triplet<double> kTripletsQ[] = {{0, 0, 1.0}, {1, 0, 1.0}};
  constraint_matrix.setFromTriplets(std::begin(kTripletsQ), std::end(kTripletsQ));

  OsqpInstance instance;
  instance.objective_matrix = objective_matrix;
  instance.objective_vector.resize(1);
  instance.objective_vector << 0.0;
  instance.constraint_matrix = constraint_matrix;
  instance.lower_bounds.resize(2);
  instance.lower_bounds << 2, 3;
  instance.upper_bounds.resize(2);
  instance.upper_bounds << 2, kInfinity;

  OsqpSolver solver;
  OsqpSettings settings;
  ASSERT_TRUE(solver.Init(instance, settings).ok());
  ASSERT_EQ(solver.Solve(), OsqpExitCode::kPrimalInfeasible);
  Eigen::Map<const VectorXd> cert = solver.primal_infeasibility_certificate();

  // The infeasibility certificate satisfies cert[0] + cert[1] == 0,
  // cert[0]* 2 + cert[1] * 3 < 0, and cert[1] <= 0. See the OSQP documentation
  // for the general definition of the certificate.
  EXPECT_NEAR(cert[0] / cert[1], -1.0, kTolerance);
  EXPECT_LE(cert[1], 0.0);
  EXPECT_LT(cert[0] * 2 + cert[1] * 3, 0.0);
}

TEST(OsqpTest, NewConstraintMatrixInTwoDimLP)
{
  // Minimize -x subject to:
  // x + y <= 1, x >= 1, y >= 0.

  SparseMatrix<double> constraint_matrix(3, 2);
  const Triplet<double> kTripletsA[] = {{0, 0, 1.0}, {0, 1, 1.0}, {1, 0, 1.0}, {2, 1, 1.0}};
  constraint_matrix.setFromTriplets(std::begin(kTripletsA), std::end(kTripletsA));

  OsqpInstance instance;
  instance.objective_matrix = SparseMatrix<double>(2, 2);
  instance.objective_vector.resize(2);
  instance.objective_vector << -1.0, 0.0;
  instance.constraint_matrix = constraint_matrix;
  instance.lower_bounds.resize(3);
  instance.lower_bounds << -kInfinity, 1.0, 0.0;
  instance.upper_bounds.resize(3);
  instance.upper_bounds << 1.0, kInfinity, kInfinity;

  OsqpSolver solver;
  OsqpSettings settings;
  // absolute_convergence_tolerance (eps_abs) is an l_2 tolerance on the
  // residual vector, so this is safe given we use kTolerance
  // as an l_infty tolerance.
  settings.eps_abs = kTolerance;
  settings.eps_rel = 0.0;
  EXPECT_FALSE(solver.IsInitialized());
  ASSERT_TRUE(solver.Init(instance, settings).ok());
  EXPECT_TRUE(solver.IsInitialized());
  ASSERT_EQ(solver.Solve(), OsqpExitCode::kOptimal);

  double x_exp = 1.0;
  double y_exp = 0.0;

  EXPECT_NEAR(solver.objective_value(), -x_exp, kTolerance);
  ExpectElementsAre(solver.primal_solution(), {x_exp, y_exp}, kTolerance);

  // Change to Minimize -x subject to:
  // 2*x + y <= 1.5, 2*x >= 1, y >= 0.25
  SparseMatrix<double> new_constraint_matrix = constraint_matrix;
  new_constraint_matrix.coeffRef(0, 0) = 2;
  new_constraint_matrix.coeffRef(1, 0) = 2;

  VectorXd new_lower_bounds(3);
  new_lower_bounds << -kInfinity, 1, 0.25;
  VectorXd new_upper_bounds(3);
  new_upper_bounds << 1.5, kInfinity, kInfinity;

  ASSERT_TRUE(solver.SetBounds(new_lower_bounds, new_upper_bounds).ok());

  ASSERT_TRUE(solver.UpdateConstraintMatrix(new_constraint_matrix).ok());
  EXPECT_TRUE(solver.IsInitialized());
  ASSERT_EQ(solver.Solve(), OsqpExitCode::kOptimal);

  x_exp = 0.624997;
  y_exp = 0.250002;

  EXPECT_NEAR(solver.objective_value(), -x_exp, 2 * kTolerance);
  ExpectElementsAre(solver.primal_solution(), {x_exp, y_exp}, kTolerance);
}

class TwoDimensionalQpTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Minimize x^2 + (1/2)xy + y^2 subject to x >= 1.
    SparseMatrix<double> objective_matrix(2, 2);
    const Triplet<double> kTripletsP[] = {{0, 0, 2.0}, {1, 0, 0.5}, {0, 1, 0.5}, {1, 1, 2.0}};
    objective_matrix.setFromTriplets(std::begin(kTripletsP), std::end(kTripletsP));

    SparseMatrix<double> constraint_matrix(1, 2);
    const Triplet<double> kTripletsA[] = {{0, 0, 1.0}};
    constraint_matrix.setFromTriplets(std::begin(kTripletsA), std::end(kTripletsA));

    OsqpInstance instance;
    instance.objective_matrix = objective_matrix;
    instance.objective_vector.resize(2);
    instance.objective_vector << 0.0, 0.0;
    instance.constraint_matrix = constraint_matrix;
    instance.lower_bounds.resize(1);
    instance.lower_bounds << 1.0;
    instance.upper_bounds.resize(1);
    instance.upper_bounds << kInfinity;

    OsqpSettings settings;
    settings.eps_abs = kTolerance / 10;
    settings.eps_rel = 0.0;
    // check_termination is set so that the warm-started instance converges
    // immediately.
    settings.check_termination = 1;
    ASSERT_TRUE(solver_.Init(instance, settings).ok());
  }

  OsqpSolver solver_;
};

TEST_F(TwoDimensionalQpTest, Solves)
{
  ASSERT_EQ(solver_.Solve(), OsqpExitCode::kOptimal);

  // NOTE(ml): The convergence guarantees don't imply that the error in
  // objective_value() is within kTolerance. To be more precise, we could
  // compute the worst error bound in a kTolerance box around the optimal
  // solution, but 2 * kTolerance works here.
  EXPECT_NEAR(solver_.objective_value(), 1.0 - 0.5 * 0.25 + 0.25 * 0.25, 2 * kTolerance);
  ExpectElementsAre(solver_.primal_solution(), {1.0, -0.25}, kTolerance);
  ExpectElementsAre(solver_.dual_solution(), {-15.0 / 8.0}, 2 * kTolerance);
}

TEST_F(TwoDimensionalQpTest, UsesJointWarmStart)
{
  VectorXd primal_warm_start(2);
  primal_warm_start << 1.0, -0.25;
  VectorXd dual_warm_start(1);
  dual_warm_start << -15.0 / 8.0;
  ASSERT_TRUE(solver_.SetWarmStart(primal_warm_start, dual_warm_start).ok());
  ASSERT_EQ(solver_.Solve(), OsqpExitCode::kOptimal);
  EXPECT_EQ(solver_.iterations(), 1);
}

TEST_F(TwoDimensionalQpTest, UsesSeparateWarmStart)
{
  VectorXd primal_warm_start(2);
  primal_warm_start << 1.0, -0.25;
  VectorXd dual_warm_start(1);
  dual_warm_start << -15.0 / 8.0;
  ASSERT_TRUE(solver_.SetPrimalWarmStart(primal_warm_start).ok());
  ASSERT_TRUE(solver_.SetDualWarmStart(dual_warm_start).ok());
  ASSERT_EQ(solver_.Solve(), OsqpExitCode::kOptimal);
  EXPECT_EQ(solver_.iterations(), 1);
}

TEST_F(TwoDimensionalQpTest, SolvesWithNewObjectiveVector)
{
  // Changes the objective to x^2 + (1/2)xy + y^2 + x
  VectorXd objective_vector(2);
  objective_vector << 1.0, 0.0;
  ASSERT_TRUE(solver_.SetObjectiveVector(objective_vector).ok());
  ASSERT_EQ(solver_.Solve(), OsqpExitCode::kOptimal);

  EXPECT_NEAR(solver_.objective_value(), 1.0 - 0.5 * 0.25 + 0.25 * 0.25 + 1.0, 3 * kTolerance);
  ExpectElementsAre(solver_.primal_solution(), {1.0, -0.25}, kTolerance);
}

TEST_F(TwoDimensionalQpTest, SolvesWithNewBounds)
{
  // Fixes x to 2.0.
  VectorXd bound_vector(1);
  bound_vector << 2.0;
  ASSERT_TRUE(solver_.SetBounds(bound_vector, bound_vector).ok());
  ASSERT_EQ(solver_.Solve(), OsqpExitCode::kOptimal);
  EXPECT_NEAR(solver_.objective_value(), 4.0 - 0.5 + 0.5 * 0.5, 2 * kTolerance);
  ExpectElementsAre(solver_.primal_solution(), {2.0, -0.5}, kTolerance);
}

// These tests are instantiated twice, once with a non-upper triangular
// objective matrix, and once with an upper triangular objective matrix.
class ParameterizedTwoDimensionalQpTest : public testing::WithParamInterface<bool>,
  public TwoDimensionalQpTest
{
};

TEST_P(ParameterizedTwoDimensionalQpTest, SolvesWithNewObjectiveMatrix)
{
  // Minimize x^2 + (1/2)xy + y^2 subject to x >= 1.
  ASSERT_EQ(solver_.Solve(), OsqpExitCode::kOptimal);

  double x_exp = 1.0;
  double y_exp = -0.25;

  EXPECT_NEAR(
    solver_.objective_value(), x_exp * x_exp + 0.5 * x_exp * y_exp + y_exp * y_exp, 2 * kTolerance);
  ExpectElementsAre(solver_.primal_solution(), {x_exp, y_exp}, kTolerance);
  ExpectElementsAre(solver_.dual_solution(), {-15.0 / 8.0}, 2 * kTolerance);

  // Minimize x^2 + (1/2)xy + (1/2)y^2 subject to x >= 1.
  SparseMatrix<double> new_objective_matrix(2, 2);
  const Triplet<double> kTripletsP[] = {{0, 0, 2.0}, {1, 0, 0.5}, {0, 1, 0.5}, {1, 1, 1}};
  new_objective_matrix.setFromTriplets(std::begin(kTripletsP), std::end(kTripletsP));
  if (GetParam()) {
    new_objective_matrix = new_objective_matrix.triangularView<Eigen::Upper>();
  }

  ASSERT_TRUE(solver_.UpdateObjectiveMatrix(new_objective_matrix).ok());
  ASSERT_EQ(solver_.Solve(), OsqpExitCode::kOptimal);

  x_exp = 1.0;
  y_exp = -0.5;
  EXPECT_NEAR(
    solver_.objective_value(), x_exp * x_exp + 0.5 * x_exp * y_exp + 0.5 * y_exp * y_exp,
    2 * kTolerance);
  ExpectElementsAre(solver_.primal_solution(), {x_exp, y_exp}, kTolerance);
}

TEST_P(ParameterizedTwoDimensionalQpTest, SolvesWithNewObjectiveAndConstraintMatrices)
{
  // Minimize x^2 + (1/2)xy + y^2 subject to x >= 1.
  ASSERT_EQ(solver_.Solve(), OsqpExitCode::kOptimal);

  double x_exp = 1.0;
  double y_exp = -0.25;

  EXPECT_NEAR(
    solver_.objective_value(), x_exp * x_exp + 0.5 * x_exp * y_exp + y_exp * y_exp, 2 * kTolerance);
  ExpectElementsAre(solver_.primal_solution(), {x_exp, y_exp}, kTolerance);
  ExpectElementsAre(solver_.dual_solution(), {-15.0 / 8.0}, 2 * kTolerance);

  // Change to minimize x^2 + (1/2)xy + (1/2)y^2 subject to 0.5*x >= 1.
  SparseMatrix<double> new_objective_matrix(2, 2);
  const Triplet<double> kTripletsP[] = {{0, 0, 2.0}, {1, 0, 0.5}, {0, 1, 0.5}, {1, 1, 1}};
  new_objective_matrix.setFromTriplets(std::begin(kTripletsP), std::end(kTripletsP));
  if (GetParam()) {
    new_objective_matrix = new_objective_matrix.triangularView<Eigen::Upper>();
  }

  SparseMatrix<double> new_constraint_matrix(1, 2);
  const Triplet<double> kTripletsA[] = {{0, 0, 0.5}};
  new_constraint_matrix.setFromTriplets(std::begin(kTripletsA), std::end(kTripletsA));

  const Status result =
    solver_.UpdateObjectiveAndConstraintMatrices(new_objective_matrix, new_constraint_matrix);
  ASSERT_TRUE(result.ok());
  ASSERT_EQ(solver_.Solve(), OsqpExitCode::kOptimal);

  x_exp = 2.0;
  y_exp = -1.0;
  EXPECT_NEAR(
    solver_.objective_value(), x_exp * x_exp + 0.5 * x_exp * y_exp + 0.5 * y_exp * y_exp,
    2 * kTolerance);
  ExpectElementsAre(solver_.primal_solution(), {x_exp, y_exp}, kTolerance);
}

TEST(OsqpTest, ErrorIfNotPsd)
{
  // Minimize -x^2.

  SparseMatrix<double> objective_matrix(1, 1);
  const Triplet<double> kTripletsP[] = {{0, 0, -2.0}};
  objective_matrix.setFromTriplets(std::begin(kTripletsP), std::end(kTripletsP));

  SparseMatrix<double> constraint_matrix(0, 1);

  OsqpInstance instance;
  instance.objective_matrix = objective_matrix;
  instance.objective_vector.resize(1);
  instance.objective_vector << 0.0;
  instance.constraint_matrix = constraint_matrix;

  // instance.lower_bounds not set.
  // instance.upper_bounds not set.

  OsqpSolver solver;
  OsqpSettings settings;
  settings.eps_abs = kTolerance;
  settings.eps_rel = 0.0;
  settings.verbose = true;
  EXPECT_EQ(solver.Init(instance, settings).code(), StatusCode::kInvalidArgument);
}

// This returns an OsqpInstance that corresponds to minimizing
//   (x-1)^2 + (y-1)^2 + c
// within the bounds -100 <= x, y <= 100.
OsqpInstance GetToyProblem()
{
  OsqpInstance instance;

  instance.objective_matrix = SparseMatrix<double>(2, 2);
  const Triplet<double> triplets[] = {{0, 0, 2.0}, {1, 1, 2.0}};
  instance.objective_matrix.setFromTriplets(std::begin(triplets), std::end(triplets));

  instance.objective_vector.resize(2);
  instance.objective_vector << -2.0, -2.0;

  instance.constraint_matrix = instance.objective_matrix;

  instance.lower_bounds.resize(2);
  instance.lower_bounds << -100.0, -100.0;

  instance.upper_bounds.resize(2);
  instance.upper_bounds << 100.0, 100.0;

  return instance;
}

// Returns the solution to the problem returned by GetToyProblem().
VectorXd GetToySolution()
{
  VectorXd soln(2);
  soln << 1.0, 1.0;
  return soln;
}

TEST(OsqpTest, UpdateMaxIter)
{
  OsqpSettings settings;
  settings.eps_abs = 1e-6;
  settings.max_iter = 1;

  // Try solving an easy problem and verify that we only used one iteration.
  OsqpSolver solver;
  ASSERT_TRUE(solver.Init(GetToyProblem(), settings).ok());
  ASSERT_EQ(solver.Solve(), OsqpExitCode::kMaxIterations);
  EXPECT_EQ(solver.iterations(), 1);

  // Now allow for more iterations and check that they are actually used.
  ASSERT_TRUE(solver.UpdateMaxIter(2).ok());
  ASSERT_EQ(solver.Solve(), OsqpExitCode::kMaxIterations);
  EXPECT_EQ(solver.iterations(), 2);

  // Setting a large number of iterations allows us to actually solve the
  // problem.
  ASSERT_TRUE(solver.UpdateMaxIter(1000).ok());
  ASSERT_EQ(solver.Solve(), OsqpExitCode::kOptimal);
}

TEST(OsqpTest, UpdateEpsAbs)
{
  OsqpSettings settings;
  // Check for termination after every iteration.
  settings.check_termination = 1;
  // Disable eps_rel to isolate the effect of eps_abs.
  settings.eps_rel = 0.0;
  // Set a very loose eps_abs value.
  settings.eps_abs = 1;

  // Solve and make a note of the error in our solution.
  OsqpSolver solver;
  ASSERT_TRUE(solver.Init(GetToyProblem(), settings).ok());
  ASSERT_EQ(solver.Solve(), OsqpExitCode::kOptimal);
  const double err1 = (solver.primal_solution() - GetToySolution()).norm();

  // Now set a much tighter eps_abs and solve again.
  ASSERT_TRUE(solver.UpdateEpsAbs(1e-7).ok());
  ASSERT_EQ(solver.Solve(), OsqpExitCode::kOptimal);
  const double err2 = (solver.primal_solution() - GetToySolution()).norm();

  // We should have ended up with a significantly smaller error the second time.
  EXPECT_LT(100 * err2, err1);
  // And we should be nearly optimal.
  EXPECT_LT(err2, 1e-5);
}

}  // namespace
}  // namespace osqp
