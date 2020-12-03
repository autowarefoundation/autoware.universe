// Copyright 2020 Tier IV, Inc.
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

#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace spline_interpolation
{
enum class Method { Explicit, PCG, SOR };

class LinearSystemSolver
{
public:
  void setCoefficients(
    const std::vector<double> & h, const std::vector<double> & a, const size_t max_iter);
  virtual std::vector<double> solve() const = 0;

protected:
  size_t max_iter_;
  const double converge_range_ = 0.00001;
  std::vector<double> coef_prev_;
  std::vector<double> coef_diag_;
  std::vector<double> coef_next_;
  std::vector<double> rhs_;

  bool isConvergeL1(const std::vector<double> & r1, const std::vector<double> & r2) const;
};

class PreconditionedConjugateGradient : public LinearSystemSolver
{
public:
  PreconditionedConjugateGradient() = default;
  std::vector<double> solve() const;

private:
  std::vector<double> calcMatrixVectorProduct(const std::vector<double> & src) const;
  std::vector<double> calcDiagonalScaling(const std::vector<double> & src) const;
  double calcInnerProduct(const std::vector<double> & src1, const std::vector<double> & src2) const;
};

class SOR : public LinearSystemSolver
{
public:
  SOR() = default;
  std::vector<double> solve() const;

private:
  const double omega_ = 1.8;
};

class SplineInterpolator
{
public:
  SplineInterpolator() = default;
  bool interpolate(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const std::vector<double> & return_index, std::vector<double> & return_value,
    const Method method = Method::PCG);

private:
  double getValue(const double & query, const std::vector<double> & base_index) const;
  void generateSpline(
    const std::vector<double> & base_index, const std::vector<double> & base_value);
  bool isIncrease(const std::vector<double> & x) const;
  bool isNonDecrease(const std::vector<double> & x) const;
  bool isValidInput(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const std::vector<double> & return_index, std::vector<double> & return_value) const;

  std::vector<double> solveLinearSystemExplicit();

  bool initialized_;
  Method method_;

  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
  std::vector<double> h_;

  PreconditionedConjugateGradient PCG_solver_;
  SOR SOR_solver_;
};

}  // namespace spline_interpolation
