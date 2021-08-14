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

#include "spline_interpolation/spline_interpolation.hpp"

namespace spline_interpolation
{
void SplineInterpolator::generateSpline(
  const std::vector<double> & base_index, const std::vector<double> & base_value)
{
  const size_t N = base_value.size();

  a_.clear();
  b_.clear();
  c_.clear();
  d_.clear();
  h_.clear();

  a_ = base_value;

  for (size_t i = 0; i < N - 1; ++i) {
    h_.push_back(base_index[i + 1] - base_index[i]);
  }

  if (method_ == Method::Explicit) {
    c_ = solveLinearSystemExplicit();
  } else if (method_ == Method::PCG) {
    PCG_solver_.setCoefficients(h_, a_, 1000);
    c_ = PCG_solver_.solve();
  } else if (method_ == Method::SOR) {
    SOR_solver_.setCoefficients(h_, a_, 1000);
    c_ = SOR_solver_.solve();
  } else {
    std::cerr << "Unsupported method. Supported solver: Explicit, PCG, SOR" << std::endl;
  }

  for (size_t i = 0; i < N - 1; i++) {
    d_.push_back((c_[i + 1] - c_[i]) / (3.0 * h_[i]));
    b_.push_back((a_[i + 1] - a_[i]) / h_[i] - h_[i] * (2.0 * c_[i] + c_[i + 1]) / 3.0);
  }

  d_.push_back(0.0);
  b_.push_back(0.0);

  initialized_ = true;
}

double SplineInterpolator::getValue(
  const double & query, const std::vector<double> & base_index) const
{
  if (!initialized_) {
    std::cerr << "[interpolate] spline is uninitialized" << std::endl;
    return 0.0;
  }

  // Find corresponding index
  size_t j = 0;
  while (base_index[j] <= query) {
    ++j;

    // Prevent out-of-range access at endpoint
    if (j == base_index.size()) {
      break;
    }
  }
  --j;

  const double ds = query - base_index[j];
  return a_[j] + (b_[j] + (c_[j] + d_[j] * ds) * ds) * ds;
}

bool SplineInterpolator::interpolate(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index, std::vector<double> & return_value, const Method method)
{
  method_ = method;

  return_value.clear();

  if (!isValidInput(base_index, base_value, return_index, return_value)) {
    std::cerr << "[interpolate] invalid input. interpolation failed." << std::endl;
    return false;
  }

  // calculate spline coefficients
  generateSpline(base_index, base_value);

  // interpolate values at query points
  for (size_t i = 0; i < return_index.size(); ++i) {
    return_value.push_back(getValue(return_index[i], base_index));
  }
  return true;
}

bool SplineInterpolator::isIncrease(const std::vector<double> & x) const
{
  for (int i = 0; i < static_cast<int>(x.size()) - 1; ++i) {
    if (x[i] >= x[i + 1]) {return false;}
  }
  return true;
}

bool SplineInterpolator::isNonDecrease(const std::vector<double> & x) const
{
  for (int i = 0; i < static_cast<int>(x.size()) - 1; ++i) {
    if (x[i] > x[i + 1]) {return false;}
  }
  return true;
}

bool SplineInterpolator::isValidInput(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index,
  [[maybe_unused]] std::vector<double> & return_value) const
{
  if (base_index.empty() || base_value.empty() || return_index.empty()) {
    std::cout << "bad index : some vector is empty. base_index: " << base_index.size() <<
      ", base_value: " << base_value.size() << ", return_index: " << return_index.size() <<
      std::endl;
    return false;
  }
  if (!isIncrease(base_index)) {
    std::cout << "bad index : base_index is not monotonically increasing. base_index = [" <<
      base_index.front() << ", " << base_index.back() << "]" << std::endl;
    return false;
  }
  if (!isNonDecrease(return_index)) {
    std::cout << "bad index : base_index is not monotonically nondecreasing. return_index = [" <<
      return_index.front() << ", " << return_index.back() << "]" << std::endl;
    return false;
  }
  if (return_index.front() < base_index.front()) {
    std::cout << "bad index : return_index.front() < base_index.front()" << std::endl;
    return false;
  }
  if (base_index.back() < return_index.back()) {
    std::cout << "bad index : base_index.back() < return_index.back()" << std::endl;
    return false;
  }
  if (base_index.size() != base_value.size()) {
    std::cout << "bad index : base_index.size() != base_value.size()" << std::endl;
    return false;
  }

  return true;
}

std::vector<double> SplineInterpolator::solveLinearSystemExplicit()
{
  const size_t N = a_.size();
  std::vector<double> ans;
  ans.push_back(0.0);
  for (size_t i = 1; i < N - 1; i++) {
    ans.push_back(3.0 * (a_[i - 1] - 2.0 * a_[i] + a_[i + 1]));
  }
  ans.push_back(0.0);

  std::vector<double> w;
  w.push_back(0.0);

  for (size_t i = 1; i < N - 1; ++i) {
    const double tmp = 1.0 / (4.0 - w[i - 1]);
    ans[i] = (ans[i] - ans[i - 1]) * tmp;
    w.push_back(tmp);
  }

  for (size_t i = N - 2; i > 0; --i) {
    ans[i] = ans[i] - ans[i + 1] * w[i];
  }

  return ans;
}

void LinearSystemSolver::setCoefficients(
  const std::vector<double> & h, const std::vector<double> & a, const size_t max_iter)
{
  max_iter_ = max_iter;

  rhs_.assign(a.size(), 0.0);
  coef_prev_.assign(a.size(), 0.0);
  coef_diag_.assign(a.size(), 1.0);
  coef_next_.assign(a.size(), 0.0);

  for (size_t i = 1; i < a.size() - 1; ++i) {
    rhs_[i] = 3.0 / h[i] * (a[i + 1] - a[i]) - 3.0 / h[i - 1] * (a[i] - a[i - 1]);
    coef_prev_[i] = h[i - 1];
    coef_diag_[i] = 2.0 * (h[i] + h[i - 1]);
    coef_next_[i] = h[i];
  }
}

bool LinearSystemSolver::isConvergeL1(
  const std::vector<double> & r1, const std::vector<double> & r2) const
{
  double d = 0.0;
  for (size_t i = 0; i < r1.size(); ++i) {
    d += std::fabs(r1.at(i) - r2.at(i));
  }
  return d < converge_range_;
}

std::vector<double> PreconditionedConjugateGradient::solve() const
{
  // allocation
  const size_t N = rhs_.size();
  std::vector<double> x(N);          // Solution
  std::vector<double> r(N - 2);      // Residual
  std::vector<double> rn(N - 2);     // Residual (next step)
  std::vector<double> z(N - 2);      // Preconditioned residual
  std::vector<double> zn(N - 2);     // Preconditioned residual (next step)
  std::vector<double> p(N - 2);      // Basis
  std::vector<double> zeros(N - 2);  // Zero vector (for check convergence)

  // initialization
  x[0] = rhs_[0];
  x[N - 1] = rhs_[N - 1];

  // extract rhs[1:N-2] (rhs[1] = rhs[1] - h[0]*rhs[0], rhs[N-2] = rhs[N-2] - h[N-2]*rhs[N-1])
  std::vector<double> rhs_in = rhs_;
  rhs_in[1] -= coef_prev_[1] * rhs_[0];
  rhs_in[N - 2] -= coef_next_[N - 2] * rhs_[N - 1];
  rhs_in.erase(rhs_in.begin());
  rhs_in.erase(std::prev(rhs_in.end()));

  // extract x[1:N-2]
  std::vector<double> x_in = x;
  x_in.erase(x_in.begin());
  x_in.erase(std::prev(x_in.end()));

  // r = rhs - Ax
  const std::vector<double> Ax = calcMatrixVectorProduct(x_in);
  std::transform(rhs_in.begin(), rhs_in.end(), Ax.begin(), r.begin(), std::minus<double>());
  if (isConvergeL1(rhs_in, Ax)) {return x;}
  // p0 = DiagonalScaling(r)
  // z0 = p0
  p = calcDiagonalScaling(r);
  z = p;
  size_t num_iter = max_iter_;
  for (size_t iter = 0; iter <= max_iter_; ++iter) {
    // (1) y_k = A * p_k
    const std::vector<double> y = calcMatrixVectorProduct(p);
    // (2) alpha = (r_k' * z_k) / (p_k' * y_k);
    const double alpha = std::inner_product(r.begin(), r.end(), z.begin(), 0.0f) /
      std::inner_product(p.begin(), p.end(), y.begin(), 0.0f);
    // (3) x_k+1 = x_k + alpha * p_k
    std::transform(
      x_in.begin(), x_in.end(), p.begin(), x_in.begin(), [alpha](double x, double p) {
        return x + alpha * p;
      });
    // (4) r_k+1 = r_k - alpha * y_k
    std::transform(
      r.begin(), r.end(), y.begin(), rn.begin(), [alpha](double r, double y) {
        return r - alpha * y;
      });
    // (5) check convergence
    if (isConvergeL1(rn, zeros)) {
      num_iter = iter;
      break;
    }
    // (6) zn = DiagonalScaling(rn)
    zn = calcDiagonalScaling(rn);
    // (7) beta = (r_k+1' * z_k+1) / (r_k * z_k)
    const double beta = std::inner_product(rn.begin(), rn.end(), zn.begin(), 0.0f) /
      std::inner_product(r.begin(), r.end(), z.begin(), 0.0f);
    // (8) p_k+1 = z_k+1 + beta * p_k
    std::transform(
      zn.begin(), zn.end(), p.begin(), p.begin(), [beta](double zn, double p) {
        return zn + beta * p;
      });
    r = rn;
    z = zn;
  }

  for (size_t i = 1; i < x.size(); ++i) {
    x[i] = x_in[i - 1];
  }

  if (num_iter == max_iter_) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "PreconditionedConjugateGradient"), "[interpolate (PCG)] unconverged!");
  }
  return x;
}

std::vector<double> PreconditionedConjugateGradient::calcMatrixVectorProduct(
  const std::vector<double> & src) const
{
  std::vector<double> dst(src.size(), 0.0);
  for (size_t i = 0; i < src.size(); ++i) {
    if (i != 0) {dst[i] += coef_prev_[i + 1] * src[i - 1];}
    dst[i] += coef_diag_[i + 1] * src[i];
    if (i != (src.size() - 1)) {dst[i] += coef_next_[i + 1] * src[i + 1];}
  }
  return dst;
}

std::vector<double> PreconditionedConjugateGradient::calcDiagonalScaling(
  const std::vector<double> & src) const
{
  std::vector<double> dst(src.size(), 0.0);
  for (size_t i = 0; i < src.size(); ++i) {
    dst[i] = src[i] / coef_diag_[i + 1];
  }
  return dst;
}

std::vector<double> SOR::solve() const
{
  // solve A * ans = rhs by SOR method
  std::vector<double> ans(rhs_.size(), 1.0);
  std::vector<double> ans_next(rhs_.size(), 0.0);
  size_t num_iter = 0;

  while (!isConvergeL1(ans, ans_next) && num_iter <= max_iter_) {
    ans = ans_next;
    for (size_t i = 1; i < rhs_.size() - 1; ++i) {
      ans_next[i] += omega_ / (coef_diag_[i]) *
        (rhs_[i] - (coef_prev_[i] * ans_next[i - 1] + coef_diag_[i] * ans[i] +
        coef_next_[i] * ans[i + 1]));
    }
    ++num_iter;
  }
  if (num_iter > max_iter_) {
    RCLCPP_WARN(rclcpp::get_logger("SOR"), "[interpolate (SOR)] unconverged!");
  }
  return ans_next;
}

}  // namespace spline_interpolation
