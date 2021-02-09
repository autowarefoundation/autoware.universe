/*
 * Copyright 2021 Autoware Foundation. All rights reserved.
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

#include <frenet_planner/transform/spline_transform.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace frenet_planner::transform
{
Spline::Spline(const std::vector<double> & base_index, const std::vector<double> & base_value)
{
  generateSpline(base_index, base_value);
}

Spline::Spline(const std::vector<Point> & points)
{
  std::vector<double> xs;
  std::vector<double> ys;
  xs.reserve(points.size());
  ys.reserve(points.size());
  for (const auto & p : points) {
    xs.push_back(p.x());
    ys.push_back(p.y());
  }
  generateSpline(xs, ys);
}

void Spline::generateSpline(
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

  c_ = solveLinearSystem(1.8, 100);

  for (size_t i = 0; i < N - 1; i++) {
    d_.push_back((c_[i + 1] - c_[i]) / (3.0 * h_[i]));
    b_.push_back((a_[i + 1] - a_[i]) / h_[i] - h_[i] * (2.0 * c_[i] + c_[i + 1]) / 3.0);
  }

  d_.push_back(0.0);
  b_.push_back(0.0);
}

double Spline::value(const double query, const std::vector<double> & base_index) const
{
  const auto ub = std::upper_bound(base_index.begin(), base_index.end(), query);
  const size_t i = std::distance(base_index.begin(), ub) - 1;
  const double ds = query - base_index[i];
  return a_[i] + (b_[i] + (c_[i] + d_[i] * ds) * ds) * ds;
}

double Spline::velocity(const double query, const std::vector<double> & base_index) const
{
  const auto lb = std::lower_bound(base_index.begin(), base_index.end(), query);
  const size_t i = std::distance(base_index.begin(), lb) - 1;
  const auto delta = query - base_index[i];
  return b_[i] + 2.0 * c_[i] * delta + 3.0 * d_[i] * delta * delta;
}

double Spline::acceleration(const double query, const std::vector<double> & base_index) const
{
  const auto lb = std::lower_bound(base_index.begin(), base_index.end(), query);
  const size_t i = std::distance(base_index.begin(), lb) - 1;
  return 2.0 * c_[i] + 6.0 * d_[i] * (query - base_index[i]);
}

bool Spline::isIncrease(const std::vector<double> & x) const
{
  for (int i = 0; i < static_cast<int>(x.size()) - 1; ++i) {
    if (x[i] > x[i + 1]) return false;
  }
  return true;
}

bool Spline::isValidInput(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index) const
{
  if (base_index.empty() || base_value.empty() || return_index.empty()) {
    std::cout << "bad index : some vector is empty. base_index: " << base_index.size()
              << ", base_value: " << base_value.size() << ", return_index: " << return_index.size()
              << std::endl;
    return false;
  }
  if (!isIncrease(base_index)) {
    std::cout << "bad index : base_index is not monotonically increasing. base_index = ["
              << base_index.front() << ", " << base_index.back() << "]" << std::endl;
    return false;
  }
  if (!isIncrease(return_index)) {
    std::cout << "bad index : base_index is not monotonically increasing. return_index = ["
              << return_index.front() << ", " << return_index.back() << "]" << std::endl;
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

std::vector<double> Spline::solveLinearSystem(const double omega, const size_t max_iter) const
{
  // solve A * ans = rhs by SOR method
  constexpr double converge_range = 0.00001;
  std::vector<double> ans(a_.size(), 1.0);
  std::vector<double> ans_next(a_.size(), 0.0);
  size_t num_iter = 0;

  while (!isConvergeL1(ans, ans_next, converge_range) && num_iter <= max_iter) {
    ans = ans_next;
    for (size_t i = 1; i < a_.size() - 1; ++i) {
      const double rhs = 3.0 / h_[i] * (a_[i + 1] - a_[i]) - 3.0 / h_[i - 1] * (a_[i] - a_[i - 1]);
      ans_next[i] += omega / (2.0 * (h_[i - 1] + h_[i])) *
                     (rhs - (h_[i - 1] * ans_next[i - 1] + 2.0 * (h_[i - 1] + h_[i]) * ans[i] +
                             h_[i] * ans[i + 1]));
    }
    ++num_iter;
  }

  if (num_iter > max_iter) std::cerr << "[Spline::solveLinearSystem] unconverged!\n";
  return ans_next;
}

bool Spline::isConvergeL1(
  const std::vector<double> & r1, const std::vector<double> & r2, const double converge_range) const
{
  double d = 0.0;
  for (size_t i = 0; i < r1.size(); ++i) {
    d += std::fabs(r1.at(i) - r2.at(i));
  }
  return d < converge_range;
}

/*
 * 2D Spline
 */

Spline2D::Spline2D(const std::vector<double> & x, const std::vector<double> & y)
: s_(arcLength(x, y)), x_spline_{s_, x}, y_spline_{s_, y}
{
}

// @brief Calculate the distances of the points along the path
// TODO(Maxime CLEMENT) need something more precise than linear segments ?
// https://medium.com/@all2one/how-to-compute-the-length-of-a-spline-e44f5f04c40
std::vector<double> Spline2D::arcLength(
  const std::vector<double> & x, const std::vector<double> & y)
{
  std::vector<double> s = {0.0};
  s.reserve(x.size() - 1);
  for (size_t i = 0; i < x.size() - 1; ++i) {
    const double ds = std::hypot((x[i + 1] - x[i]), (y[i + 1] - y[i]));
    s.push_back(s.back() + ds);
  }
  return s;
}

// @brief Convert the given point to the Frenet frame of this spline
FrenetPoint Spline2D::frenet(const Point & p) const
{
  // TODO(Maxime CLEMENT) Implement fast algorithm from https://hal.inria.fr/inria-00518379/document
  // Naive approach: sample points along the splines and return the closest one
  constexpr float precision = 0.01;  // 1cm precision
  double closest_d = std::numeric_limits<double>::max();
  double arc_length = std::numeric_limits<double>::min();
  for (double s = s_.front(); s < s_.back(); s += precision) {
    const double x_s = x_spline_.value(s, s_);
    const double y_s = y_spline_.value(s, s_);
    const double d = std::hypot(p.x() - x_s, p.y() - y_s);
    if (d <= closest_d)  // also accept equality to return the largest possible arc length
    {
      closest_d = d;
      arc_length = s;
    }
  }
  // check sign of d
  const double x0 = x_spline_.value(arc_length, s_);
  const double y0 = y_spline_.value(arc_length, s_);
  const double x1 = x_spline_.value(arc_length + precision, s_);
  const double y1 = y_spline_.value(arc_length + precision, s_);
  if ((x1 - x0) * (p.y() - y0) - (y1 - y0) * (p.x() - x0) < 0) {
    closest_d *= -1.0;
  }
  return {arc_length, closest_d};
}

Point Spline2D::cartesian(const double s) const
{
  return {x_spline_.value(s, s_), y_spline_.value(s, s_)};
}

Point Spline2D::cartesian(const FrenetPoint & fp) const
{
  const auto heading = yaw(fp.s);
  const auto x = x_spline_.value(fp.s, s_);
  const auto y = y_spline_.value(fp.s, s_);
  return {x + fp.d * std::cos(heading + M_PI_2), y + fp.d * std::sin(heading + M_PI_2)};
}

double Spline2D::curvature(const double s) const
{
  // TODO(Maxime CLEMENT) if need to gain performance, search for s in s_ here and pass index
  const double x_vel = x_spline_.velocity(s, s_);
  const double y_vel = y_spline_.velocity(s, s_);
  const double x_acc = x_spline_.acceleration(s, s_);
  const double y_acc = y_spline_.acceleration(s, s_);
  return (y_acc * x_vel - x_acc * y_vel) / std::pow(x_vel * x_vel + y_vel * y_vel, 3.0 / 2.0);
}

double Spline2D::yaw(const double s) const
{
  const double x_vel = x_spline_.velocity(s, s_);
  const double y_vel = y_spline_.velocity(s, s_);
  return std::atan2(y_vel, x_vel);
}

}  // namespace frenet_planner::transform
