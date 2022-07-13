// Copyright 2022 The Autoware Foundation.
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

#include "control/act_signal_builder.hpp"

Eigen::VectorXd ns_control_toolbox::make_time_signal(const double & dt, const double & final_time)
{
  Eigen::Index num_of_time_points = static_cast<Eigen::Index>(final_time / dt) + 1;
  auto && time_vector = Eigen::VectorXd::LinSpaced(num_of_time_points, 0., final_time);
  return time_vector;
}

Eigen::VectorXd ns_control_toolbox::make_sinus_signal(
  const Eigen::VectorXd & time_vec, const double & frequency_hz)
{
  double const w_ = 2 * M_PI * frequency_hz;  // [rad/sec]
  auto sin_vec = Eigen::VectorXd(time_vec.unaryExpr([&](auto const & t) { return sin(w_ * t); }));

  return sin_vec;
}

Eigen::VectorXd ns_control_toolbox::make_square_signal(
  const Eigen::VectorXd & time_vec, const double & frequency_hz)
{
  double const w_ = 2 * M_PI * frequency_hz;  // [rad/sec]
  auto sqr_vec = Eigen::VectorXd(time_vec.unaryExpr([&](auto const & t) {
    return sin(w_ * t) < 0 ? -1. : sin(w_ * t) == 0 ? 0. : 1.;
  }));
  return sqr_vec;
}

Eigen::VectorXd ns_control_toolbox::make_triangle_signal(
  const Eigen::VectorXd & time_vec, const double & frequency_hz)
{
  // double const w_ = 2 * M_PI * frequency_hz; // [rad/sec]
  double period{1.};

  if (frequency_hz > 0) {
    period = 1. / frequency_hz;
  }

  auto tmax = time_vec.maxCoeff();

  // define a starting point.

  auto trg_vec = Eigen::VectorXd(time_vec.unaryExpr([&](auto const & t) {
    auto tn = t / tmax;
    auto y = 2 * std::fabs(tn / period - std::floor(tn / period + 1. / 2));

    return y;
  }));
  return trg_vec;
}
