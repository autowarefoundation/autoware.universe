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

#ifndef FRENET_PLANNER__POLYNOMIALS_HPP_
#define FRENET_PLANNER__POLYNOMIALS_HPP_

namespace frenet_planner
{
class Polynomial
{
  // @brief polynomial coefficients
  double a_;
  double b_;
  double c_;
  double d_;
  double e_;
  double f_;

public:
  Polynomial(
    const double x0, const double x0v, const double x0a, const double xT, const double xTv,
    const double xTa, const double T);
  // TODO(Maxime CLEMENT) add case where we do not care about final position/vel/acc

  /// @brief Get the position at the given time
  [[nodiscard]] double position(const double t) const;
  /// @brief Get the velocity at the given time
  [[nodiscard]] double velocity(const double t) const;
  /// @brief Get the acceleration at the given time
  [[nodiscard]] double acceleration(const double t) const;
  /// @brief Get the jerk at the given time
  [[nodiscard]] double jerk(const double t) const;
};
}  // namespace frenet_planner

#endif  // FRENET_PLANNER__POLYNOMIALS_HPP_
