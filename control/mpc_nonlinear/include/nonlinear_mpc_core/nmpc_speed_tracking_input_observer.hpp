/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
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

#ifndef NONLINEAR_MPC_CORE__NMPC_SPEED_TRACKING_INPUT_OBSERVER_HPP_
#define NONLINEAR_MPC_CORE__NMPC_SPEED_TRACKING_INPUT_OBSERVER_HPP_

namespace ns_filters
{
class SimpleDisturbanceInputObserver
{
public:
  SimpleDisturbanceInputObserver() = default;
  explicit SimpleDisturbanceInputObserver(double const & observer_gain) : L_{observer_gain} {}

  ~SimpleDisturbanceInputObserver() = default;
  /**
   * @brief Get the next Input disturbance estimate
   *
   * @param xdot  xdot = Ax + Bu a 1D linear system.
   * @param dt
   * @return d[k+1] next input disturbance.
   */
  double getNextInputDisturbance(double const & x, double const & xdot, double const & dt);

  double reset()
  {
    zk_ = 0.0;
    dk_ = 0.0;
    return dk_;
  }

private:
  double L_{1.};  // !<-@brief observer gain  must be [> 0.]
  double dk_{};   // !<-@brief disturbance input estimate
  double zk_{};   // !<-@brief internal state of the disturbance observer
};
}  // namespace ns_filters
#endif  // NONLINEAR_MPC_CORE__NMPC_SPEED_TRACKING_INPUT_OBSERVER_HPP_
