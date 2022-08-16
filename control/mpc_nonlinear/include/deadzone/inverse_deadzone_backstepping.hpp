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


#ifndef MPC_NONLINEAR_INCLUDE_DEADZONE_INVERSE_DEADZONE_BACKSTEPPING_HPP_
#define MPC_NONLINEAR_INCLUDE_DEADZONE_INVERSE_DEADZONE_BACKSTEPPING_HPP_

#include <cmath>
#include "utils_act/act_utils.hpp"
#include "autoware_control_toolbox.hpp"
#include "eigen3/Eigen/Core"

namespace ns_deadzone
{

constexpr double hertz2radsec(double x)
{
  return 2. * M_PI * x;
}

struct sExtremumSeekerParams
{
  double K{1.}; // Gain
  double ay{0.05}; // Gain
  double freq_low_pass{1.}; // low-pass filter cut-off frequency
  double freq_high_pass{1.}; // high-pass filter cut-off frequency
  double freq_dither{1.}; // dither signal frequency
  double dt{0.033};
};

struct sDeadZone
{
  // Constructors
  sDeadZone() = default;
  sDeadZone(double const &mr, /*right slope*/
            double const &br,  /*right threshold*/
            double const &ml,
            double const &bl, double const &bmax);

  // Members
  double mr_{1.};
  double mrbr_{0.};

  double ml_{1.};
  double mlbl_{0.};

  double bmax_{0.15};

  double e0_{1e-2}; // param for sigmoid like indicator function for the inverse.

  // Methods
  [[nodiscard]] double get_br() const;
  [[nodiscard]] double get_bl() const;

  [[nodiscard]] double deadzoneOutput(double const &u, double const &Du) const;
  [[nodiscard]] double invDeadzoneOutputSmooth(double const &u, const double &desired_Du) const;
  [[nodiscard]] double invDeadzoneOutput(double const &u, double const &Du) const;

//  [[nodiscard]] double convertInverted_d_minus_u(double const &current_steering,
//                                                 double const &current_steering_cmd,
//                                                 double const &invDu) const;

  void updateCurrentBreakpoints(const double &bs);

};

class ExtremumSeeker
{
 public:

  ExtremumSeeker() = default;
  explicit ExtremumSeeker(sExtremumSeekerParams const &es_params);

  void print() const
  {
    ns_utils::print("ES params wh, wl, wd", wh_, wl_, wd_);
    ns_utils::print("High-pass filter transfer function");
    hpf_tf_.print();

    ns_utils::print("Low-pass filter transfer function");
    lpf_tf_.print();

    ns_utils::print("High-pass filter continuous state-space");
    hpf_ss_.print();

    ns_utils::print("Low-pass filter continuous state-space");
    lpf_ss_.print();
  }

  // Methods
  double getTheta(double const &error);
  [[nodiscard]] double getMeanError() const
  {
    return mean_error_;
  }

 private:
  double K_{1.}; // Gain
  double ay_{0.1}; // Dither sinus amplitude
  double wl_{1.}; // low-pass filter cut-off frequency [rad/sec]
  double wh_{1.}; // high-pass filter cut-off frequency [rad/sec]
  double wd_{1.}; // dither signal frequency
  double dt_{0.033};

  // cumulative time.
  double cum_dt_{};
  double theta_hat_{}; // integrated dither signal.
  double mean_error_{};

  ns_control_toolbox::tf lpf_tf_{}; // low-pass filter
  ns_control_toolbox::tf hpf_tf_{}; // high-pass filter

  ns_control_toolbox::scalarFilters_ss lpf_ss_{}; // low-pass filter
  ns_control_toolbox::scalarFilters_ss hpf_ss_{}; // high-pass filter


};

} // namespace ns_deadzone

#endif //MPC_NONLINEAR_INCLUDE_DEADZONE_INVERSE_DEADZONE_BACKSTEPPING_HPP_
