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

#include "deadzone/inverse_deadzone_backstepping.hpp"

namespace ns_deadzone
{

sDeadZone::sDeadZone(const double &mr, const double &br, const double &ml,
                     const double &bl, double const &bmax)
  : mr_{mr}, mrbr_{mr * br}, ml_{ml}, mlbl_{-ml * bl}, bmax_{bmax}
{

}

double sDeadZone::get_br() const
{
  return mrbr_ / mr_;
}
double sDeadZone::get_bl() const
{
  return mlbl_ / ml_;
}

/**
 * @brief given a steering deviation, computes the deadzone function output.
 * @param Du = steering - steering_control = δ - u
 * */
double sDeadZone::deadzoneOutput(const double &u, const double &Du) const
{
  auto const &br = get_br();
  auto const &bl = get_bl();

  auto const &indicator_pos = Du >= br ? 1. : 0.;
  auto const &indicator_neg = Du <= bl ? 1. : 0.;
  auto const &u_dz = (u + br) * indicator_pos + (u + bl) * indicator_neg;

  return u_dz;
}

/**
 * @brief given a steering deviation, computes the smooth inverse deadzone function output.
 * @param desired_Du = steering - steering_control = δ - u
 * */

double sDeadZone::invDeadzoneOutputSmooth(double const &u, const double &desired_Du) const
{

  auto x = desired_Du / e0_;

  auto const &phi_r = exp(x) / (exp(x) + exp(-x));
  auto const &phi_l = exp(-x) / (exp(x) + exp(-x));

  // auto const &dz_inv = phi_r * (u - mrbr_) / mr_ + phi_l * (u - mlbl_) / ml_;
  auto const &udz_inv = phi_r * (u - mrbr_) / mr_ + phi_l * (u - mlbl_) / ml_;

  return udz_inv;
}

double sDeadZone::invDeadzoneOutput(double const &u, const double &desired_Du) const
{

  auto const &br = get_br();
  auto const &bl = get_bl();

  auto const &indicator_pos = desired_Du >= 0. ? 1. : 0.;
  auto const &indicator_neg = desired_Du <= 0. ? 1. : 0.;
  auto const &u_dz = (u - br) * indicator_pos + (u - bl) * indicator_neg;
  return u_dz;
}
void sDeadZone::updateCurrentBreakpoints(const double &bs)
{
  auto bsaturated = ns_utils::clamp(bs, 0., bmax_);

  mrbr_ = mr_ * bsaturated;
  mlbl_ = -ml_ * bsaturated;

}

ExtremumSeeker::ExtremumSeeker(sExtremumSeekerParams const &es_params)
  : K_{es_params.K},
    ay_{es_params.ay},
    wl_{hertz2radsec(es_params.freq_low_pass)},
    wh_{hertz2radsec(es_params.freq_high_pass)},
    wd_{hertz2radsec(es_params.freq_dither)},
    dt_{es_params.dt}
{
  auto const &tau_wl = 1. / wl_; // low-pass filter tau
  auto const &tau_wh = 1. / wh_; // high-pass filter tau

  // create the filter transfer functions.
  lpf_tf_ = ns_control_toolbox::tf({1.}, {tau_wl, 1.});
  hpf_tf_ = ns_control_toolbox::tf({1., 0.,}, {tau_wh, 1.});

  // create the discrete state space systems.
  lpf_ss_ = ns_control_toolbox::scalarFilters_ss(lpf_tf_, dt_);
  hpf_ss_ = ns_control_toolbox::scalarFilters_ss(hpf_tf_, dt_);

}

double ExtremumSeeker::getTheta(double const &error)
{
  cum_dt_ += dt_; // for simulating time-varying sinusoidal perturbation.
  // ns_utils::print("In extremum seeker - current time : ", cum_dt_);

  // High-pass filter the error sqr, to remove trends.
  auto const &error_sqr_filt = hpf_ss_.simulateOneStep(error);
  // mean_error_ = (mean_error_ * cum_dt_ + error_sqr_filt) / cum_dt_;

  // Compute the dither signal.
  auto const &dither_sig = ay_ * sin(wd_ * cum_dt_);

  //  auto fmod_remainder = std::fmod(wd_ * cum_dt_, 2. * M_PI);
  //  if (wd_ * cum_dt_ - 20 * M_PI > 0. && fmod_remainder <= 1e-2)
  //  {
  //    cum_dt_ = 0.;
  //  }

  // Low-pass filter the dither signal
  // auto const &xi = lpf_ss_.simulateOneStep(dither_sig * std::sqrt(mean_error_));
  auto const &xi = lpf_ss_.simulateOneStep(dither_sig * error_sqr_filt);

  // Integrate the filtered signal
  theta_hat_ += K_ * xi * dt_;

  // excite theta_hat_;
  auto const &theta = theta_hat_ + ay_ * sin(wd_ * cum_dt_);

  return theta;
}
} // namespace ns_deadzone