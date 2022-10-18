// Copyright 2022 Tier IV, Inc.
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

#include "signal_processing/butterworth.hpp"

/**
 *  @brief Computes the minimum of an analog Butterworth filter order and cut-off frequency give
 *  the pass and stop-band frequencies and ripple magnitude (tolerances).
 *  @param Wp [in] pass-band frequency [rad/sc],
 *  @param Ws [in] stop-band frequency [rad/sc],
 *  @param Ap [in] pass-band ripple [dB]
 *  @param As [in] stop-band ripple [dB]
 * */

void ButterworthFilter::Buttord(
  const double & Wp, const double & Ws, const double & Ap, const double & As)
{
  // N*ln(alpha) > ln(beta)

  auto alpha = Ws / Wp;
  auto beta = sqrt((pow(10, As / 10.0) - 1.0) / (pow(10, Ap / 10.0) - 1.0));
  auto order = static_cast<int>(std::ceil(log(beta) / log(alpha)));

  setOrder(order);

  // right limit, left limit
  /**
   * The left and right limits of the magnitudes satisfy the specs at the
   * frequencies Ws and Wp Scipy.buttord gives left limit as the cut-off
   * frequency whereas Matlab gives right limit
   * */

  double right_lim = Ws * (pow((pow(10.0, As / 10.0) - 1.0), -1.0 / (2. * order)));
  // double left_lim = Wp * (pow((pow(10.0, Ap / 10.0) - 1.0), -1.0 / (2. * order)));

  setCuttoffFrequency(right_lim);
}

void ButterworthFilter::setOrder(int const & N) { order_ = N; }

void ButterworthFilter::setCuttoffFrequency(const double & Wc) { cutoff_frequency_ = Wc; }

sOrderCutoff ButterworthFilter::getOrderCutOff() const
{
  return sOrderCutoff{order_, cutoff_frequency_};
}
void ButterworthFilter::computeContinuousTimeTF(const bool & use_sampling_frequency)
{
  // First compute  the phase angles of the roots
  computePhaseAngles();
  computeContinuousTimeRoots(use_sampling_frequency);

  continuous_time_denominator_ = poly(continuous_time_roots_);
  continuous_time_numerator_ = std::pow(cutoff_frequency_, order_);
}

void ButterworthFilter::computePhaseAngles()
{
  phase_angles_.resize(order_, 0.);
  int k{1};

  for (auto & x : phase_angles_) {
    x = M_PI_2 + (M_PI * (2.0 * k - 1.0) / (2.0 * order_));
    k++;
    print("Phase angle x = ", x);
  }
}

void ButterworthFilter::computeContinuousTimeRoots(const bool & use_sampling_freqency)
{
  continuous_time_roots_.resize(order_, {0.0, 0.0});
  int k{};

  if (use_sampling_freqency) {
    print("\n Sampling Frequency is used to compute pre-warped frequency \n");

    double const & Fc =
      (sampling_frequency_ / M_PI) * tan(cutoff_frequency_ / (sampling_frequency_ * 2.0));

    for (auto & x : continuous_time_roots_) {
      x = {cos(phase_angles_[k]) * Fc * 2.0 * M_PI, sin(phase_angles_[k]) * Fc * 2.0 * M_PI};
      k++;
    }

  } else {
    for (auto & x : continuous_time_roots_) {
      x = {cutoff_frequency_ * cos(phase_angles_[k]), cutoff_frequency_ * sin(phase_angles_[k])};
      k++;
    }
  }
}
std::vector<std::complex<double>> ButterworthFilter::poly(
  std::vector<std::complex<double>> const & roots)
{
  std::vector<std::complex<double>> coefficients(roots.size() + 1, {0, 0});

  int const n{static_cast<int>(roots.size())};

  coefficients[0] = {1.0, 0.0};

  for (int i = 0; i < n; i++) {
    for (int j = i; j != -1; j--) {
      coefficients[j + 1] = coefficients[j + 1] - (roots[i] * coefficients[j]);
    }
  }

  return coefficients;
}

/**
 *  @brief Prints the order and cut-off angular frequency (rad/sec) of the filter
 * */

void ButterworthFilter::printFilterContinuousTimeRoots() const
{
  print("\n The roots of the continuous-time filter Transfer Function's Denominator are : \n");

  for (auto const & x : continuous_time_roots_) {
    print(std::real(x), std::imag(x) < 0 ? " - " : " + ", " j", std::abs(std::imag(x)));
  }
}
