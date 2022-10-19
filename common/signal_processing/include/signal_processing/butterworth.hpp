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

#ifndef SIGNAL_PROCESSING_INCLUDE_SIGNAL_PROCESSING_BUTTERWORTH_HPP_
#define SIGNAL_PROCESSING_INCLUDE_SIGNAL_PROCESSING_BUTTERWORTH_HPP_

#include <cmath>
#include <complex>
#include <iomanip>
#include <iostream>
#include <vector>

template <typename T>
const T & append_separator(const T & arg)
{
  std::cout << " ";
  return arg;
}

template <typename... Args>
void print(Args &&... args)
{
  (std::cout << ... << append_separator(args)) << "\n";
}

struct sOrderCutoff
{
  int N{};
  double Wc{};
};

struct sDifferenceAnBn
{
  std::vector<double> An;
  std::vector<double> Bn;
};

class ButterworthFilter
{
public:
  // Prints the filter order and cutoff frequency
  void printFilterSpecs() const;
  void printFilterContinuousTimeRoots() const;

  void printContinuousTimeTF() const;
  void printDiscreteTimeTF() const;

  void Buttord(double const & Wp, double const & Ws, double const & Ap, double const & As);

  // Setters and Getters
  void setCuttoffFrequency(double const & Wc);  // Wc is the cut-off frequency in [rad/sec]

  // fc is cut-off frequency in [Hz] and fs is the sampling frequency in [Hz]
  void setCuttoffFrequency(double const & fc, double const & fs);
  void setOrder(int const & N);

  // Get the order, cut-off frequency and other filter properties
  [[nodiscard]] sOrderCutoff getOrderCutOff() const;
  [[nodiscard]] sDifferenceAnBn getAnBn() const;

  [[nodiscard]] std::vector<double> getAn() const;
  [[nodiscard]] std::vector<double> getBn() const;

  // computes continuous time transfer function
  void computeContinuousTimeTF(bool const & use_sampling_frequency = false);

  // computes continuous time transfer function
  void computeDiscreteTimeTF(bool const & use_sampling_frequency = false);

private:
  // member variables
  int order_{1};                      // filter order
  double cutoff_frequency_rad_sec{};  // filter cut-off frequency [rad/sec]

  // Boolean parameter when a sampling frequency is defined. Default is false.
  // bool pre_warp_{};
  double sampling_frequency_hz{1.0};

  const double Td_{2.};  // constant of bilinear transformation

  // Gain of the discrete time function
  std::complex<double> discrete_time_gain_{1.0, 0.0};

  // Continuous time transfer function roots
  std::vector<double> phase_angles_{};
  std::vector<std::complex<double>> continuous_time_roots_{};

  // Discrete time zeros and roots
  std::vector<std::complex<double>> discrete_time_roots_{{0.0, 0.0}};
  std::vector<std::complex<double>> discrete_time_zeros_{{-1.0, 0.0}};

  // Continuous time transfer function numerator denominators
  std::vector<std::complex<double>> continuous_time_denominator_{{0.0, 0.0}};
  double continuous_time_numerator_{0.0};

  // Discrete time transfer function numerator denominators
  std::vector<std::complex<double>> discrete_time_denominator_{{0.0, 0.0}};
  std::vector<std::complex<double>> discrete_time_numerator_{{0.0, 0.0}};

  // Numerator and Denominator Coefficients Bn and An of Discrete Time Filter

  std::vector<double> An_{0.0};
  std::vector<double> Bn_{0.0};

  // METHODS
  // polynomial function returns the coefficients given the roots of a polynomial
  static std::vector<std::complex<double>> poly(std::vector<std::complex<double>> const & roots);

  /*
   * Implementation starts by computing the pole locations of the filter in the
   * polar coordinate system . The algorithm first locates the poles  computing
   * the phase angle and then poles as a complex number From the poles, the
   * coefficients of denominator polynomial is calculated.
   *
   * Therefore, without phase, the roots cannot be calculated. The following
   * three methods should be called successively.
   *
   * */

  // computes the filter root locations in the polar coordinate system
  void computePhaseAngles();

  // Computes continuous time roots from the phase angles
  void computeContinuousTimeRoots(bool const & use_sampling_freqency = false);
};

#endif  // SIGNAL_PROCESSING_INCLUDE_SIGNAL_PROCESSING_BUTTERWORTH_HPP_
