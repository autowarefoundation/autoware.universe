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

#include <iostream>
#include <vector>

/** The filter tool can be used in the following ways.
 *
 * 1- Defining specs Wp, Ws, Ap, As and obtaining order and cut-off frequency (rad/sec)
 *      ButterworthFilter bf;
 *      bf.Buttord(Wp, Ws, Ap, As); Wp and Ws in [rad/sec]
 *      bf.computeContinuousTimeTF();
 *      bf.computeDiscreteTimeTF();
 *
 *   2- Defining the order and cut-off frequency (rad/sec) directly and computing the filter TFs
 *      bf.setOrder(N); N is integer
 *      bf.setCuttoffFrequency(Wc); Wc in [rad/sec]
 *      bf.computeContinuousTimeTF();
 *      bf.computeDiscreteTimeTF();
 *
 *   3- Defining the order N, cut-off and sampling frequencies (Hz)
 *      bf.setOrder(N); N is integer
 *      bf.setCuttoffFrequency_Hz(fc, fs); cut-off fc and sampling fs are in [Hz]
 *      bf.computeContinuousTimeTF();
 *      bf.computeDiscreteTimeTF();
 * */

int main()
{
  // 1st Method
  double Wp{2.};   // pass-band frequency [rad/sec]
  double Ws{3.};   // stop-band frequency [rad/sec]
  double Ap{6.};   // pass-band ripple mag or loss [dB]
  double As{20.};  // stop band ripple attenuation [dB]

  ButterworthFilter bf;
  bf.Buttord(Wp, Ws, Ap, As);

  auto NWc = bf.getOrderCutOff();
  print("The computed order and frequency for the give specification : ");
  print("Minimum order N = ", NWc.N, ", and The cut-off frequency Wc = ", NWc.Wc, "rad/sec \n");

  /**
   * Approximate the continuous and discrete time transfer functions.
   * */
  bf.computeContinuousTimeTF();

  // Print continuous time roots.
  bf.printFilterContinuousTimeRoots();
  bf.printContinuousTimeTF();

  // Compute the discrete time transfer function.
  bf.computeDiscreteTimeTF();
  bf.printDiscreteTimeTF();

  int a = 1;

  return 0;
}