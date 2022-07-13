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

#include "autoware_control_toolbox.hpp"

int main()
{
  double tfinal = 10.;
  double dt = 1. / 40.;
  auto time_vec = ns_control_toolbox::make_time_signal(dt, tfinal);

  ns_utils::print("Time points");
  ns_eigen_utils::printEigenMat(time_vec);

  // Creating a sinusoidal points.
  double frequency_hz = 1;

  auto sin_signal = ns_control_toolbox::make_sinus_signal(time_vec, frequency_hz);
  ns_utils::print("Sinusoidal Signal");
  ns_eigen_utils::printEigenMat(sin_signal);

  // test triangle wave
  auto triangle_wave = ns_control_toolbox::make_triangle_signal(time_vec, frequency_hz);
  ns_eigen_utils::printEigenMat(triangle_wave);
  return 0;
}
