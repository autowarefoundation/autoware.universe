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

#include "autoware_control_toolbox.hpp"

#include <fmt/core.h>

#include <cassert>

int main()
{
  double Td = 0.01;  // time delay in seconds.
  size_t order = 3;  // order of the Pade approximation.

  auto tf_delay = ns_control_toolbox::pade(Td, order);
  tf_delay.print();

  auto ss_sys = ns_control_toolbox::tf2ss(tf_delay);
  ss_sys.print();

  // Test discretization and compare with Matlab.
  double Ts = 0.1;

  // ss_sys.discretisize(Ts);
  ss_sys.print_discrete_system();

  //
  ns_utils::print("Discretization with a given Ts when constructing");
  Ts = 0.15;
  auto sys_ss2 = ns_control_toolbox::tf2ss(tf_delay, Ts);
  sys_ss2.print();
  sys_ss2.print_discrete_system();

  // Edge case, when the delay = 0.
  Td = 0.0;   // time delay in seconds.
  order = 3;  // order of the Pade approximation.

  tf_delay = ns_control_toolbox::pade(Td, order);
  tf_delay.print();

  if (ns_utils::isEqual(Td, 0.0))
  {
    ns_utils::print("Must throw an error STATIC GAIN .........");
    ss_sys = ns_control_toolbox::tf2ss(tf_delay);
    ss_sys.print();
  }

  return 0;
}
