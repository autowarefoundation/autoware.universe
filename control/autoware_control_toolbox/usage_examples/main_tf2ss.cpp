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
#include "utils_act/act_utils.hpp"
#include "utils_act/state_space.hpp"

#include <cassert>

int main()
{
  double Ts = 0.1;  // sampling time for discretization.
  std::vector<double> num{1.};
  std::vector<double> den{5.039e-07, 0.00019, 0.02387, 1};

  // With a num, den
  ns_control_toolbox::tf sys_tf(num, den);

  // Print sys
  sys_tf.print();

  //  Default constructor example
  // ns_control_toolbox::tf2ss sys_ss; args : sys, Ts with Ts is defaulted = 0.1
  ns_control_toolbox::tf2ss sys_ss1(sys_tf);
  sys_ss1.print();

  // Constructor with num and den
  // args : sys, Ts with Ts is defaulted = 0.1
  ns_control_toolbox::tf2ss sys_ss2(num, den, Ts);
  sys_ss2.print();

  ns_utils::print("SS.A \n");
  ns_eigen_utils::printEigenMat(sys_ss2.A());

  ns_utils::print("SS.Ad \n");
  ns_eigen_utils::printEigenMat(sys_ss2.Ad());

  // DISCRETIZATION : Note sstf2 automatically discretisize
  Ts = 0.05;
  sys_ss2.print_discrete_system();

  // Test defaulted.
  ns_utils::print("Discretization with a given Ts when constructing");
  auto sys_ss3 = ns_control_toolbox::tf2ss(sys_tf, Ts);
  sys_ss3.print_discrete_system();

  // Testing qfilter example
  double dt = 1. / 40;
  int order_q = 3;
  double frq_q = 20;  // Hz
  double wc = 2 * M_PI * frq_q;
  auto tau_q = 1 / wc;
  ns_control_toolbox::tf_factor den_q{{tau_q, 1}};
  den_q.power(order_q);

  ns_utils::print("\nQfilter Transfer Function \n");
  auto Qtf = ns_control_toolbox::tf({1.}, {den_q()});
  Qtf.print();

  ns_control_toolbox::tf2ss Qss(Qtf, dt);

  ns_utils::print("\nQfilter Continuous State Space \n");
  Qss.print();

  ns_utils::print("\nQfilter Discrete State Space \n");
  Qss.print_discrete_system();

  // Test on the vehicle model.

  return 0;
}