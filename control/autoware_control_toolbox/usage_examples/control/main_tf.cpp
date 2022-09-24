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
#include "utils_act/act_utils.hpp"

int main()
{
  /**
   *  Define a transfer function by its numerator and denominator.
   * */
  std::vector<double> num{1.};
  std::vector<double> den{5.039e-07, 0.00019, 0.02387, 1};

  // Constructor by two vectors.
  ns_control_toolbox::tf sys(num, den);

  // Print the transfer function representation.
  sys.print();

  // The default TF is 1./1.
  ns_utils::print("\n");
  ns_control_toolbox::tf sys_default;
  sys_default.print();

  // One invert the fraction.
  ns_utils::print("Inverse of the TF");
  sys.inv();
  sys.print();

  /**
   * The TF factor class can can be used to represent a numerator or denominator which are equipped with algebraic
   * operations (summation, subtraction and multiplication).
   * */
  ns_control_toolbox::tf_factor ntf1{{1, 0, 0}};
  ns_control_toolbox::tf_factor dtf2{{1, 0, 0.2}};

  auto tf_from_tf_factors = ns_control_toolbox::tf(ntf1, dtf2);
  tf_from_tf_factors.print();

  // Transfer function multiplication.
  ns_control_toolbox::tf tf1{{5}, {1, 0}, 2., 7.};
  ns_control_toolbox::tf tf2{{2}, {1, 1}, 3., 3.};

  auto tf3 = tf1 * tf2;

  ns_utils::print("TF TF multiplication with num den constants ");
  tf3.print();

  // Test vector overloading.
  std::vector<double> ov1{1, 2, 4};

  auto ov2 = ov1 * 7.0;
  auto ov3 = 2. * ov1;

  ns_utils::print_container(std::vector<double>(ov1));
  ns_utils::print_container(std::vector<double>(ov2));
  ns_utils::print_container(std::vector<double>(ov3));

  // Test overloaded a*num, b*num
  ns_control_toolbox::tf tf_overloaded{{2, 0, 4}, {1, 4, 0}};

  ns_utils::print("Before vector scalar multiplication \n");
  tf_overloaded.print();

  tf_overloaded.update_num_coef(10.);
  tf_overloaded.update_den_coef(5.);

  ns_utils::print("After vector scalar multiplication\n");
  tf_overloaded.print();

  ns_utils::print("After inverting by swap \n");
  tf_overloaded.inv();
  tf_overloaded.print();

  // TF-TF multiplication
  auto tfm1 = ns_control_toolbox::tf({1.}, {3., 1});
  auto tfm2 = ns_control_toolbox::tf({2., 2}, {3., 1});

  auto tfm3 = tfm1 * tfm2;

  tfm1.print();
  tfm2.print();
  tfm3.print();

  return 0;
}