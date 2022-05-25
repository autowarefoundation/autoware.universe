// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "autoware_control_toolbox.hpp"

void print(std::vector<double> const& x)
{
	for (auto y: x)
	{
		std::cout << y << " \t";
	}

	std::cout << std::endl;
}


int main()
{
	std::vector<double> a{ 1, 4 };
	std::vector<double> b{ 3, 8, 7, 1.2 };

	// Create two tfs
	ns_control_toolbox::tf_factor tf1(a);
	ns_control_toolbox::tf_factor tf2(b);

	// Add two tf factors.
	auto tf3 = tf1 + tf2;
	print(tf3());

	// Subtract two factors
	auto tf4 = tf1 - tf2;
	print(tf4());


	// BOOST POLYNOMIAL stores  1, x, x^2 ... x^n
	// We need to reverse it for our tf_factor operations.

	ns_control_toolbox::boost_polynomial const b1{{ 10, -6, -4, 3 }};
	ns_control_toolbox::boost_polynomial const b2{{ -2, 1 }};

	auto c1 = b1 * b2;

	ns_utils::print("\nBoost Storage : \n");
	for (size_t i = 0; i < c1.size(); ++i)
	{
		std::cout << c1[i] << '\t';
	}
	std::cout << std::endl;

	// formula_format() converts from Boost storage to human notation.
	auto xx = std::vector<double>(c1.data());
	std::reverse(xx.begin(), xx.end());

	ns_utils::print("\nReversed Boost Storage : \n");
	print(xx);

	// USING tf1*tf2 multiplication
	ns_utils::print("\n Multiplication of tf1, tf2 \n");
	ns_utils::print("Matlab Results  : 3 s^4 + 20 s^3 + 39 s^2 + 29.2 s + 4.8 \n");

	auto tf5 = tf1 * tf2;
	// auto tf6 = tf5 * tf2;
	print(tf5());

	// Power of tf
	ns_control_toolbox::tf_factor tf7({ 0.2, 1.1 });
	tf7.power(3);

	ns_utils::print("\n Power of a TF  \n");
	print(tf7());

	// Test state space model conversion.
	double                        tau   = 0.008;
	unsigned int                  order = 2;
	ns_control_toolbox::tf_factor denominator({ tau, 1 });
	denominator.power(order);

	ns_utils::print("\n Power of a TF8  \n");
	print(denominator());

	ns_control_toolbox::tf    tf8{{ 1. }, denominator() };
	ns_control_toolbox::tf2ss ss(tf8);
	ss.print();


	return 0;
}