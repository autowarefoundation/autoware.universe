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

#ifndef AUTOWARE_CONTROL_TOOLBOX_TF_ALGEBRA_HPP
#define AUTOWARE_CONTROL_TOOLBOX_TF_ALGEBRA_HPP

#include <cstddef>
#include <iostream>
#include <array>
#include <utility>
#include <vector>
#include <sstream>
#include <iomanip>
#include <limits>
#include <algorithm>
#include <boost/optional.hpp>

// BOOST POLYNOMIAL
#include <boost/math/tools/polynomial.hpp>

// Library headers
#include "utils_act/act_utils_eigen.hpp"
#include "utils_act/act_utils.hpp"
#include "visibility_control.hpp"
#include "utils_act/act_definitions.hpp"


namespace ns_control_toolbox
{

	//using namespace boost::math;
	using boost_polynomial = boost::math::tools::polynomial<double>; // for polynomial

	// OPERATORS
	template<typename T>
	struct addition_tf_factors
	{

		friend T operator+(T const& lhs, T const& rhs)
		{
			// zero pad shorter vector from left.
			auto tf_factor_vec_zpadded = ns_utils::zero_pad_left_make_equal(lhs(), rhs());
			auto&& v1 = tf_factor_vec_zpadded[0];
			auto&& v2 = tf_factor_vec_zpadded[1];

			std::transform(v1.cbegin(), v1.cend(), v2.cbegin(), v1.begin(), [](auto it1, auto const& it2)
			{
				return it1 + it2;
			});
			return T(v1);
		}


		friend T operator+=(T const& lhs, T const& rhs)
		{
			// zero pad shorter vector from left.
			auto tf_factor_vec_zpadded = ns_utils::zero_pad_left_make_equal(lhs(), rhs());
			auto&& v1 = tf_factor_vec_zpadded[0];
			auto&& v2 = tf_factor_vec_zpadded[1];

			std::transform(v1.cbegin(), v1.cend(), v2.cbegin(), v1.begin(), [](auto it1, auto const& it2)
			{
				return it1 + it2;
			});
			return T(v1);
		}
	};

	template<typename T>
	struct subtraction_tf_factors
	{
		friend T operator-(T const& lhs, T const& rhs)
		{
			// zero pad shorter vector from left.
			auto tf_factor_vec_zpadded = ns_utils::zero_pad_left_make_equal(lhs(), rhs());
			auto&& v1 = tf_factor_vec_zpadded[0];
			auto&& v2 = tf_factor_vec_zpadded[1];

			std::transform(v1.cbegin(), v1.cend(), v2.cbegin(), v1.begin(), [](auto it1, auto const& it2)
			{
				return it1 - it2;
			});
			return T(v1);
		}


		friend T operator-=(T const& lhs, T const& rhs)
		{
			// zero pad shorter vector from left.
			auto tf_factor_vec_zpadded = ns_utils::zero_pad_left_make_equal(lhs(), rhs());
			auto&& v1 = tf_factor_vec_zpadded[0];
			auto&& v2 = tf_factor_vec_zpadded[1];

			std::transform(v1.cbegin(), v1.cend(), v2.cbegin(), v1.begin(), [](auto it1, auto const& it2)
			{
				return it1 - it2;
			});
			return T(v1);
		}
	};

	template<typename T>
	struct multiplication_tf_factors
	{

		friend T operator*(T const& lhs, T const& rhs)
		{
			auto&& v1 = lhs(); // we store in tf classes as x^3, x^2 ... 1
			auto&& v2 = rhs();

			boost_polynomial p1(v1.rbegin(), v1.rend()); // Boost stores 1, x, x^2 ... x^n
			boost_polynomial p2(v2.rbegin(), v2.rend());

			auto&& p3 = p1 * p2;

			// Create a vector from the multiplication data.
			std::vector<double> vresults(p3.data());

			// Reverse
			std::reverse(vresults.begin(), vresults.end());

			return T(vresults);
		}


		friend T operator*=(T const& lhs, T const& rhs)
		{
			auto&& v1 = lhs(); // we store in tf classes as x^3, x^2 ... 1
			auto&& v2 = rhs();

			boost_polynomial p1(v1.rbegin(), v1.rend()); // Boost stores 1, x, x^2 ... x^n
			boost_polynomial p2(v2.rbegin(), v2.rend());

			auto&& p3 = p1 * p2;

			// Create a vector from the multiplication data.
			std::vector<double> vresults(p3.data());

			// Reverse
			std::reverse(vresults.begin(), vresults.end());

			return T(vresults);
		}
	};

	template<typename T>
	struct division_tf_factors
	{

		friend T operator/(T const& lhs, T const& rhs)
		{
			auto&& v1 = lhs(); // we store in tf classes as x^3, x^2 ... 1
			auto&& v2 = rhs();

			boost_polynomial p1(v1.rbegin(), v1.rend()); // Boost stores 1, x, x^2 ... x^n
			boost_polynomial p2(v2.rbegin(), v2.rend());

			auto&& p3 = p1 / p2;

			// Create a vector from the multiplication data.
			std::vector<double> vresults(p3.data());

			// Reverse
			std::reverse(vresults.begin(), vresults.end());

			return T(vresults);
		}


		friend T operator/=(T const& lhs, T const& rhs)
		{

			auto&& v1 = lhs(); // we store in tf classes as x^3, x^2 ... 1
			auto&& v2 = rhs();

			boost_polynomial p1(v1.rbegin(), v1.rend()); // Boost stores 1, x, x^2 ... x^n
			boost_polynomial p2(v2.rbegin(), v2.rend());

			auto&& p3 = p1 / p2;

			// Create a vector from the multiplication data.
			std::vector<double> vresults(p3.data());

			// Reverse
			std::reverse(vresults.begin(), vresults.end());

			return T(vresults);
		}
	};

	template<typename T>
	struct tf_factor_arithmetic : addition_tf_factors<T>,
	                              subtraction_tf_factors<T>,
	                              multiplication_tf_factors<T> //, division_tf_factors
	{
	};


	/**
	 * @brief Structs equipped with polynomial addition and multiplication property for the denominator and numerator
	 * operations of the transfer function objects.
	 * */
	struct tf_factor : tf_factor_arithmetic<tf_factor>
	{
	public:

		tf_factor() = default;

		explicit tf_factor(std::vector<double> factor) : factor_{ std::move(factor) }
		{
		}

		std::vector<double> operator()() const
		{
			return factor_;
		}

		void power(unsigned int const& n)
		{

			if (n > 10)
			{
				throw std::invalid_argument("the power is too big than expected ...");
			}
			auto&& v1 = factor_; // we store in tf classes as x^3, x^2 ... 1


			boost_polynomial p0{ 1. };
			boost_polynomial p1(v1.rbegin(), v1.rend()); // Boost stores 1, x, x^2 ... x^n

			for (size_t k = 0; k < n; ++k)
			{
				p0 *= p1;
			}

			// Create a vector from the multiplication data.
			std::vector<double> vresults(p0.data());

			// Reverse
			std::reverse(vresults.begin(), vresults.end());

			factor_ = vresults;
		}

	private:
		std::vector<double> factor_{ 1. };
	};


} // namespace ns_control_toolbox

#endif //AUTOWARE_CONTROL_TOOLBOX_TF_ALGEBRA_HPP
