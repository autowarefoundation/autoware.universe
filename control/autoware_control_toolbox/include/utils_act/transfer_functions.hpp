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

#ifndef AUTOWARE_CONTROL_TOOLBOX_TRANSFER_FUNCTIONS_HPP
#define AUTOWARE_CONTROL_TOOLBOX_TRANSFER_FUNCTIONS_HPP

#include "visibility_control.hpp"
#include <vector>
#include "act_utils.hpp"
#include "act_utils_eigen.hpp"
#include "act_definitions.hpp"
#include "tf_algebra.hpp"

namespace ns_control_toolbox
{

	template<typename T, class E>
	class vector_scalar_mult
	{

	public:

		friend E operator*(E const& vec, T const& a)
		{
			E temp{ vec };

			for (auto& x: temp)
			{
				x = x * a;
			}

			return temp;
		}

		friend E& operator*=(E& vec, T const& a)
		{
			for (auto& x: vec)
			{
				x = x * a;
			}

			return vec;
		}
	};


	template<typename T>
	class std_vector_overloaded : public std::vector<T>, vector_scalar_mult<T, std_vector_overloaded<T>>
	{
		using std::vector<T>::vector;
	};


	// Multiplication Operator.

	template<typename T>
	struct TF_multiplication
	{
		friend T operator*(T const& tf1, T const& tf2)
		{
			// Compute numerator multiplication.
			tf_factor num1{{ tf1.num() }};
			tf_factor num2{{ tf2.num() }};

			auto num_mult = num1 * num2;

			// Compute denominator multiplication.
			tf_factor den1{{ tf1.den() }};
			tf_factor den2{{ tf2.den() }};

			tf_factor den_mult = den1 * den2;

			return T{ num_mult(), den_mult() };
		};

		friend T operator*=(T const& tf1, T const& tf2)
		{
			// Compute numerator multiplication.
			tf_factor num1{{ tf1.num() }};
			tf_factor num2{{ tf2.num() }};

			auto num_mult = num1 * num2;

			// Compute denominator multiplication.
			tf_factor den1{{ tf1.den() }};
			tf_factor den2{{ tf2.den() }};

			tf_factor den_mult = den1 * den2;

			return T{ num_mult(), den_mult() };
		};

	};


	// All Operations
	template<typename T>
	struct TF_algebra : public TF_multiplication<T>
	{
	};


	/**
	 * @brief Transfer Function representation in descending power of  Laplace variable "s".
	 * [s^n, s^{n-1} ... s^0]
	 * @param Nn	: size of the numerator array.
	 * @param Nd	: size of the denominator array
	 * */

	// For visibility, we can set for each object : __attribute__((visibility("default"))) class_name
	struct ACT_PUBLIC tf : TF_algebra<tf>
	{

		// Constructors.
		tf() : num_{ 1. }, den_{ 1. }
		{
		}

		// Constructor from non-empty numerator and denominator std::vectors.
		tf(std::vector<double> num, std::vector<double> den)
				: num_{ num }, den_{ den }
		{

			ns_utils::stripVectorZerosFromLeft(num_); // remove zeros from the left.
			ns_utils::stripVectorZerosFromLeft(den_);
		}

		// Constructor from tf_factors
		tf(tf_factor const& num, tf_factor const& den);

		// Member functions.
		/**
		 * @brief : Creates a string stream object of polynomial representation of given the vector,
		 * */
		static size_t getPolynomialStringAndSize(std::vector<double> const& num_or_den,
		                                         std::ostringstream& string_stream);


		void print() const;

		[[nodiscard]] std::vector<double> num() const;

		[[nodiscard]] std::vector<double> den() const;


		// Invert the transfer function.
		void inv();

		// Update num and den
		void update_num(std::vector<double> const& num);

		void update_num(tf_factor const& num);

		void update_den(std::vector<double> const& den);

		void update_den(tf_factor const& den);

		// In some cases, the TF might have variable multiplier of the form a*[num], b*den where a, b vary.
		void update_num_den_coef(double const& num_coeff, double const& den_coeff);

		void update_num_coef(double const& num_coeff);

		void update_den_coef(double const& den_coeff);


	private:

		// Data members
		std::vector<double> num_{ 1. };   // <-@brief numerator
		std::vector<double> den_{ 1. };   // <-@brief denominator

		double num_coef_{ 1. };
		double den_coef_{ 1. };

	};


	/**
	 * @param Td	: time delay value in seconds.
	 * @param N		: Order of Pade approximation.
	 * */
	tf ACT_PUBLIC pade(double const& Td, size_t const& order);

	/**
	 * @bried see pade()
	 * @refitem Golub and Van Loan, Matrix Computations, 4rd edition, Chapter 9., Section 9.3.1 pp 530
	 * */
	tf ACT_PUBLIC padecoeff(double const& Td, size_t const& order);

} // namespace ns_control_toolbox

#endif //AUTOWARE_CONTROL_TOOLBOX_TRANSFER_FUNCTIONS_HPP
