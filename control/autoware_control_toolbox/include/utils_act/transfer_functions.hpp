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

namespace ns_control_toolbox
{


/**
 * @brief Transfer Function representation in descending power of  Laplace variable "s".
 * [s^n, s^{n-1} ... s^0]
 * @param Nn	: size of the numerator array.
 * @param Nd	: size of the denominator array
 * */

	// For visibility we can set for each object : __attribute__((visibility("default"))) class_name
	struct ACT_PUBLIC tf
		{

		// Constructors.
		tf() : num_{ 1. }, den_{ 1. }
		{
		}

		// Constructor from non-empty numerator and denominator std::vectors.
		tf(std::vector<double> num, std::vector<double> den)
				: num_{ std::move(num) }, den_{ std::move(den) }
		{

			ns_utils::stripVectorZerosFromLeft(num_); // remove zeros from the left.
			ns_utils::stripVectorZerosFromLeft(den_);
		}


		// Member functions.
		/**
		 * @brief : Creates a string stream object of polynomial representation of given the vector,
		 * */
		static size_t getPolynomialStringAndSize(std::vector<double> const& num_or_den,
				std::ostringstream& string_stream);


		void print() const;

		[[nodiscard]] std::vector<double> num() const
		{
			return num_;
		}

		[[nodiscard]] std::vector<double> den() const
		{
			return den_;
		}

	private:

		// Data members
		std::vector<double> num_;   // <-@brief numerator
		std::vector<double> den_;   // <-@brief denominator
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
