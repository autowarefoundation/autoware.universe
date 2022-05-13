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

#ifndef AUTOWARE_CONTROL_TOOLBOX_HPP_
#define AUTOWARE_CONTROL_TOOLBOX_HPP_

#include <cstddef>
#include <iostream>
#include <array>
#include <utility>
#include <vector>
#include <sstream>
#include <iomanip>
#include <limits>
#include <boost/optional.hpp>
#include "utils/act_utils_eigen.hpp"
#include "visibility_control.hpp"

namespace ns_control_toolbox
{

	double constexpr EPS = std::numeric_limits<double>::epsilon();

/**
 * @brief Transfer Function representation in descending power of  Laplace variable "s".
 * [s^n, s^{n-1} ... s^0]
 * @param Nn	: size of the numerator array.
 * @param Nd	: size of the denominator array
 * */

	struct __attribute__((visibility("default"))) tf
		{
		// Constructors.
		tf() : num_{ 1. }, den_{ 1. }
		{
		}

		// Constructor from non-empty numerator and denominator std::vectors.
		tf(std::vector<double> num, std::vector<double> den)
				: num_{ std::move(num) }, den_{ std::move(den) }
		{
		}

		// Copy constructor.
		tf(tf const& other)
		{
			num_ = other.num_;
			den_ = other.den_;
		}

		tf(tf&& other)

		noexcept
		{
			num_ = std::move(other.num_);
			den_ = std::move(other.den_);
		}

		tf& operator=(tf const& other)
		{
			if (this != &other)
			{
				num_ = other.num_;
				den_ = other.den_;
			}
			return *this;
		}

		tf& operator=(tf&& other) noexcept
		{
			if (this != &other)
			{
				num_ = other.num_;
				den_ = other.den_;
			}
			return *this;
		}

		// Member functions.
		/**
		 * @brief : Creates a string stream object of polynomial representation of given the vector,
		 * */
		static size_t getPolynomialStringAndSize(std::vector<double> const& num_or_den,
				std::ostringstream& string_stream);


		void print() const;

		// Data members
		std::vector<double> num_;   // <-@brief numerator
		std::vector<double> den_;   // <-@brief denominator
		};

/**
 * @brief tf2ss Converts a transfer function representation in to a state-space form.
 * We assume the system is SISO type.
 *
 * */

	struct __attribute__((visibility("default"))) tf2ss
		{
		tf2ss();

		explicit tf2ss(tf const& sys_tf);

		tf2ss(std::vector<double> const& num, std::vector<double> const& den);

		// Copy constructor
		tf2ss(tf2ss const& other)
		{
			A_ = other.A_;
			B_ = other.B_;
			C_ = other.C_;
			D_ = other.D_;
		}

		tf2ss& operator=(tf2ss const& other)
		{
			if (this != &other)
			{
				A_ = other.A_;
				B_ = other.B_;
				C_ = other.C_;
				D_ = other.D_;
			}
			return *this;
		}

		void print() const;

		// Data members Eigen Matrices.
		Eigen::MatrixXd A_;
		Eigen::MatrixXd B_;
		Eigen::MatrixXd C_;
		Eigen::MatrixXd D_;

	private:
		/**
		 * @brief some numerator and denominator can be defined by leading zeros like [0, 0, 1], and we want to strip
		 * away the leading zeros.
		 * */
		static void checkOrderAndStrip(std::vector<double>& num_or_den);

		/**
		 * @brief Compute the system Matrices
		 * */

		void computeSystemMatrices(std::vector<double> const& num,
				std::vector<double> const& den);

		};

	/**
	 * @param Td	: time delay value in seconds.
	 * @param N		: Order of Pade approximation.
	 * */
	tf __attribute__((visibility("default"))) pade(double const& Td, size_t const& order);

	/**
	 * @bried see pade()
	 * @refitem Golub and Van Loan, Matrix Computations, 4rd edition, Chapter 9., Section 9.3.1 pp 530
	 * */
	tf __attribute__((visibility("default"))) padecoeff(double const& Td, size_t const& order);

} // namespace ns_control_toolbox
#endif // AUTOWARE_CONTROL_TOOLBOX_HPP_
