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

#include <vector>
#include <string>
#include <algorithm>

#include "utils_act/act_utils.hpp"
#include "autoware_control_toolbox.hpp"


size_t ns_control_toolbox::tf::getPolynomialStringAndSize(std::vector<double> const& num_or_den,
		std::ostringstream& string_stream)
{

	auto Nn        = num_or_den.size();
	int  precision = 4;

	for (size_t k = 0; k < Nn; ++k)
	{

		auto&& power_of_term = Nn - 1 - k;
		auto&& coeff_abs     = std::abs(num_or_den.at(k));

		if (std::fabs(num_or_den.at(k)) > EPS)
		{
			// to put "+" or "-" char in front of "s"
			auto sign_str = num_or_den.at(k) > 0 ? " + " : " - ";

			// k is not the last idx.
			if (k != Nn - 1)
			{
				// if k==0, do not use +s for the first term of the polynomial.
				if (k == 0 && num_or_den.at(k) > 0)
				{
					sign_str = "";
				}

				// If the coefficient is 1 do not print 1s --> instead print s
				if (std::fabs(std::fabs(num_or_den.at(k)) - 1) > EPS)
				{
					string_stream << sign_str << std::setprecision(precision) << coeff_abs;
				}
				else
				{
					string_stream << sign_str;
				}

				// if power is 1 do not print s^1 --> instead s
				if (power_of_term != 1)
				{
					string_stream << " s^" << power_of_term;
				}
				else
				{
					string_stream << " s";
				}

			}
			else
			{               // if k is the last index
				if (Nn > 1)
				{
					string_stream << sign_str << std::setprecision(precision) << coeff_abs;
				}
				else
				{
					string_stream << num_or_den.at(k);
				}

			}
		}
	}
	return string_stream.str().length();
}

void ns_control_toolbox::tf::print() const
{

	std::ostringstream num_string;
	std::ostringstream den_string;

	auto num_string_size = getPolynomialStringAndSize(num_, num_string);
	auto den_string_size = getPolynomialStringAndSize(den_, den_string);

	if (num_string_size > den_string_size)
	{
		size_t num_of_tab = (num_string_size - den_string_size) / 4;
		ns_utils::print(num_string.str());
		ns_utils::print(std::string(std::max(num_string_size, den_string_size), '-'));
		ns_utils::print(std::string(num_of_tab, ' '), den_string.str());

	}
	else if (num_string_size < den_string_size)
	{
		size_t num_of_tab = (den_string_size - num_string_size) / 4;
		ns_utils::print(std::string(num_of_tab, ' '), num_string.str());
		ns_utils::print(std::string(std::max(num_string_size, den_string_size), '-'));
		ns_utils::print(den_string.str());

	}
	else
	{

		ns_utils::print(num_string.str());
		ns_utils::print(std::string(std::max(num_string_size, den_string_size), '-'));
		ns_utils::print(den_string.str());
	}

	ns_utils::print("\nContinuous-time transfer function.\n");

}


ns_control_toolbox::tf2ss::tf2ss()
		: A_{ Eigen::MatrixXd(1, 1) },
		  B_{ Eigen::MatrixXd(1, 1) },
		  C_{ Eigen::MatrixXd(1, 1) },
		  D_{ Eigen::MatrixXd(1, 1) },
		  Ad_{ Eigen::MatrixXd(1, 1) },
		  Bd_{ Eigen::MatrixXd(1, 1) },
		  Cd_{ Eigen::MatrixXd(1, 1) },
		  Dd_{ Eigen::MatrixXd(1, 1) }
{
	A_.setIdentity();
	B_.setIdentity();
	C_.setIdentity();
	D_.setZero();

	Ad_.setIdentity();
	Bd_.setIdentity();
	Cd_.setIdentity();
	Dd_.setZero();
	// ns_eigen_utils::printEigenMat(A_);
}

ns_control_toolbox::tf2ss::tf2ss(const ns_control_toolbox::tf& sys_tf)
{

	auto num = sys_tf.num();
	auto den = sys_tf.den();

	// Check the leading zeros to determine the order of the system, and strip the leading zeros.
	ns_utils::stripVectorZerosFromLeft(num);
	ns_utils::stripVectorZerosFromLeft(den);

	// Compare if the system is a proper.
	if (den.size() < num.size())
	{
		throw std::invalid_argument("This system is not a proper system.");
	}

	// Compute the system matrices.
	computeSystemMatrices(num, den);

}

ns_control_toolbox::tf2ss::tf2ss(const std::vector<double>& numerator,
		const std::vector<double>& denominator)
{
	auto num = numerator;
	auto den = denominator;

	// Check the leading zeros to determine the order of the system, and strip the leading zeros.
	ns_utils::stripVectorZerosFromLeft(num);
	ns_utils::stripVectorZerosFromLeft(den);

	// Compare if the system is a proper.
	if (den.size() < num.size())
	{
		throw std::invalid_argument("This system is not a proper system.");
	}

	// Compute the system matrices.
	computeSystemMatrices(num, den);

}


void ns_control_toolbox::tf2ss::computeSystemMatrices(
		const std::vector<double>& num,
		const std::vector<double>& den)
{
	auto n_order = static_cast<long>(den.size() - 1);       // Order of the system.

	// We can put system check function if the n_order = 0 -- i.e throw exception.

	A_ = Eigen::MatrixXd::Zero(n_order, n_order);
	C_ = Eigen::MatrixXd::Zero(1, n_order);
	B_ = Eigen::MatrixXd::Identity(n_order, 1);
	D_ = Eigen::MatrixXd::Zero(1, 1);

	// set discrete matrix sizes
	Ad_.resize(n_order, n_order);
	Bd_.resize(1, n_order);
	Cd_.resize(n_order, 1);
	Dd_.resize(1, 1);

	// Zero padding the numerator.
	auto num_of_zero = den.size() - num.size();

	std::vector<double> zero_padded_num{ num };

	if (num_of_zero > 0)
	{
		zero_padded_num = ns_utils::zero_pad_left_first_arg(num, den);
	}


	// Normalize the numerator and denominator
	auto&& den_first_item = den[0];
	std::vector<double> normalized_den{ den };



	// normalize the numerator
	if (std::fabs(den_first_item) > EPS)
	{
		std::transform(zero_padded_num.begin(), zero_padded_num.end(), zero_padded_num.begin(),
				[&den_first_item](auto x)
				{ return x / den_first_item; });

		std::transform(normalized_den.begin(), normalized_den.end(), normalized_den.begin(),
				[&den_first_item](auto x)
				{ return x / den_first_item; });
	}
	else
	{
		throw std::invalid_argument("The first item in the denominator cannot be zero ...");
	}


	if (n_order > 0)
	{
		D_(0, 0) = zero_padded_num[0];
	}

	//	auto B = Eigen::MatrixXd::Identity(n_order - 1, n_order);
	//	ns_eigen_utils::printEigenMat(B);

	if (n_order > 1)
	{
		A_.bottomRows(n_order - 1) = Eigen::MatrixXd::Identity(n_order - 1, n_order);
	}


	// normalize the denominator and assign the first row of the A_matrix to the normalized denominator's values
	// excluding the first item of the denominator.
	for (size_t k = 1; k < normalized_den.size(); k++)
	{
		Eigen::Index ind_eig{ static_cast<long>(k - 1) };
		A_(0, ind_eig) = -1 * normalized_den[k];
		C_(0, ind_eig) = zero_padded_num[k] - zero_padded_num[0] * normalized_den[k];
	}


	bool debug = false;
	if (debug)
	{
//				ns_utils::print("A : \n_order");
//				ns_eigen_utils::printEigenMat(A_);
//
//        ns_utils::print("B : \n_order");
//        ns_eigen_utils::printEigenMat(B_);
//
//        ns_utils::print("C : \n_order");
//        ns_eigen_utils::printEigenMat(C_);
//
//				ns_utils::print("D : \n_order");
//				ns_eigen_utils::printEigenMat(D_);
		ns_utils::print("Zero padded vector");
		ns_utils::print_container(zero_padded_num);

	}

}

void ns_control_toolbox::tf2ss::print() const
{

	ns_utils::print("A : \n");
	ns_eigen_utils::printEigenMat(A_);

	ns_utils::print("B : \n");
	ns_eigen_utils::printEigenMat(B_);

	ns_utils::print("C : \n");
	ns_eigen_utils::printEigenMat(C_);

	ns_utils::print("D : \n");
	ns_eigen_utils::printEigenMat(D_);
}


void ns_control_toolbox::tf2ss::print_discrete_system() const
{

	ns_utils::print("Ad : \n");
	ns_eigen_utils::printEigenMat(Ad_);

	ns_utils::print("Bd : \n");
	ns_eigen_utils::printEigenMat(Bd_);

	ns_utils::print("Cd : \n");
	ns_eigen_utils::printEigenMat(Cd_);

	ns_utils::print("Dd : \n");
	ns_eigen_utils::printEigenMat(Dd_);
}

/**
 * @brief Discretisize the continuous time state-space model using Tustin approximation.
 * */
void ns_control_toolbox::tf2ss::discretisize(double const& Ts)
{

	auto&& n = A_.rows();

	// take inverse:
	auto const&& I = Eigen::MatrixXd::Identity(n, n);

	auto const&& mat1_ATs = I - A_ * Ts / 2.;

	auto const&& inv1_ATs = mat1_ATs.inverse();

	Ad_ = inv1_ATs * (I + A_ * Ts / 2.);
	Bd_ = inv1_ATs * B_ * Ts;
	Cd_ = C_ * inv1_ATs;
	Dd_ = D_ + C_ * Bd_ / 2.;

	//	ns_utils::print("invA \n");
	//	ns_eigen_utils::printEigenMat(inv1_ATs);

}

ns_control_toolbox::tf ns_control_toolbox::padecoeff(const double& Td, const size_t& order)
{
	if (std::fabs(Td) <= EPS)
	{
		return {};             // constant tf.
	}

	auto&& n = order + 1;
	std::vector<double> num(n, 0.0);
	std::vector<double> den(n, 0.0);

	num.back() = 1.;
	den.back() = 1.;

	// Double conversion of order
	double order_double_t{ static_cast<double>(order) };

	for (size_t k = 1; k < n; ++k)
	{
		double k_double_t{ static_cast<double>(k) };
		double fact = Td * (order_double_t - k_double_t + 1) / (2. * order_double_t - k_double_t + 1) /
		              k_double_t;
		num[n - 1 - k] = -fact * num[n - k];
		den[n - 1 - k] = fact * den[n - k];
	}

	auto const den0 = den[0];
	std::transform(num.begin(), num.end(), num.begin(),
			[&den0](auto& x)
			{ return x / den0; });

	std::transform(den.begin(), den.end(), den.begin(),
			[&den0](auto& x)
			{ return x / den0; });

	return { num, den };
}

ns_control_toolbox::tf ns_control_toolbox::pade(const double& Td, const size_t& order)
{
	return padecoeff(Td, order);

}
