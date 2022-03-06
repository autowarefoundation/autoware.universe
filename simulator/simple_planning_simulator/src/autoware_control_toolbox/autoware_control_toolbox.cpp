//
// Created by ali on 18/2/22.
//

#include <vector>
#include <string>
#include <algorithm>
#include "autoware_control_toolbox/utils/act_utils.hpp"
#include "autoware_control_toolbox/autoware_control_toolbox.hpp"

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
		D_{ Eigen::MatrixXd(1, 1) }
{
	A_.setIdentity();
	B_.setIdentity();
	C_.setIdentity();
	D_.setZero();
	// ns_eigen_utils::printEigenMat(A_);
}

ns_control_toolbox::tf2ss::tf2ss(const ns_control_toolbox::tf& sys_tf)
{
	auto num = sys_tf.num_;
	auto den = sys_tf.den_;

	// Check the leading zeros to determine the order of the system, and strip the leading zeros.
	checkOrderAndStrip(num);
	checkOrderAndStrip(den);

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
	checkOrderAndStrip(num);
	checkOrderAndStrip(den);

	// Compare if the system is a proper.
	if (den.size() < num.size())
	{
		throw std::invalid_argument("This system is not a proper system.");
	}

	// Compute the system matrices.
	computeSystemMatrices(num, den);

}

void ns_control_toolbox::tf2ss::checkOrderAndStrip(std::vector<double>& num_or_den)
{

	for (auto it = num_or_den.begin(); it != num_or_den.end();)
	{
		auto const& value_of_it = *it;
		if (std::fabs(value_of_it) <= EPS)
		{
			num_or_den.erase(it);
		}
		else
		{
			return;
		}
	}

}

void ns_control_toolbox::tf2ss::computeSystemMatrices(
		const std::vector<double>& num,
		const std::vector<double>& den)
{
	auto n = static_cast<long>(den.size() - 1);       // Order of the system.

	// We can put system check function if the n = 0 -- i.e throw exception.

	A_ = Eigen::MatrixXd::Zero(n, n);
	C_ = Eigen::MatrixXd::Zero(1, n);
	B_ = Eigen::MatrixXd::Identity(n, 1);
	D_ = Eigen::MatrixXd::Zero(1, 1);

	// Zero padding the numerator.
	auto num_of_zero = den.size() - num.size();

	std::vector<double> zero_padded_num;
	if (num_of_zero > 0)
	{
		zero_padded_num = std::vector<double>(num_of_zero, 0.);
		zero_padded_num.insert(zero_padded_num.end(), num.begin(), num.end());
	}
	else
	{
		zero_padded_num = num;
	}

	// Normalize the numerator and denominator
	auto&& den_first_item = den[0];
	std::vector<double> normalized_den = den;

	std::transform(normalized_den.begin(), normalized_den.end(), normalized_den.begin(),
			[den_first_item](auto x)
			{ return x / den_first_item; });

	std::transform(zero_padded_num.begin(), zero_padded_num.end(), zero_padded_num.begin(),
			[den_first_item](auto x)
			{ return x / den_first_item; });

	if (n > 0)
	{
		D_(0, 0) = zero_padded_num[0];
	}

	//	auto B = Eigen::MatrixXd::Identity(n - 1, n);
	//	ns_eigen_utils::printEigenMat(B);

	if (n > 1)
	{
		A_.bottomRows(n - 1) = Eigen::MatrixXd::Identity(n - 1, n);
	}


	// normalize the denominator and assign the first row of the A_matrix to the normalized denominator's values
	// excluding the first item of the denominator.

	for (size_t k = 1; k < normalized_den.size(); k++)
	{
		Eigen::Index ind_eig{ static_cast<long>(k - 1) };
		A_(0, ind_eig) = -1 * normalized_den[k];
		C_(0, ind_eig) = zero_padded_num[k] - zero_padded_num[0] * normalized_den[k];
	}

//        ns_utils::print("A : \n");
//        ns_eigen_utils::printEigenMat(A_);
//
//        ns_utils::print("B : \n");
//        ns_eigen_utils::printEigenMat(B_);
//
//        ns_utils::print("C : \n");
//        ns_eigen_utils::printEigenMat(C_);
//
//        ns_utils::print("D : \n");
//        ns_eigen_utils::printEigenMat(D_);

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

ns_control_toolbox::tf ns_control_toolbox::padecoeff(const double& Td, const size_t& order)
{
	if (std::fabs(Td) <= EPS)
	{
		return tf();             // constant tf.
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
		double fact = Td * (order_double_t - k_double_t + 1) / (2. * order_double_t - k_double_t + 1) / k_double_t;
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

	return tf(num, den);
}

ns_control_toolbox::tf ns_control_toolbox::pade(const double& Td, const size_t& order)
{
	return padecoeff(Td, order);

}
