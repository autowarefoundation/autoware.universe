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

#include "utils_act/state_space.hpp"
#include "utils_act/transfer_functions.hpp"


/**
* @brief Stores the state space model matrices either discrete, or continuous.
* */
ns_control_toolbox::ss_system::ss_system(const Eigen::MatrixXd& Am,
                                         const Eigen::MatrixXd& Bm,
                                         const Eigen::MatrixXd& Cm,
                                         const Eigen::MatrixXd& Dm) : A(Am), B(Bm), C(Cm), D(Dm)
{

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


ns_control_toolbox::tf2ss::tf2ss(const ns_control_toolbox::tf& sys_tf, const double& Ts) : Ts_{ Ts }
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

	// Discretisize.
	discretisize(Ts);

}

ns_control_toolbox::tf2ss::tf2ss(const std::vector<double>& numerator,
                                 const std::vector<double>& denominator, const double& Ts) : Ts_{ Ts }
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

	// Discretisize
	discretisize(Ts);
}


void ns_control_toolbox::tf2ss::computeSystemMatrices(const std::vector<double>& num,
                                                      const std::vector<double>& den)
{
	auto nx = static_cast<long>(den.size() - 1);       // Order of the system.

	// We can put system check function if the nx = 0 -- i.e throw exception.

	A_ = Eigen::MatrixXd::Zero(nx, nx);
	C_ = Eigen::MatrixXd::Zero(1, nx);
	B_ = Eigen::MatrixXd::Identity(nx, 1);
	D_ = Eigen::MatrixXd::Zero(1, 1);

	// set discrete matrix sizes
	Ad_.resize(nx, nx);
	Bd_.resize(1, nx);
	Cd_.resize(nx, 1);
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


	if (nx > 0)
	{
		D_(0, 0) = zero_padded_num[0];
	}

	//	auto B = Eigen::MatrixXd::Identity(nx - 1, nx);
	//	ns_eigen_utils::printEigenMat(B);

	if (nx > 1)
	{
		A_.bottomRows(nx - 1) = Eigen::MatrixXd::Identity(nx - 1, nx);
	}


	// normalize the denominator and assign the first row of the A_matrix to the normalized denominator's values
	// excluding the first item of the denominator.
	for (size_t k = 1; k < normalized_den.size(); k++)
	{
		Eigen::Index ind_eig{ static_cast<long>(k - 1) };
		A_(0, ind_eig) = -1 * normalized_den[k];
		C_(0, ind_eig) = zero_padded_num[k] - zero_padded_num[0] * normalized_den[k];
	}

	// Balance the matrices.
	// First concatenate
	auto AB = ns_eigen_utils::hstack<double>(A_, B_);
	auto CD = ns_eigen_utils::hstack<double>(C_, D_);
	auto ss_system = ns_eigen_utils::vstack<double>(AB, CD);

	// Call balance metho on the system matrices.
	ns_control_toolbox::balance(ss_system);

	// Re-assign back to A, B, C, D. We might make this step faster with arrays.
	A_ = ss_system.topLeftCorner(nx, nx);
	B_ = ss_system.topRightCorner(nx, 1);
	C_ = ss_system.bottomLeftCorner(1, nx);
	D_ = ss_system.bottomRightCorner(1, 1);

//		bool debug = false;
//		if (debug)
//			{
////				ns_utils::print("A : \nx");
////				ns_eigen_utils::printEigenMat(A_);
////
////        ns_utils::print("B : \nx");
////        ns_eigen_utils::printEigenMat(B_);
////
////        ns_utils::print("C : \nx");
////        ns_eigen_utils::printEigenMat(C_);
////
////				ns_utils::print("D : \nx");
////				ns_eigen_utils::printEigenMat(D_);
//				ns_utils::print("Zero padded vector");
//				ns_utils::print_container(zero_padded_num);
//
//			}

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

ns_control_toolbox::ss_system ns_control_toolbox::tf2ss::get_ssABCD_continuous() const
{

	return ns_control_toolbox::ss_system(A_, B_, C_, D_);
}

ns_control_toolbox::ss_system ns_control_toolbox::tf2ss::get_ssABCD_discrete() const
{
	return ns_control_toolbox::ss_system(Ad_, Bd_, Cd_, Dd_);
}


