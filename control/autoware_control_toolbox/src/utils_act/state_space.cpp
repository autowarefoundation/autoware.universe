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

#include <utility>
#include "utils_act/transfer_functions.hpp"


ns_control_toolbox::tf2ss::tf2ss(const ns_control_toolbox::tf& sys_tf, const double& Ts) : Ts_{ Ts }
{
	N_ = sys_tf.order(); // size of A will be order of the TF.
	updateStateSpace(sys_tf);

}

void ns_control_toolbox::tf2ss::updateStateSpace(const ns_control_toolbox::tf& sys_tf)
{

	auto&& num = sys_tf.num();
	auto&& den = sys_tf.den();

	// Check the leading zeros to determine the order of the system, and strip the leading zeros.
	ns_utils::stripVectorZerosFromLeft(num);
	ns_utils::stripVectorZerosFromLeft(den);

	// Compare if the system is a proper.
	if (den.size() < num.size())
	{
		throw std::invalid_argument("This system is not a proper system.");
	}

	if (den.size() == 1 && num.size() == 1)
	{
		throw std::invalid_argument("System is a static gain not a dynamic system");
	}

	// Compute the system matrices.
	computeSystemMatrices(num, den);

	// Discretisize.
	discretisize(Ts_);

}

ns_control_toolbox::tf2ss::tf2ss(const std::vector<double>& numerator,
                                 const std::vector<double>& denominator, const double& Ts) : Ts_{ Ts }
{
	auto num = numerator;
	auto den = denominator;

	// Check the leading zeros to determine the order of the system, and strip the leading zeros.
	ns_utils::stripVectorZerosFromLeft(num);
	ns_utils::stripVectorZerosFromLeft(den);

	N_ = static_cast<Eigen::Index>(std::max(num.size(), den.size()));

	// Compare if the system is a proper.
	if (den.size() < num.size())
	{
		throw std::invalid_argument("This system is not a proper system.");
	}

	if (den.size() == 1 && num.size() == 1)
	{
		throw std::invalid_argument("System is a static gain not a dynamic system");
	}

	// Compute the system matrices.
	computeSystemMatrices(num, den);

	// Discretisize
	discretisize(Ts);
}


void ns_control_toolbox::tf2ss::computeSystemMatrices(const std::vector<double>& num,
                                                      const std::vector<double>& den)
{
	auto&& nx = N_ - 1; //static_cast<long>(den.size() - 1);       // Order of the system.

	// We can put system check function if the nx = 0 -- i.e throw exception.
	// B_ = Eigen::MatrixXd::Identity(nx, 1); // We assign B here and this not only an initialization.

	if (sys_matABCD_cont_.rows() != nx + 1 && sys_matABCD_cont_.cols() != nx + 1)
	{
		sys_matABCD_cont_.resize(nx + 1, nx + 1);
		sys_matABCD_disc_.resize(nx + 1, nx + 1);
	}

	sys_matABCD_cont_.setZero();
	sys_matABCD_disc_.setZero();


	// Zero padding the numerator.
	auto&& num_of_zero = den.size() - num.size();

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


	//	A_ = ss_system.topLeftCorner(nx, nx);
	//	B_ = ss_system.topRightCorner(nx, 1);
	//	C_ = ss_system.bottomLeftCorner(1, nx);
	//	D_ = ss_system.bottomRightCorner(1, 1);

	if (nx > 0)
	{
		// D_(0, 0) = zero_padded_num[0];
		sys_matABCD_cont_(nx, nx) = 0;

	}

	//	auto B = Eigen::MatrixXd::Identity(nx - 1, nx);
	//	ns_eigen_utils::printEigenMat(B);

	if (nx > 1)
	{
		// A_.bottomRows(nx - 1) = Eigen::MatrixXd::Identity(nx - 1, nx);
		sys_matABCD_cont_.block(1, 0, nx - 1, nx) = Eigen::MatrixXd::Identity(nx - 1, nx);
	}

	// Assign B in system_matABCD
	sys_matABCD_cont_(0, nx) = 1;

	// normalize the denominator and assign the first row of the A_matrix to the normalized denominator's values
	// excluding the first item of the denominator.
	for (size_t k = 1; k < normalized_den.size(); k++)
	{
		Eigen::Index ind_eig{ static_cast<long>(k - 1) };

		// A_(0, ind_eig) = -1 * normalized_den[k];
		sys_matABCD_cont_.topLeftCorner(nx, nx)(0, ind_eig) = -1 * normalized_den[k];


		// C_(0, ind_eig) = zero_padded_num[k] - zero_padded_num[0] * normalized_den[k];
		sys_matABCD_cont_.bottomLeftCorner(1, nx)(0, ind_eig) =
				zero_padded_num[k] - zero_padded_num[0] * normalized_den[k];

	}

	// Balance the matrices.
	// Call balance method on the system matrices.
	ns_control_toolbox::balance(sys_matABCD_cont_);
}

void ns_control_toolbox::tf2ss::print() const
{
	ns_utils::print("System Matrices [A, B; C, D] : ");
	ns_eigen_utils::printEigenMat(sys_matABCD_cont_);

	ns_utils::print("A : ");
	ns_eigen_utils::printEigenMat(A());

	ns_utils::print("B : ");
	ns_eigen_utils::printEigenMat(B());

	ns_utils::print("C : ");
	ns_eigen_utils::printEigenMat(C());

	ns_utils::print("D : ");
	ns_eigen_utils::printEigenMat(D());
}


void ns_control_toolbox::tf2ss::print_discrete_system() const
{

	ns_utils::print("Ad : ");
	ns_eigen_utils::printEigenMat(Ad());

	ns_utils::print("Bd : ");
	ns_eigen_utils::printEigenMat(Bd());

	ns_utils::print("Cd : ");
	ns_eigen_utils::printEigenMat(Cd());

	ns_utils::print("Dd : ");
	ns_eigen_utils::printEigenMat(Dd());

	ns_utils::print("System Matrices [Ad, Bd; Cd, Dd] : ");
	ns_eigen_utils::printEigenMat(sys_matABCD_disc_);
}

/**
 * @brief Discretisize the continuous time state-space model using Tustin approximation.
 * */
void ns_control_toolbox::tf2ss::discretisize(double const& Ts)
{

	auto&& nx = N_ - 1;

	auto&& A_ = sys_matABCD_cont_.topLeftCorner(nx, nx);
	auto&& B_ = sys_matABCD_cont_.topRightCorner(nx, 1);
	auto&& C_ = sys_matABCD_cont_.bottomLeftCorner(1, nx);
	auto&& D_ = sys_matABCD_cont_.bottomRightCorner(1, 1);

	ns_utils::print("A : ");
	ns_eigen_utils::printEigenMat(A_);

	ns_utils::print("B : ");
	ns_eigen_utils::printEigenMat(B_);

	ns_utils::print("C : ");
	ns_eigen_utils::printEigenMat(C_);

	ns_utils::print("D : ");
	ns_eigen_utils::printEigenMat(D_);

	// take inverse:
	auto const&& I = Eigen::MatrixXd::Identity(nx, nx);

	auto const&& mat1_ATs = I - A_ * Ts / 2.;
	auto const&& inv1_ATs = mat1_ATs.inverse();


	auto&& Ad_ = inv1_ATs * (I + A_ * Ts / 2.);
	auto&& Bd_ = inv1_ATs * B_ * Ts;
	auto&& Cd_ = C_ * inv1_ATs;
	auto&& Dd_ = D_ + C_ * Bd_ / 2.;


	ns_utils::print("Ad : ");
	ns_eigen_utils::printEigenMat(Ad_);

	ns_utils::print("Bd : ");
	ns_eigen_utils::printEigenMat(Bd_);

	ns_utils::print("Cd : ");
	ns_eigen_utils::printEigenMat(Cd_);

	ns_utils::print("Dd : ");
	ns_eigen_utils::printEigenMat(Dd_);

	sys_matABCD_disc_.topLeftCorner(nx, nx) = Ad_;
	sys_matABCD_disc_.topRightCorner(nx, 1) = Bd_;
	sys_matABCD_disc_.bottomLeftCorner(1, nx) = Cd_;
	sys_matABCD_disc_.bottomRightCorner(1, 1) = Dd_;


	//	ns_utils::print("invA \nx");
	//	ns_eigen_utils::printEigenMat(inv1_ATs);

}

// Getters for the system matrices.
// Discrete time state-space matrices.
Eigen::MatrixXd ns_control_toolbox::tf2ss::Ad() const
{
	auto&& Ad = sys_matABCD_disc_.topLeftCorner(N_ - 1, N_ - 1);
	return Ad;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::Bd() const
{
	auto&& Bd = sys_matABCD_disc_.topRightCorner(N_ - 1, 1);
	return Bd;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::Cd() const
{
	auto&& Cd = sys_matABCD_disc_.bottomLeftCorner(1, N_ - 1);
	return Cd;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::Dd() const
{
	auto&& Dd = sys_matABCD_disc_.bottomRightCorner(1, 1);
	return Dd;
}


// Continuous time state-space matrices.
Eigen::MatrixXd ns_control_toolbox::tf2ss::A() const
{
	auto&& A = sys_matABCD_cont_.topLeftCorner(N_ - 1, N_ - 1);
	return A;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::B() const
{
	auto&& B = sys_matABCD_cont_.topRightCorner(N_ - 1, 1);
	return B;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::C() const
{
	auto&& C = sys_matABCD_cont_.bottomLeftCorner(1, N_ - 1);
	return C;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::D() const
{
	auto&& D = sys_matABCD_cont_.bottomRightCorner(1, 1);
	return D;
}


void ns_control_toolbox::tf2ss::getSystemMatricesABCD_disc(Eigen::MatrixXd& sysMat) const
{
	sysMat = sys_matABCD_disc_;
}

/**
 * @brief simulated the discrete system matrices [Ad, Bd:Cd, Dd] for one step. Its state matrix as an input
 * is a column matrix [x;u]. This state matrix returns as [x; y] which is in the form of xy = [A B;C D]xu.
 * */


void ns_control_toolbox::tf2ss::simulateOneStep(Eigen::MatrixXd& system_state_xu)
{
	system_state_xu.noalias() = sys_matABCD_disc_ * system_state_xu;
}








