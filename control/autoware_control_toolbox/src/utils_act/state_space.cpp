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


ns_control_toolbox::tf2ss::tf2ss(const ns_control_toolbox::tf& sys_tf, const double& Ts) : Ts_{ Ts },
                                                                                           N_{ sys_tf.order() }
{
	//N_ = sys_tf.order(); // size of A will be order of the TF.

	auto const& nx = N_;
	A_.resize(nx, nx);
	B_.resize(nx, 1);
	C_.resize(1, nx);
	D_.resize(1, 1);

	Ad_.resize(nx, nx);
	Bd_.resize(nx, 1);
	Cd_.resize(1, nx);
	Dd_.resize(1, 1);

	Tsimilarity_mat_.resize(nx, nx);

	A_.setZero();
	B_.setZero();
	C_.setZero();
	D_.setZero();

	Ad_.setZero();
	Bd_.setZero();
	Cd_.setZero();
	Dd_.setZero();
	Tsimilarity_mat_.setIdentity();

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

	N_ = static_cast<Eigen::Index>(std::max(num.size(), den.size())) - 1;

	// Compare if the system is a proper.
	if (den.size() < num.size())
	{
		throw std::invalid_argument("This system is not a proper system.");
	}

	if (den.size() == 1 && num.size() == 1)
	{
		throw std::invalid_argument("System is a static gain not a dynamic system");
	}

	// Initialize the system matrices.
	auto const& nx = N_;
	A_.resize(nx, nx);
	B_.resize(nx, 1);
	C_.resize(1, nx);
	D_.resize(1, 1);

	Ad_.resize(nx, nx);
	Bd_.resize(nx, 1);
	Cd_.resize(1, nx);
	Dd_.resize(1, 1);

	Tsimilarity_mat_.resize(nx, nx);

	A_.setZero();
	B_.setZero();
	C_.setZero();
	D_.setZero();

	Ad_.setZero();
	Bd_.setZero();
	Cd_.setZero();
	Dd_.setZero();


	// Compute the system matrices.
	computeSystemMatrices(num, den);

	// Discretisize
	discretisize(Ts);
}


void ns_control_toolbox::tf2ss::computeSystemMatrices(const std::vector<double>& num,
                                                      const std::vector<double>& den)
{
	auto const nx = N_; //static_cast<long>(den.size() - 1);       // Order of the system.

	// We can put system check function if the nx = 0 -- i.e throw exception.
	// B_ = Eigen::MatrixXd::Identity(nx, 1); // We assign B here and this not only an initialization.

	if (A_.rows() != nx && A_.cols() != nx)
	{
		A_.resize(nx, nx);
		B_.resize(nx, 1);
		C_.resize(1, nx);
		D_.resize(1, 1);

		Ad_.resize(nx, nx);
		Bd_.resize(nx, 1);
		Cd_.resize(1, nx);
		Dd_.resize(1, 1);

		Tsimilarity_mat_.resize(nx, nx);

		A_.setZero();
		B_.setZero();
		C_.setZero();
		D_.setZero();

		Ad_.setZero();
		Bd_.setZero();
		Cd_.setZero();
		Dd_.setZero();

	}

	B_ = Eigen::MatrixXd::Identity(nx, 1);
	Tsimilarity_mat_.setIdentity();

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
	// Call balance_a_matrix method on the system matrices.
	//	ns_utils::print("Before Balancing");
	//	ns_eigen_utils::printEigenMat(A_);
	//	ns_eigen_utils::printEigenMat(B_);
	//	ns_eigen_utils::printEigenMat(C_);
	//	ns_eigen_utils::printEigenMat(D_);

	ns_control_toolbox::balance_a_matrix(A_, Tsimilarity_mat_);

	// Balance C_ and B_;
	double nB = B_.lpNorm<1>();
	double nC = C_.lpNorm<1>();

	// alpha is a conditioning number that multiplies the smaller normed vector, divides the larger one.
	double alpha = ns_control_toolbox::balance_symmetric(nB, nC);
	// ns_utils::print("Alpha :", alpha);

	if (nB < nC)
	{
		// Apply similarity transformation
		B_.noalias() = Tsimilarity_mat_.inverse() * B_ * alpha;
		C_.noalias() = C_ * Tsimilarity_mat_ / alpha;
	}
	else
	{
		B_.noalias() = Tsimilarity_mat_.inverse() * B_ / alpha;
		C_.noalias() = C_ * Tsimilarity_mat_ * alpha;
	}

//	nB = B_.lpNorm<1>();
//	ns_utils::print("After Balancing");
//	ns_eigen_utils::printEigenMat(A_);
//	ns_eigen_utils::printEigenMat(B_);
//	ns_eigen_utils::printEigenMat(C_);
//	ns_eigen_utils::printEigenMat(D_);

	// auto Tinv = Eigen::MatrixXd()

//	ns_utils::print("Tsimilarity ");
//	ns_eigen_utils::printEigenMat(Tsimilarity_mat_);
//
//	ns_utils::print("Tsimilarity Inverse ");
//	ns_eigen_utils::printEigenMat(Tsimilarity_mat_.inverse());
}

void ns_control_toolbox::tf2ss::print() const
{

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


}

/**
 * @brief Discretisize the continuous time state-space model using Tustin approximation.
 * */
void ns_control_toolbox::tf2ss::discretisize(double const& Ts)
{

	auto const& nx = N_;

	// take inverse:
	auto const&& I = Eigen::MatrixXd::Identity(nx, nx);

	auto const&& mat1_ATs = I - A_ * Ts / 2.;
	auto const&& inv1_ATs = mat1_ATs.inverse();


	Ad_ = inv1_ATs * (I + A_ * Ts / 2.);
	Bd_ = inv1_ATs * B_ * Ts;
	Cd_ = C_ * inv1_ATs;
	Dd_ = D_ + C_ * Bd_ / 2.;


//	ns_utils::print("Ad : ");
//	ns_eigen_utils::printEigenMat(Ad_);
//
//	ns_utils::print("Bd : ");
//	ns_eigen_utils::printEigenMat(Bd_);
//
//	ns_utils::print("Cd : ");
//	ns_eigen_utils::printEigenMat(Cd_);
//
//	ns_utils::print("Dd : ");
//	ns_eigen_utils::printEigenMat(Dd_);


}

// Getters for the system matrices.
// Discrete time state-space matrices.
Eigen::MatrixXd ns_control_toolbox::tf2ss::Ad() const
{

	return Ad_;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::Bd() const
{

	return Bd_;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::Cd() const
{
	return Cd_;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::Dd() const
{

	return Dd_;
}


// Continuous time state-space matrices.
Eigen::MatrixXd ns_control_toolbox::tf2ss::A() const
{

	return A_;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::B() const
{

	return B_;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::C() const
{

	return C_;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::D() const
{

	return D_;
}


/**
 * @brief simulated the discrete system matrices [Ad, Bd:Cd, Dd] for one step. Its state matrix as an input
 * is a column matrix [x;u]. This state matrix returns as [x; y] which is in the form of xy = [A B;C D]xu.
 * */



double ns_control_toolbox::tf2ss::simulateOneStep(Eigen::MatrixXd& x0, const double& u) const
{

	// First compute the output y.
	double y = (Cd_ * x0 + Dd_ * u)(0, 0);

	// Then update x0;
	x0.noalias() = Ad_ * x0 + Bd_ * u;

	return y;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::T() const
{
	return Tsimilarity_mat_;
}








