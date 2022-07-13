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

#include "splines/bsplines_smoother.hpp"
#include <algorithm>
#include <vector>

ns_nmpc_splines::BSplineSmoother::BSplineSmoother(size_t base_signal_length,
																									double num_of_knots_ratio)
	: npoints_{base_signal_length}, knots_ratio_{num_of_knots_ratio}
{
	// Build Basis Matrix to be used in the penalized Least Squares.
	// We keep te same dimension for smoothed and base signals.
	auto tvec = ns_nmpc_utils::linspace<double>(0, 1, npoints_);

	// Create knot points.
	nknots_ = static_cast<Eigen::Index>(static_cast<double>(base_signal_length) * knots_ratio_);
	knots_vec_ = ns_nmpc_utils::linspace<double>(0, 1, nknots_);

	// Create the basis matrix from tvec and knots_vec with its first and second derivatives.
	Eigen::MatrixXd basis_mat(npoints_, nknots_);
	Eigen::MatrixXd basis_dmat(npoints_, nknots_);
	Eigen::MatrixXd basis_ddmat(npoints_, nknots_);

	basis_mat.setZero();
	basis_dmat.setZero();
	basis_ddmat.setZero();

	// Create the bases matrices for b-spline, its first and second derivatives.
	createBasesMatrix(tvec, basis_mat, basis_dmat, basis_ddmat);

	// Compute smoother projection matrices with pre-multiplication by bases.
	/*
	*   y = Basis * coeffs
	*   coeffs = projection_mat * ydata_original
	*
	*   y = Basis* projection_mat * (any data of original length) - (Basis* projection_mat can be reused).
	*
	*   Same applies for the derivatives.
	*
	* */

	// Call solver to obtain the projection matrix.
	solveByDemmlerReisch(basis_mat, basis_dmat, basis_ddmat);

	// We can also use solveByQR method to initialize the interpolators.
	//  solveByQR(basis_mat, basis_dmat, basis_ddmat);
}

void ns_nmpc_splines::BSplineSmoother::createBasesMatrix(const std::vector<double> &tvec,
																												 Eigen::MatrixXd &basis_mat,
																												 Eigen::MatrixXd &basis_dmat,
																												 Eigen::MatrixXd &basis_ddmat)
{
	for (auto k = 0; k < npoints_; k++)
	{
		auto ti = tvec[k];

		// first row is polynomial row, the rest are first and second derivatives.
		auto &&kk = basisRowsWithDerivatives(ti);

		basis_mat.row(k) = Eigen::Map<Eigen::MatrixXd>(kk[0].data(), 1, static_cast<int32_t>(kk[0].size()));
		basis_dmat.row(k) = Eigen::Map<Eigen::MatrixXd>(kk[1].data(), 1, static_cast<int32_t>(kk[1].size()));
		basis_ddmat.row(k) = Eigen::Map<Eigen::MatrixXd>(kk[2].data(), 1, static_cast<int32_t>(kk[2].size()));
	}
	//  ns_nmpc_eigen_utils::printEigenMat(basis_mat);
	//  ns_nmpc_eigen_utils::printEigenMat(basis_ddmat);
}

void ns_nmpc_splines::BSplineSmoother::createBasesMatrix(Eigen::MatrixXd const &tvec,
																												 Eigen::MatrixXd &basis_mat,
																												 Eigen::MatrixXd &basis_dmat,
																												 Eigen::MatrixXd &basis_ddmat)
{
	auto new_size = tvec.rows();
	for (auto k = 0; k < new_size; k++)
	{
		auto ti = tvec(k);

		// first row is polynomail row, the rest are first and second derivatives.
		auto &&kk = basisRowsWithDerivatives(ti);

		basis_mat.row(k) = Eigen::Map<Eigen::MatrixXd>(kk[0].data(), 1, static_cast<int32_t>(kk[0].size()));
		basis_dmat.row(k) = Eigen::Map<Eigen::MatrixXd>(kk[1].data(), 1, static_cast<int32_t>(kk[1].size()));
		basis_ddmat.row(k) = Eigen::Map<Eigen::MatrixXd>(kk[2].data(), 1, static_cast<int32_t>(kk[2].size()));
	}
}

std::vector<std::vector<double>> ns_nmpc_splines::BSplineSmoother::basisRowsWithDerivatives(const double &ti)
{
	std::vector<std::vector<double>> row_vectors{{1., ti}, {0., 1.}, {0., 0.}};

	for (size_t k = 0; k < nknots_ - 2; k++)
	{
		double ki = knots_vec_[k];
		std::vector<double> p_d1_d2 = fPlusCube(ti, ki);  // polynomial value with first and second derivatives.

		row_vectors[0].emplace_back(p_d1_d2[0]);
		row_vectors[1].emplace_back(p_d1_d2[1]);
		row_vectors[2].emplace_back(p_d1_d2[2]);
	}

	return row_vectors;
}

std::vector<double> ns_nmpc_splines::BSplineSmoother::fPlusCube(double const &ti, double const &ki) const
{
	auto it_final = std::prev(knots_vec_.cend());
	auto it_prev_final = std::prev(it_final);

	// b-spline polynomial
	double &&dk0 = (std::pow(std::max(0.0, ti - ki), 3) - std::pow(std::max(0.0, ti - *it_final), 3)) /
		(*it_final - ki);

	double &&dK0 = (std::pow(std::max(0.0, ti - *it_prev_final), 3) - std::pow(std::max(0.0, ti - *it_final), 3)) /
		(*it_final - *it_prev_final);

	// first derivative
	double &&dk1 = (3 * std::pow(std::max(0.0, ti - ki), 2) - 3 * std::pow(std::max(0.0, ti - *it_final), 2)) /
		(*it_final - ki);

	double
		&&dK1 = (3 * std::pow(std::max(0.0, ti - *it_prev_final), 2) - 3 * std::pow(std::max(0.0, ti - *it_final), 2)) /
		(*it_final - *it_prev_final);

	// second derivative
	double &&dk2 = (6 * std::max(0.0, ti - ki) - 6 * std::max(0.0, ti - *it_final)) / (*it_final - ki);
	double &&dK2 = (6 * std::max(0.0, ti - *it_prev_final) - 6 * std::max(0.0, ti - *it_final)) /
		(*it_final - *it_prev_final);

	return std::vector<double>{dk0 - dK0, dk1 - dK1, dk2 - dK2};
}

void ns_nmpc_splines::BSplineSmoother::solveByDemmlerReisch(Eigen::MatrixXd &basis_mat,
																														Eigen::MatrixXd &basis_dmat,
																														Eigen::MatrixXd &basis_ddmat)
{
	// Using the polynomial basis, obtain the projection matrix.
	auto Dc = basis_ddmat.transpose() * basis_ddmat;  // for minimum curvature smoothing penalization matrix.
	auto Bc = basis_mat.transpose() * basis_mat;

	// Compute Cholesky.
	Eigen::LLT<Eigen::MatrixXd> cholRRT(Bc + 1e-8 * Dc);
	Eigen::MatrixXd &&R = cholRRT.matrixL();
	Eigen::MatrixXd Rinv = R.inverse();

	//  ns_nmpc_eigen_utils::printEigenMat(Dc);
	//  ns_nmpc_eigen_utils::printEigenMat(Bc);
	//  ns_nmpc_eigen_utils::printEigenMat(Rinv);

	// Compute SVD
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(Rinv.transpose() * Dc * Rinv, Eigen::ComputeFullU | Eigen::ComputeFullV);

	Eigen::MatrixXd Sdiag(svd.singularValues().rows(), svd.singularValues().rows());
	Sdiag.setZero();

	Sdiag.diagonal() = svd.singularValues();
	auto A = basis_mat * Rinv * svd.matrixU();

	//  std::cout << "Diagonal Matrix \n";
	//  ns_nmpc_eigen_utils::printEigenMat(Dc);
	//  ns_nmpc_eigen_utils::printEigenMat(Sdiag);
	//  ns_nmpc_eigen_utils::printEigenMat(A);
	//  ns_nmpc_eigen_utils::printEigenMat(Rinv);
	//  ns_nmpc_eigen_utils::printEigenMat(svd.matrixU());
	//  ns_nmpc_eigen_utils::printEigenMat(svd.matrixV());

	// To be computed in the constructor.
	/**
	 *   y = Ax --> At*y = At*x -- > x = coeffs = inv(At*A)*A*ybase_data and A = A(t parameter)
	 *                          -- > ynew = Anew(t) * x  = Anew(t) * coeffs  = Anew(t) * inv(At*A)*A * ydata
	 *                          -- > projection matrix p = inv(At*A)*A this is projection_mat_base_.
	 *                          ---> projection with base included  Anew(t) * inv(At*A)*A    *
	 * */

	projection_mat_base_ =
		Rinv * svd.matrixU() * (A.transpose() * A + lambda_ * lambda_ * Sdiag).inverse() * A.transpose();

	projection_mat_wb_ = basis_mat * projection_mat_base_;  // ynew = A{A_T A + lambda diag(s)}−1 A^T y.
	projection_mat_dot_wb_ = basis_dmat * projection_mat_base_;
	projection_mat_ddot_wb_ = basis_ddmat * projection_mat_base_;
}

// Direct interpolation skips computation of coefficients.
void ns_nmpc_splines::BSplineSmoother::InterpolateInCoordinates(const Eigen::MatrixXd &ybase,
																																Eigen::MatrixXd &data_tobe_interpolated)
{
	// ynew = A {A_T A + lambda diag(s)}−1 A^T ybase =
	// projection_mat_wb_ * ybase where {A_T A + lambda diag(s)}−1 are coeffs.
	data_tobe_interpolated = projection_mat_wb_ * ybase;
}

void ns_nmpc_splines::BSplineSmoother::solveByQR(Eigen::MatrixXd &basis_mat,
																								 Eigen::MatrixXd &basis_dmat,
																								 Eigen::MatrixXd &basis_ddmat)
{

	Eigen::MatrixXd lambdaD = lambda_ * basis_ddmat;
	auto AD = ns_nmpc_eigen_utils::vstack<double>(basis_mat, lambdaD);

	// Take QR
	Eigen::HouseholderQR<Eigen::MatrixXd> qrAD(AD);

	Eigen::MatrixXd thinQ(AD.rows(), AD.cols());
	thinQ.setIdentity();

	Eigen::MatrixXd &&Q = qrAD.householderQ() * thinQ;
	Eigen::MatrixXd &&R = Q.transpose() * AD;

	// get Q1
	auto m = basis_mat.rows();
	auto &&Q1 = Q.topRows(m);

	auto &&Rinv = R.inverse();
	auto RinvQ1T = Rinv * Q1.transpose();

	projection_mat_wb_ = basis_mat * RinvQ1T;
	projection_mat_dot_wb_ = basis_dmat * RinvQ1T;
	projection_mat_ddot_wb_ = basis_ddmat * RinvQ1T;

	//  std::cout << "Q1 size " << Q1.rows() << " " << Q1.cols() << std::endl;
	//  std::cout << "R size " << R.rows() << " " << R.cols() << std::endl;
	//  std::cout << "Rinv size " << R.rows() << " " << R.cols() << std::endl;
}

void ns_nmpc_splines::BSplineSmoother::getFirstDerivative(const Eigen::MatrixXd &ybase,
																													Eigen::MatrixXd &ybase_dot) const
{
	ybase_dot = projection_mat_dot_wb_ * ybase;
}

void ns_nmpc_splines::BSplineSmoother::getSecondDerivative(const Eigen::MatrixXd &ybase,
																													 Eigen::MatrixXd &ybase_dot_dot) const
{
	ybase_dot_dot = projection_mat_ddot_wb_ * ybase;
}
