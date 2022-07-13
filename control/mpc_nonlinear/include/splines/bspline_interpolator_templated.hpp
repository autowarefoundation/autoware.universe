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
#ifndef SPLINES__BSPLINE_INTERPOLATOR_TEMPLATED_HPP_
#define SPLINES__BSPLINE_INTERPOLATOR_TEMPLATED_HPP_

#include <Eigen/Core>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <algorithm>
#include <functional>
#include <iostream>
#include <numeric>
#include <vector>
#include "utils/nmpc_utils.hpp"
#include "utils/nmpc_utils_eigen.hpp"

namespace ns_nmpc_splines
{
/**
 * @brief Interpolates any given coordinate-data pair by fitting a global spline using BSpline methods.
 * @tparam Nin is the length of the input coordinate and data.
 * @tparam Nout is the length of the output coordinate and data: depending on Nin-Nout this class
 * either down or up-sample the data defined by the smoothness factor \lambda.
 * */
template<int Nin, int Nout>
class BSplineInterpolatorTemplated
{
 public:
	using eigen_base_vector_t = Eigen::Matrix<double, Nin, 1>;  //
	using eigen_new_base_vector_t = Eigen::Matrix<double, Nout, 1>;

	// BSplineInterpolatorTemplated() = default;

	explicit BSplineInterpolatorTemplated(double num_of_knots_ratio = 0.5, bool compute_derivatives = true);

	// Constructors.
	BSplineInterpolatorTemplated(BSplineInterpolatorTemplated const &other);

	BSplineInterpolatorTemplated &operator=(BSplineInterpolatorTemplated const &other);

	~BSplineInterpolatorTemplated() = default;

	// BSplineInterpolatorTemplated(BSplineInterpolatorTemplated &&other) noexcept;
	// BSplineInterpolatorTemplated &operator=(BSplineInterpolatorTemplated &&other) noexcept;

	// Base data in, interpolated data out. The respective coordinates are created for each of the
	// data feed.
	void InterpolateImplicitCoordinates(
		Eigen::MatrixXd const &ybase, Eigen::MatrixXd &data_tobe_interpolated);

	// If the derivatives are requested, one can call these two following methods.
	// Base data is provided, first and second derivatives are returned as the second terms.
	void getFirstDerivative(
		Eigen::MatrixXd const &ybase, Eigen::MatrixXd &data_dot_tobe_interpolated);

	void getSecondDerivative(
		Eigen::MatrixXd const &ybase, Eigen::MatrixXd &data_dot_dot_tobe_interpolated);

 private:
	// Pre-settings. Increasing lambda yield more smooth and flattened curve.
	/**
	 * Smoothing factor used in normal form of LS; B*B + (lambda**2)*D*D, D is f''(x) in the
	 *  smoother version.
	 **/
	double lambda_{0.001};

	double knots_ratio_{};        // !<-@brief ratio of the number of knots to the coord points.
	bool compute_derivatives_{};  // !<-@brief flag to compute the derivatives.

	// Initialized during instantiation.
	size_t nknots_{};                // !<-@brief number of knot points in the BSpline formulation.
	size_t n_base_points_{};         // !<-@brief number of points in the base,
	size_t new_npoints_{};           // !<-@brief number of points in the new dimensions
	std::vector<double> knots_vec_;  // !<-@brief knot coordinates of the global smoother.

	// To be computed in the constructor.
	/** @brief
	 *   A = A(t) parametric polynomials in the rows, each row has
	 *   [1, t, t**2, t**3, (t-knot)**3_i ... ]
	 *   y = Ax --> A^T*y = A^T A*x -- > x = coeffs = inv(A^T*A)*A*ybase_data
	 *              -- > ynew = Anew_base(t) * x  = Anew(t) * coeffs  = Anew(t) * inv(A^T*A)*A * ydata
	 *              -- > projection matrix p = inv(A^T*A)*A
	 *              ---> projection with base included  Anew(t) * inv(At*A)*A
	 * */
	Eigen::MatrixXd projection_mat_base_;  // Base signal to be interpolated from.

	// inv(A^T*A)*A  when ydata comes it becomes coeffs = x = inv(A^T*A)*A * ybase_data
	Eigen::MatrixXd projection_mat_w_new_base_;

	// First derivative: ysmooth_dot = basis_dot * projection_mat_dot*ydata
	Eigen::MatrixXd projection_mat_w_new_base_dot_;

	// Second derivative: ysmooth_ddot = basis_ddot * projection_mat_ddot*ydata
	Eigen::MatrixXd projection_mat_w_new_base_dot_dot_;

	// Inner Methods.
	// If the derivatives are not requested: The projection matrix.
	/**
	 * @brief Creates a projection matrix from coordinates.
	 * @param [in] tvec a monotonic sequence coordinate vector.
	 * @param [out] basis_mat projection matrix.
	 */
	void createBasesMatrix(Eigen::VectorXd const &tvec, Eigen::MatrixXd &basis_mat) const;

	// If the derivatives of the new interpolated data are requested.
	/**
	 * @brief Creates a projection and a regularization matrix given the coordinates.
	 * @param [in] tvec a monotonic sequence coordinate vector.
	 * @param [out] basis_mat projection matrix.
	 * @param [out] regularization_mat_dd regularization matrix to be computed.
	 */
	void createBasesMatrix(
		Eigen::VectorXd const &tvec, Eigen::MatrixXd &basis_mat,
		Eigen::MatrixXd &regularization_mat_dd) const;

	// Used to compute new interpolated data and its derivatives.
	/**
	 * @brief Creates a projection and a regularization matrix given the coordinates.
	 * @param [in] tvec a monotonic sequence coordinate vector.
	 * @param [out] basis_mat projection matrix.
	 * @param [out] basis_dmat projection matrix for the first derivatives.
	 * @param [out] basis_ddmat projection matrix for the second derivatives.
	 **/
	void createBasesMatrix(
		Eigen::VectorXd const &tvec, Eigen::MatrixXd &basis_mat, Eigen::MatrixXd &basis_dmat,
		Eigen::MatrixXd &basis_ddmat) const;

	// Set projection matrix for the base data.
	/**
	 * @brief Computes the spline coefficients using Demmler-Reisch ortogonalization.
	 * @param [in] basis_mat projection matrix.
	 * @param [in] penalizing_mat_D regularization matrix.
	 * */
	void solveByDemmlerReisch(
		Eigen::MatrixXd const &basis_mat, Eigen::MatrixXd const &penalizing_mat_D);

	// An alternative solution to Demmler-Reisch method.
	/**
	 * @brief Computes the spline coefficients using QR factorization alternative to Demmler-Reisch
	 * orthogonalization.
	 * @param [in] basis_mat projection matrix.
	 * @param [in] penalizing_mat_D regularization matrix.
	 * */
	void solveByQR(Eigen::MatrixXd const &basis_mat, Eigen::MatrixXd const &penalizing_mat_D);

	// Columwise normalization (standardization)
	void normalizeColumns(Eigen::MatrixXd const &ybase,
												Eigen::MatrixXd &ybase_tobe_normalized,
												std::vector<double> &col_maxs);

	void normalizeColumns_Back(Eigen::MatrixXd const &normalized_estimate,
														 std::vector<double> const &col_maxs,
														 Eigen::MatrixXd &unnormalized_output_matrix
	);

};

template<int Nin, int Nout>
BSplineInterpolatorTemplated<Nin, Nout>::BSplineInterpolatorTemplated(
	double num_of_knots_ratio, bool compute_derivatives)
	: knots_ratio_(num_of_knots_ratio), compute_derivatives_{compute_derivatives}
{
	/**
	 *  @brief This interpolation class is designed to get re-sampled vectors or matrices. The entire signals are
	 *  parameterized using a "t" parameter scaled in [0, 1]. The curve starts at zero and ends at one.
	 * */

	nknots_ = static_cast<Eigen::Index>(Nin * num_of_knots_ratio);
	n_base_points_ = Nin;
	new_npoints_ = Nout;

	// Create knot points.
	knots_vec_ = ns_nmpc_utils::linspace<double>(0, 1, nknots_);

	// Initialize the Eigen matrix data members.
	projection_mat_base_ = Eigen::MatrixXd::Zero(nknots_ + 4, Nin);
	projection_mat_w_new_base_ = Eigen::MatrixXd::Zero(Nout, Nin);
	projection_mat_w_new_base_dot_ = Eigen::MatrixXd::Zero(Nout, Nin);
	projection_mat_w_new_base_dot_dot_ = Eigen::MatrixXd::Zero(Nout, Nin);

	// We need two different coordinates, one for base, one for the new dimensions.
	// base signal coordinates.
	eigen_base_vector_t tvec_base = eigen_base_vector_t::LinSpaced(Nin, 0.0, 1.);

	// new coordinates.
	eigen_new_base_vector_t tvec_new = eigen_new_base_vector_t::LinSpaced(Nout, 0.0, 1.0);

	// DEBUG
	//  auto size_base = tvec_base.size();
	//  auto size_new = tvec_new.size();
	// end of debug.

	// Create the basis matrix from tvec_base to compute basis matrices for the base data.
	// Number of polynomials items of  four + knots [1, t, t**2, t**3, (t-ki)**3, ....]
	// if the derivatives are not requested.
	if (!compute_derivatives_)
	{
		Eigen::MatrixXd basis_mat(Nin, nknots_ + 4);
		basis_mat.setZero();

		// for Tikhonov regulatization D which can be identity or second derivative matrix for smooth
		// curvature.
		Eigen::MatrixXd penalized_weights_D(Nin, nknots_ + 4);

		// If we don't use the first and second derivative, it can be an identity mat.
		penalized_weights_D.setIdentity();

		// Create interpolating base matrix for the base data.
		// This basis is used to compute the coefficients from the given ybase data.
		// y = BaseMat@ coeffs = B(t) [a, b, c,d]^T
		createBasesMatrix(tvec_base, basis_mat);

		// Create interpolating base matrix for the new coordinates.
		/**
		 *  We can define new basis mat using different t-parameters. And use the same coefficients computed from the
		 *  original data. ynew = Base_new(t is evaluated at different points) * coeffs_computed_from_ybase.
		 * */
		Eigen::MatrixXd new_basis_mat(Nout, nknots_ + 4);
		new_basis_mat.setZero();

		createBasesMatrix(tvec_new, new_basis_mat);

		// Solve the matrix equations. This computes and sets the projection_mat_base_
		solveByDemmlerReisch(basis_mat, penalized_weights_D);

		// Once the following matrix is computed, we multiply it with the given ybase which yields
		// ynew_in_new coordinates.
		// This new matrix; projection_mat_w_new_base_ can be used to interpolate ybase --> ynew with
		// a new length.
		// ynew_length = projection_mat_w_new_base_ @ ybase_length
		projection_mat_w_new_base_ =
			new_basis_mat * projection_mat_base_;  // ynew = A{A^T@A + lambda diag(s)}−1@A^T .

	} else
	{
		// Derivatives are requested.
		// We don't need to create an additional penalized_weights_D, we have a second derivative
		// matrix here to be used as a regularization matrix.
		// Base matrix to compute the coefficients from the data ybase.
		Eigen::MatrixXd basis_mat(n_base_points_, nknots_ + 4);

		// For optimal curvature, we need the second derivative of the basis matrix for the base data.
		Eigen::MatrixXd basis_mat_dd(n_base_points_, nknots_ + 4);
		basis_mat.setZero();
		basis_mat_dd.setIdentity();

		// Using the second derivative as a regularizer is problematic,
		// we need to solve singularity when taking the inverse.
		// createBasesMatrix(tvec_base, basis_mat, basis_mat_dd);
		// If we use the second derivative as a
		// regularizer.
		createBasesMatrix(tvec_base, basis_mat);

		// Compute the new interpolation base matrices.
		// We the coefficients computed for the ybase and new bases.
		// y, y', y" = B, B', B" @ coeff_base.
		Eigen::MatrixXd new_basis_mat(new_npoints_, nknots_ + 4);
		Eigen::MatrixXd new_basis_d_mat(new_npoints_, nknots_ + 4);
		Eigen::MatrixXd new_basis_dd_mat(new_npoints_, nknots_ + 4);

		new_basis_mat.setZero();
		new_basis_d_mat.setZero();
		new_basis_dd_mat.setZero();

		createBasesMatrix(tvec_new, new_basis_mat, new_basis_d_mat, new_basis_dd_mat);

		solveByDemmlerReisch(basis_mat, basis_mat_dd);

		// Set Interpolating projection matrices
		projection_mat_w_new_base_ = new_basis_mat * projection_mat_base_;
		projection_mat_w_new_base_dot_ = new_basis_d_mat * projection_mat_base_;
		projection_mat_w_new_base_dot_dot_ = new_basis_dd_mat * projection_mat_base_;
	}
}

template<int Nin, int Nout>
void BSplineInterpolatorTemplated<Nin, Nout>::createBasesMatrix(
	const Eigen::VectorXd &tvec, Eigen::MatrixXd &basis_mat,
	Eigen::MatrixXd &regularization_mat_dd) const
{
	/**
	 *     bspline_primitive = lambda t, knots: [1., t, t ** 2, t ** 3] + \
	 *                                    [((t - kn) ** 3 if (t - kn) > 0 else 0.) for kn in knots]
	 */

	// Base polynomial
	basis_mat.col(0).setConstant(1.);  // [1, ...]
	basis_mat.col(1) = tvec;           // [1, t, ...]
	basis_mat.col(2) =
		Eigen::VectorXd(tvec.unaryExpr([](auto const &x)
																	 { return x * x; }));  // [1, t, t**2]

	basis_mat.col(3) = Eigen::VectorXd(
		tvec.unaryExpr([](auto const &x)
									 { return x * x * x; }));  // [1, t, t**2, t**3]

	// Second derivative
	/**
	 *   bspline_primitive_dot_dot = lambda t, knots: [0., 0, 2, 6*t] +
	 *                                        [(6*(t - kn)  if (t - kn) > 0 else 0.) for kn in knots]
	 *
	 * */
	regularization_mat_dd.leftCols(2).setZero();    // [0, 0]
	regularization_mat_dd.col(2).setConstant(2.0);  // [0, 0, 2]
	regularization_mat_dd.col(3) = 6.0 * tvec;      // [0, 0, 2, 6*t]

	for (auto k = 4; k < basis_mat.cols(); ++k)
	{
		auto ki = knots_vec_[k - 4];
		basis_mat.col(k) = Eigen::VectorXd(tvec.unaryExpr([&ki](auto const &x)
																											{
																												auto val_pos = std::max(0.0, (x - ki));
																												return std::pow(val_pos, 3);
																											}));

		regularization_mat_dd.col(k) = Eigen::VectorXd(tvec.unaryExpr([&ki](auto const &x)
																																	{
																																		auto val_pos = std::max(0.0, (x - ki));
																																		return 6.0 * val_pos;
																																	}));
	}
}

template<int Nin, int Nout>
void BSplineInterpolatorTemplated<Nin, Nout>::createBasesMatrix(
	const Eigen::VectorXd &tvec, Eigen::MatrixXd &basis_mat) const
{
	/**
	 *     bspline_primitive = lambda t, knots: [1., t, t ** 2, t ** 3] + \
	 *                                    [((t - kn) ** 3 if (t - kn) > 0 else 0.) for kn in knots]
	 */

	// DEBUG
	//  auto nbasis_row = basis_mat.rows();
	//  auto ntvec = tvec.size();
	// end of debug.

	basis_mat.col(0).setConstant(1.);
	basis_mat.col(1) = tvec;
	basis_mat.col(2) = Eigen::VectorXd(tvec.unaryExpr([](auto const &x)
																										{ return x * x; }));
	basis_mat.col(3) = Eigen::VectorXd(tvec.unaryExpr([](auto const &x)
																										{ return x * x * x; }));

	for (auto k = 4; k < basis_mat.cols(); ++k)
	{
		auto ki = knots_vec_[k - 4];
		basis_mat.col(k) = Eigen::VectorXd(tvec.unaryExpr([&ki](auto const &x)
																											{
																												auto val_pos = std::max(0.0, (x - ki));
																												return std::pow(val_pos, 3);
																											}));
	}
}

template<int Nin, int Nout>
void BSplineInterpolatorTemplated<Nin, Nout>::createBasesMatrix(
	const Eigen::VectorXd &tvec, Eigen::MatrixXd &basis_mat, Eigen::MatrixXd &basis_dmat,
	Eigen::MatrixXd &basis_ddmat) const
{
	/**
	 *     bspline_primitive = lambda t, knots: [1., t, t ** 2, t ** 3] + \
	 *                                    [((t - kn) ** 3 if (t - kn) > 0 else 0.) for kn in knots]
	 **/

	// Base polynomial
	basis_mat.col(0).setConstant(1.);
	basis_mat.col(1) = tvec;
	basis_mat.col(2) = Eigen::VectorXd(tvec.unaryExpr([](auto const &x)
																										{ return x * x; }));

	basis_mat.col(3) = Eigen::VectorXd(tvec.unaryExpr([](auto const &x)
																										{ return x * x * x; }));

	// First derivative of the basis matrix.
	/**
	 *     bspline_primitive_dot = lambda t, knots: [0., 1, 2*t, 3*t ** 2] + \
	 *                                    [(3*(t - kn) ** 2 if (t - kn) > 0 else 0.) for kn in knots]
	 **/

	basis_dmat.col(0).setConstant(0.);
	basis_dmat.col(1).setConstant(1.);
	basis_dmat.col(2) = 2 * tvec;
	basis_dmat.col(3) = Eigen::VectorXd(tvec.unaryExpr([](auto const &x)
																										 { return 3 * x * x; }));

	// Second derivative
	/**
	 *   bspline_primitive_dot_dot = lambda t, knots: [0., 0, 2, 6*t] +
	 *                                       [(6*(t - kn)  if (t - kn) > 0 else 0.) for kn in knots]
	 *
	 **/

	basis_ddmat.leftCols(2).setZero();    // [0, 0]
	basis_ddmat.col(2).setConstant(2.0);  // [0, 0, 2]
	basis_ddmat.col(3) = 6.0 * tvec;      // [0, 0, 2, t]

	for (auto k = 4; k < basis_mat.cols(); ++k)
	{
		auto const &ki = knots_vec_[k - 4];

		// (t-k)**3
		basis_mat.col(k) = Eigen::VectorXd(tvec.unaryExpr([&ki](auto const &x)
																											{
																												auto val_pos = std::max(0.0, (x - ki));

																												return std::pow(val_pos, 3);
																											}));

		// 3*(t-k)**2
		basis_dmat.col(k) = Eigen::VectorXd(tvec.unaryExpr([&ki](auto const &x)
																											 {
																												 auto val_pos = std::max(0.0, (x - ki));

																												 return 3.0 * val_pos * val_pos;
																											 }));

		//  6*(t-k)
		basis_ddmat.col(k) = Eigen::VectorXd(tvec.unaryExpr([&ki](auto const &x)
																												{
																													auto val_pos = std::max(0.0, (x - ki));

																													return 6.0 * val_pos;
																												}));
	}
}

template<int Nin, int Nout>
void BSplineInterpolatorTemplated<Nin, Nout>::solveByDemmlerReisch(
	const Eigen::MatrixXd &basis_mat, const Eigen::MatrixXd &penalizing_mat_D)
{
	// To be computed in the constructor.
	/**
	 *   y = Ax --> At*y = At*x -- > x = coeffs = inv(At*A)*A*ybase_data and A = A(t parameter)
	 *                    -- > ynew = Anew(t) * x  = Anew(t) * coeffs  = Anew(t) * inv(At*A)*A * ydata
	 *                    -- > projection matrix p = inv(At*A)*A this is projection_mat_base_.
	 *                    ---> projection with base included  Anew(t) * inv(At*A)*A    *
	 **/

	// projection_mat_base_ = Rinv * svd.matrixU() * (A.transpose() * A +
	// lambda_ * lambda_ * Sdiag).inverse() * A.transpose();
	// y_interp = A(A^T@A + )

	// for minimum curvature smoothing penalization matrix.
	auto const &&Dc = penalizing_mat_D.transpose() * penalizing_mat_D;
	auto const &&Bc = basis_mat.transpose() * basis_mat;  // B^T @ B

	// Compute Cholesky.
	// Factorize (B&T@B + eps*D) = eps for numerical stability.

	/**
	 *     @brief all three steps are put in a single step.
	 *     Eigen::LLT<Eigen::MatrixXd> cholRRT(Bc + 1e-8 * Dc);
	 *     Eigen::MatrixXd const &&R = cholRRT.matrixL();
	 *     Take the square root of (B&T@B + eps*D) = R^T * R
	 *     Eigen::MatrixXd &&Rinv = R.inverse();
	 **/
	Eigen::MatrixXd const &&Rinv = (Bc + 1e-8 * Dc).llt().matrixL().toDenseMatrix().inverse();

	// Compute SVD
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(
		Rinv.transpose() * Dc * Rinv, Eigen::ComputeFullU | Eigen::ComputeFullV);

	Eigen::MatrixXd Sdiag(svd.singularValues().rows(), svd.singularValues().rows());
	Sdiag.setZero();

	Sdiag.diagonal() = svd.singularValues();
	auto &&A = basis_mat * Rinv * svd.matrixU();  // A = Basis @ Rinverse @ U

	// To be computed in the constructor.
	/**
	 *   y = Ax --> At*y = At*x -- > x = coeffs = inv(At*A)*A*ybase_data and A = A(t parameter)
	 *                    - > ynew = Anew(t) * x  = Anew(t) * coeffs  = Anew(t) * inv(At*A)*A * ydata
	 *                    -- > projection matrix p = inv(At*A)*A this is projection_mat_base_.
	 *                    ---> projection with base included  Anew(t) * inv(At*A)*A
	 * */

	// y_interp = A(A^T@A + )
	projection_mat_base_ =
		Rinv * svd.matrixU() * (A.transpose() * A + lambda_ * lambda_ * Sdiag).inverse() * A.transpose();
}

template<int Nin, int Nout>
void BSplineInterpolatorTemplated<Nin, Nout>::solveByQR(
	const Eigen::MatrixXd &basis_mat, const Eigen::MatrixXd &penalizing_mat_D)
{

	Eigen::MatrixXd lambdaD(lambda_ * penalizing_mat_D);
	auto const &&AD = ns_nmpc_eigen_utils::vstack<double>(basis_mat, lambdaD);

	// Take QR
	Eigen::HouseholderQR<Eigen::MatrixXd> qrAD(AD);

	Eigen::MatrixXd thinQ(AD.rows(), AD.cols());
	thinQ.setIdentity();

	Eigen::MatrixXd &&Q = qrAD.householderQ() * thinQ;
	Eigen::MatrixXd &&R = Q.transpose() * AD;

	// get Q1
	auto const &&m = basis_mat.rows();
	auto const &&Q1 = Q.topRows(m);

	auto const &&Rinv = R.inverse();
	auto const RinvQ1T = Rinv * Q1.transpose();

	//  ns_nmpc_eigen_utils::printEigenMat(RinvQ1T);
	projection_mat_base_ = basis_mat * RinvQ1T;
}

template<int Nin, int Nout>
void BSplineInterpolatorTemplated<Nin, Nout>::InterpolateImplicitCoordinates(const Eigen::MatrixXd &ybase,
																																						 Eigen::MatrixXd &data_tobe_interpolated)
{

	// Standardize or normalize the data and restore back.
	auto const &&numcols = ybase.cols();
	if (auto const &&numrows = ybase.rows();numrows <= 1)
	{
		std::cerr << "\nNumber of rows must be more than 1 " << std::endl;
		return;
	}

	std::vector<double> colmaxvec(static_cast<size_t>(numcols));
	Eigen::MatrixXd ybase_normalized(ybase.rows(), ybase.cols());

	// Normalize
	normalizeColumns(ybase, ybase_normalized, colmaxvec);

	auto const &&normalized_interpolated_data = projection_mat_w_new_base_ * ybase_normalized;

	data_tobe_interpolated.resize(normalized_interpolated_data.rows(), normalized_interpolated_data.cols());

	// Unnormalize
	normalizeColumns_Back(normalized_interpolated_data, colmaxvec, data_tobe_interpolated);

}

template<int Nin, int Nout>
void BSplineInterpolatorTemplated<Nin, Nout>::getFirstDerivative(const Eigen::MatrixXd &ybase,
																																 Eigen::MatrixXd &data_dot_tobe_interpolated)
{
	if (!compute_derivatives_)
	{
		std::cout << " The interpolator was not prepared by the compute_derivative_option = True. "
								 "Cannot return the derivatives... ";
		return;
	}

	// Standardize or normalize the data and restore back.
	auto const &&numcols = ybase.cols();
	if (auto const &&numrows = ybase.rows();numrows <= 1)
	{
		std::cout << "\nNumber of rows must be more than 1 " << std::endl;
		return;
	}

	std::vector<double> colmaxvec(static_cast<size_t>(numcols));
	Eigen::MatrixXd ybase_normalized(ybase.rows(), ybase.cols());

	// Normalize
	normalizeColumns(ybase, ybase_normalized, colmaxvec);

	auto const &&normalized_interpolated_data = projection_mat_w_new_base_dot_ * ybase_normalized;

	data_dot_tobe_interpolated.resize(normalized_interpolated_data.rows(), normalized_interpolated_data.cols());

	// Unnormalize
	normalizeColumns_Back(normalized_interpolated_data, colmaxvec, data_dot_tobe_interpolated);

	// If we want to skip normalization procedures above, just use
	// the line below and comment out all the previous lines.
	//  data_dot_tobe_interpolated = projection_mat_w_new_base_ * ybase;
}

template<int Nin, int Nout>
void BSplineInterpolatorTemplated<Nin, Nout>::getSecondDerivative(const Eigen::MatrixXd &ybase,
																																	Eigen::MatrixXd &data_dot_dot_tobe_interpolated)
{
	if (!compute_derivatives_)
	{
		std::cout << " The interpolator was not prepared by the compute_derivative_option = True. "
								 "Cannot return the "
								 "derivatives... ";
		return;
	}

	// Standardize or normalize the data and restore back.
	auto const &&numcols = ybase.cols();

	if (auto const &&numrows = ybase.rows();numrows <= 1)
	{
		std::cout << "\nNumber of rows must be more than 1 " << std::endl;
		return;
	}

	std::vector<double> colmaxvec(static_cast<size_t>(numcols));
	Eigen::MatrixXd ybase_normalized(ybase.rows(), ybase.cols());

	// Normalize
	normalizeColumns(ybase, ybase_normalized, colmaxvec);

	auto &&normalized_interpolated_data = projection_mat_w_new_base_dot_dot_ * ybase_normalized;
	data_dot_dot_tobe_interpolated.resize(normalized_interpolated_data.rows(), normalized_interpolated_data.cols());

	// Unnormalize
	normalizeColumns_Back(normalized_interpolated_data, colmaxvec, data_dot_dot_tobe_interpolated);

	// If we want to skip normalization procedures above, just use the line below and
	// comment out all the previous lines.
	// data_dot_dot_tobe_interpolated = projection_mat_w_new_base_dot_dot_ * ybase;
}

template<int Nin, int Nout>
BSplineInterpolatorTemplated<Nin, Nout>::BSplineInterpolatorTemplated(BSplineInterpolatorTemplated<Nin,
																																																	 Nout> const &other)
{
	this->n_base_points_ = Nin;
	this->new_npoints_ = Nout;
	this->knots_ratio_ = other.knots_ratio_;
	this->compute_derivatives_ = other.compute_derivatives_;

	this->nknots_ = other.nknots_;
	this->knots_vec_ = other.knots_vec_;

	this->projection_mat_base_ = other.projection_mat_base_;

	this->projection_mat_w_new_base_ = other.projection_mat_w_new_base_;
	this->projection_mat_w_new_base_dot_ = other.projection_mat_w_new_base_dot_;
	this->projection_mat_w_new_base_dot_dot_ = other.projection_mat_w_new_base_dot_dot_;
}

template<int Nin, int Nout>
BSplineInterpolatorTemplated<Nin, Nout> &BSplineInterpolatorTemplated<Nin,
																																			Nout>::operator=(const BSplineInterpolatorTemplated &other)
{
	if (&other != this)
	{
		this->n_base_points_ = Nin;
		this->new_npoints_ = Nout;
		this->knots_ratio_ = other.knots_ratio_;
		this->compute_derivatives_ = other.compute_derivatives_;

		this->nknots_ = other.nknots_;
		this->knots_vec_ = other.knots_vec_;

		this->projection_mat_base_ = other.projection_mat_base_;

		this->projection_mat_w_new_base_ = other.projection_mat_w_new_base_;
		this->projection_mat_w_new_base_dot_ = other.projection_mat_w_new_base_dot_;
		this->projection_mat_w_new_base_dot_dot_ = other.projection_mat_w_new_base_dot_dot_;
	}

	return *this;
}

template<int Nin, int Nout>
void BSplineInterpolatorTemplated<Nin, Nout>::normalizeColumns(const Eigen::MatrixXd &ybase,
																															 Eigen::MatrixXd &ybase_tobe_normalized,
																															 std::vector<double> &col_maxs)
{
	auto const &&numcols = ybase.cols();
	auto &&EPS = std::numeric_limits<double>::epsilon();

	for (auto k = 0; k < numcols; ++k)
	{
		auto const &colmax = ybase.col(k).cwiseAbs().maxCoeff();
		col_maxs[k] = colmax + EPS;
		ybase_tobe_normalized.col(k) = ybase.col(k).unaryViewExpr([colmax](auto const &x)
																															{ return x / colmax; });
	}
}

template<int Nin, int Nout>
void BSplineInterpolatorTemplated<Nin, Nout>::normalizeColumns_Back(const Eigen::MatrixXd &normalized_estimate,
																																		const std::vector<double> &col_maxs,
																																		Eigen::MatrixXd &unnormalized_output_matrix)
{

	auto const &&numcols = normalized_estimate.cols();

	//  ns_nmpc_eigen_utils::printEigenMat(normalized_interpolated_data);
	for (auto k = 0; k < numcols; ++k)
	{
		auto const &colmax = col_maxs[k];
		unnormalized_output_matrix.col(k) = normalized_estimate.col(k).unaryExpr([colmax](auto const &x)
																																						 { return x * colmax; });
	}
}
}  // namespace ns_nmpc_splines
#endif  // SPLINES__BSPLINE_INTERPOLATOR_TEMPLATED_HPP_
