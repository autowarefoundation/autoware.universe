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


#include <functional>
#include <splines/bsplines_interpolator.hpp>
#include <vector>

ns_nmpc_splines::BSplineInterpolator::BSplineInterpolator(size_t base_signal_length,
																													size_t new_length,
																													double num_of_knots_ratio,
																													bool compute_derivatives)
	: n_base_points_(base_signal_length),
		new_npoints_(new_length),
		knots_ratio_(num_of_knots_ratio),
		compute_derivatives_{compute_derivatives},
		nknots_{static_cast<Eigen::size_t>(static_cast<double>(base_signal_length) * num_of_knots_ratio)}
{
	/**
	 *  This interpolation class is designed to get re-sampled vectors or matrices. The entire signals are
	 *  parameterized using a "t" parameter scaled in [0, 1]. The curve starts at zero and ends at one.
	 * */
	// We need two different coordinates, one for base, one for the new dimensions.
	// base signal coordinates
	auto tvec_base = Eigen::VectorXd::LinSpaced(static_cast<int32_t>(base_signal_length), 0.0, 1.);
	auto tvec_new = Eigen::VectorXd::LinSpaced(static_cast<int32_t>(new_length), 0.0, 1.0);

	// Create knot points.
	// nknots_ = static_cast<Eigen::size_t>(static_cast<double>(base_signal_length) * num_of_knots_ratio);
	knots_vec_ = ns_nmpc_utils::linspace<double>(0, 1, nknots_);

	// Create the basis matrix from tvec_base to compute basis matrices for the base data.
	// Number of polynomials items of  four + knots [1, t, t**2, t**3, (t-ki)**3, ....]

	if (!compute_derivatives_)
	{
		// if the derivatives are not requested.
		Eigen::MatrixXd basis_mat(n_base_points_, nknots_ + 4);

		// for Tikhonov regularization D, can be identity or second derivative matrix.
		Eigen::MatrixXd penalized_weights_D(n_base_points_, nknots_ + 4);
		basis_mat.setZero();

		// If we don't use the first and second derivative, it can be an identity mat.
		penalized_weights_D.setIdentity();

		// Create interpolating base matrix for the base data.
		// This basis is used to compute the coefficients from the given ybase data.
		// y = BaseMat@ coeffs = B(t) [a, b, c,d]^T
		createBasesMatrix(tvec_base, basis_mat);

		// Create interpolating base matrix for the new coordinates.
		/**
		 *  We can define new basis mat using different t-parameters. And use the same coefficients
		 *  computed from the original data.
		 *  ynew = Base_new(t is evaluated at different points) * coeffs_computed_from_ybase.
		 * */
		Eigen::MatrixXd new_basis_mat(new_npoints_, nknots_ + 4);
		new_basis_mat.setZero();

		createBasesMatrix(tvec_new, new_basis_mat);

		// Solve the matrix equations. This computes and sets the projection_mat_base_
		solveByDemmlerReisch(basis_mat, penalized_weights_D);

		// Once the following matrix is computed, we multiply it with the given ybase
		// which yields ynew_in_new coordinates.
		// This new matrix; projection_mat_w_new_base_ can be used to interpolate ybase --> ynew
		// with a new length.
		// ynew_length = projection_mat_w_new_base_ @ ybase_length
		projection_mat_w_new_base_ =
			new_basis_mat * projection_mat_base_;  // ynew = A{A^T@A + lambda diag(s)}−1@A^T .

	} else
	{
		// Derivatives are requested.
		// Base matrix to compute the coefficients from the data ybase.
		Eigen::MatrixXd basis_mat(n_base_points_, nknots_ + 4);

		// For optimal curvature, we need the second derivative of the basis matrix for the base data.
		Eigen::MatrixXd basis_mat_dd(n_base_points_, nknots_ + 4);
		basis_mat.setZero();
		basis_mat_dd.setIdentity();

		// Using the second derivative as a regularizer is problematic,
		// we need to solve singularity when taking the  inverse.
		// createBasesMatrix(tvec_base, basis_mat, basis_mat_dd);
		// If we use the second derivative as a regularizer.
		createBasesMatrix(tvec_base, basis_mat);

		// Compute the new interpolation base matrices. We the coefficients computed for
		// the ybase and new bases.
		// y, y', y" = B, B', B" @ coeff_base.

		Eigen::MatrixXd new_basis_mat(new_npoints_, nknots_ + 4);
		Eigen::MatrixXd new_basis_d_mat(new_npoints_, nknots_ + 4);
		Eigen::MatrixXd new_basis_dd_mat(new_npoints_, nknots_ + 4);

		new_basis_mat.setZero();
		new_basis_d_mat.setZero();
		new_basis_dd_mat.setZero();

		createBasesMatrix(tvec_new, new_basis_mat, new_basis_d_mat, new_basis_dd_mat);

		// Once the following matrix is computed, we multiply it with the given ybase
		// which yields ynew_in_new coordinates.
		// This new matrix; projection_mat_w_new_base_ can be used to interpolate
		// ybase --> ynew with a new length.
		// ynew_length = projection_mat_w_new_base_ @ ybase_length
		solveByDemmlerReisch(basis_mat, basis_mat_dd);

		// Set Interpolating projection matrices
		projection_mat_w_new_base_ = new_basis_mat * projection_mat_base_;
		projection_mat_w_new_base_dot_ = new_basis_d_mat * projection_mat_base_;
		projection_mat_w_new_base_dot_dot_ = new_basis_dd_mat * projection_mat_base_;
	}
}

ns_nmpc_splines::BSplineInterpolator::BSplineInterpolator(
	Eigen::MatrixXd const &tvec_base, Eigen::MatrixXd const &tvec_new, double num_of_knots_ratio,
	bool compute_derivatives) : knots_ratio_(num_of_knots_ratio), compute_derivatives_{compute_derivatives}
{
	//  std::cout << "Normed tvec : \n";
	//  ns_nmpc_eigen_utils::printEigenMat(tvec_base);
	//  std::cout << "Normed new tvec : \n";
	//  ns_nmpc_eigen_utils::printEigenMat(tvec_new);

	std::vector<double> tvec_std(tvec_new.data(), tvec_new.data() + tvec_new.size());
	auto isIncreasing_it = std::adjacent_find(tvec_std.begin(), tvec_std.end(), std::greater_equal<>());
	auto isDecreasing_it = std::adjacent_find(tvec_std.begin(), tvec_std.end(), std::less_equal<>());

	if (isDecreasing_it == tvec_std.cend() && isIncreasing_it == tvec_std.cend())
	{
		std::cout << "\nWARNING: Data coordinates are not monotonic series ... \n";
	}

	n_base_points_ = static_cast<size_t>(tvec_base.size());
	new_npoints_ = static_cast<size_t>(tvec_new.size());

	// We need two different coordinates, one for base, one for the new dimensions.
	// And these new coordinates are to be in the same interval [0, 1].
	auto max_tvec_base = tvec_base.maxCoeff();

	Eigen::MatrixXd tvec_base_normed(tvec_base.unaryExpr([&max_tvec_base](auto x)
																											 { return x / max_tvec_base; }));

	Eigen::MatrixXd tvec_new_normed(tvec_new.unaryExpr([&max_tvec_base](auto x)
																										 { return x / max_tvec_base; }));

	//  std::cout << "Normed tvec : \n";
	//  ns_nmpc_eigen_utils::printEigenMat(tvec_base_normed);
	//
	//  std::cout << "Normed new tvec : \n";
	//  ns_nmpc_eigen_utils::printEigenMat(tvec_new_normed);

	// Create knot points.
	nknots_ = static_cast<size_t>(static_cast<double>(tvec_base_normed.size()) * num_of_knots_ratio);
	knots_vec_ =
		ns_nmpc_utils::linspace<double>(tvec_base_normed(0), tvec_base_normed(tvec_base_normed.rows() - 1), nknots_);

	//  ns_nmpc_utils::print_container(knots_vec_);
	// Create the basis matrix from tvec_base to compute basis matrices for the base data.
	// Number of polynomials 4 + knots [1, t, t**2, t**3, (t-ki)**3, ....]

	if (!compute_derivatives_)
	{
		Eigen::MatrixXd basis_mat(n_base_points_, nknots_ + 4);
		Eigen::MatrixXd penalized_weights_D(n_base_points_, nknots_ + 4);
		basis_mat.setZero();
		penalized_weights_D.setIdentity();

		//  std::cout << "basis mat " << basis_mat.rows() << " " << basis_mat.cols() << std::endl;
		//  std::cout << "tvecbase mat " << tvec_base.rows() << " " << tvec_base.cols() << std::endl;
		// Create the bases matrices for b-spline, its first and second derivatives.

		// Create the bases matrices for b-spline using the base data size.
		createBasesMatrix(tvec_base_normed, basis_mat);

		// Create interpolating base matrix.
		Eigen::MatrixXd interpolating_base_mat(new_npoints_, nknots_ + 4);
		interpolating_base_mat.setZero();
		createBasesMatrix(tvec_new_normed, interpolating_base_mat);

		// Solve the matrix equations.
		solveByDemmlerReisch(basis_mat, penalized_weights_D);

		// solveByQR(basis_mat, penalized_weights_D);
		// Once the following matrix is computed, we multiply it with the given ybase
		// which yields ynew_in_new coordinates.
		// This new matrix; projection_mat_w_new_base_ can be used to interpolate ybase --> ynew
		// with a new length.
		// ynew_length = projection_mat_w_new_base_ @ ybase_length
		// ynew = A{A^T@A + lambda diag(s)}−1@A^T .

		projection_mat_w_new_base_ = interpolating_base_mat * projection_mat_base_;
	} else
	{
		// We don't need to create an additional penalized_weights_D, we have
		// a second derivative matrix here to be used
		// as a regularization matrix.
		// Base matrix to compute the coefficients from the data ybase.

		Eigen::MatrixXd basis_mat(n_base_points_, nknots_ + 4);

		// For optimal curvature, we need the second derivative of the basis matrix for the base data.
		Eigen::MatrixXd basis_mat_dd(n_base_points_, nknots_ + 4);
		basis_mat.setZero();
		basis_mat_dd.setIdentity();

		createBasesMatrix(tvec_base_normed, basis_mat);

		// createBasesMatrix(tvec_base_normed, basis_mat, basis_mat_dd);
		// if we use second derivative as regularization.
		// ns_nmpc_eigen_utils::printEigenMat(basis_mat_dd);
		// Compute the new interpolation base matrices. The coefficients computed
		// for the ybase and new bases.
		// y, y', y" = B, B', B" @ coeff_base.

		Eigen::MatrixXd new_basis_mat(new_npoints_, nknots_ + 4);
		Eigen::MatrixXd new_basis_d_mat(new_npoints_, nknots_ + 4);
		Eigen::MatrixXd new_basis_dd_mat(new_npoints_, nknots_ + 4);

		new_basis_mat.setZero();
		new_basis_d_mat.setZero();
		new_basis_dd_mat.setZero();

		createBasesMatrix(tvec_new_normed, new_basis_mat, new_basis_d_mat, new_basis_dd_mat);

		// Once the following matrix is computed, we multiply it with the given ybase which
		// yields ynew_in_new coordinates.
		// This new matrix; projection_mat_w_new_base_ can be used to
		// interpolate ybase --> ynew with a new length.
		// ynew_length = projection_mat_w_new_base_ @ ybase_length

		solveByDemmlerReisch(basis_mat, basis_mat_dd);
		// We cannot use these methods with the second derivative matrix,
		// singularity arises. We need use identity matrix for regularization.
		// solveByQR(basis_mat, basis_mat_dd);

		// ns_nmpc_eigen_utils::printEigenMat(projection_mat_base_);

		// Set Interpolating projection matrices
		projection_mat_w_new_base_ = new_basis_mat * projection_mat_base_;
		projection_mat_w_new_base_dot_ = new_basis_d_mat * projection_mat_base_;
		projection_mat_w_new_base_dot_dot_ = new_basis_dd_mat * projection_mat_base_;
	}
}

/*
 *  Create a basis matrix for the polynomial approximation. Basis matrix is parametrized by t, B = B(t)
 *
 * */
void ns_nmpc_splines::BSplineInterpolator::createBasesMatrix(const Eigen::VectorXd &tvec, Eigen::MatrixXd &basis_mat)
{
	/**
	 *     bspline_primitive = lambda t, knots: [1., t, t ** 2, t ** 3] + \
	 *                                    [((t - kn) ** 3 if (t - kn) > 0 else 0.) for kn in knots]
	 **/
	basis_mat.col(0).setConstant(1.);
	basis_mat.col(1) = tvec;
	basis_mat.col(2) = Eigen::VectorXd(tvec.unaryExpr([](auto const &x)
																										{ return x * x; }));

	basis_mat.col(3) = Eigen::VectorXd(tvec.unaryExpr([](auto const &x)
																										{ return x * x * x; }));

	for (auto k = 4; k < basis_mat.cols(); k++)
	{
		auto ki = knots_vec_[static_cast<size_t>(k - 4)];
		basis_mat.col(k) = Eigen::VectorXd(tvec.unaryExpr([&ki](auto const &x)
																											{
																												auto val_pos = std::max(0.0, (x - ki));

																												return std::pow(val_pos, 3);
																											}));

		// std::cout << ki << " knot " << std::endl;
	}
	//  ns_nmpc_eigen_utils::printEigenMat(basis_mat);
}

/**
 *  Create a basis matrix for the polynomial approximation. Basis matrix is parametrized by
 *  t, B = B(t). In addition to B(t), we compute a second derivative matrix that is used to
 *  regularize the curvature approximation. D = Bdd is second derivative of
 *  the base polynomial matrix.
 * */
void ns_nmpc_splines::BSplineInterpolator::createBasesMatrix(const Eigen::VectorXd &tvec,
																														 Eigen::MatrixXd &basis_mat,
																														 Eigen::MatrixXd &regularization_mat_dd)
{
	/**
	 *     bspline_primitive = lambda t, knots: [1., t, t ** 2, t ** 3] + \
	 *                                  [((t - kn) ** 3 if (t - kn) > 0 else 0.) for kn in knots]
	 **/

	// Base polynomial
	basis_mat.col(0).setConstant(1.);  // [1, ...]
	basis_mat.col(1) = tvec;           // [1, t, ...]
	basis_mat.col(2) = Eigen::VectorXd(tvec.unaryExpr([](auto const &x)
																										{ return x * x; }));    // [1, t, t**2]

	basis_mat.col(3) = Eigen::VectorXd(tvec.unaryExpr([](auto const &x)
																										{ return x * x * x; }));    // [1, t, t**2, t**3]

	// Second derivative
	/**
	 *   bspline_primitive_dot_dot = lambda t, knots: [0., 0, 2, 6*t] +
	 *                                        [(6*(t - kn)  if (t - kn) > 0 else 0.) for kn in knots]
	 *
	 * */

	regularization_mat_dd.leftCols(2).setZero();    // [0, 0]
	regularization_mat_dd.col(2).setConstant(2.0);  // [0, 0, 2]
	regularization_mat_dd.col(3) = 6.0 * tvec;      // [0, 0, 2, 6*t]

	for (auto k = 4; k < basis_mat.cols(); k++)
	{
		auto ki = knots_vec_[static_cast<size_t>(k - 4)];
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

	//  ns_nmpc_eigen_utils::printEigenMat(regularization_mat_dd);
	//  int a = 1;
}

/**
 *  For interpolating into the new dimension, with the derivatives, we need three new bases;
 *       B(t), Bdot(t), Bddot_dot(t).
 *
 *  Once we have these bases, we can use the same coefficients that are
 *  used to interpolate the base data to get new interpolated data along with its derivatives.
 *
 *  ynew = Bnew * same coeffs.
 *  y'new = B'new * same coeffs
 *  y''new = B''new * same coeffs.
 *
 * */

void ns_nmpc_splines::BSplineInterpolator::createBasesMatrix(const Eigen::VectorXd &tvec,
																														 Eigen::MatrixXd &basis_mat,
																														 Eigen::MatrixXd &basis_dmat,
																														 Eigen::MatrixXd &basis_ddmat)
{
	/**
	 * bspline_primitive = lambda t, knots: [1., t, t ** 2, t ** 3] + \
	 *                                    [((t - kn) ** 3 if (t - kn) > 0 else 0.) for kn in knots]
	 */

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
	 *                                        [(6*(t - kn)  if (t - kn) > 0 else 0.) for kn in knots]
	 *
	 * */

	//  ns_nmpc_eigen_utils::printEigenMat(tvec);
	//  ns_nmpc_eigen_utils::printEigenMat(basis_dmat);

	basis_ddmat.leftCols(2).setZero();    // [0, 0]
	basis_ddmat.col(2).setConstant(2.0);  // [0, 0, 2]
	basis_ddmat.col(3) = 6.0 * tvec;      // [0, 0, 2, t]

	for (auto k = 4; k < basis_mat.cols(); k++)
	{
		auto ki = knots_vec_[static_cast<size_t>(k - 4)];
		// (t-k)**3
		basis_mat.col(k) = Eigen::VectorXd(
			tvec.unaryExpr(
				[&ki](auto const &x)
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

	//  ns_nmpc_eigen_utils::printEigenMat(basis_dmat);
	//  int a = 1;
}

void ns_nmpc_splines::BSplineInterpolator::solveByDemmlerReisch(Eigen::MatrixXd const &basis_mat,
																																Eigen::MatrixXd const &penalizing_mat_D)
{
	// To be computed in the constructor.
	/**
	 *   y = Ax --> At*y = At*x -- > x = coeffs = inv(At*A)*A*ybase_data and A = A(t parameter)
	 *                          -- > ynew = Anew(t) * x  = Anew(t) * coeffs  = Anew(t) * inv(At*A)*A * ydata
	 *                          -- > projection matrix p = inv(At*A)*A this is projection_mat_base_.
	 *                          ---> projection with base included  Anew(t) * inv(At*A)*A    *
	 **/

	// projection_mat_base_ = Rinv * svd.matrixU() * (A.transpose() * A +
	// lambda_ * lambda_ * Sdiag).inverse() * A.transpose(); // y_interp = A(A^T@A + )

	// for minimum curvature smoothing penalization matrix.
	auto &&Dc = penalizing_mat_D.transpose() * penalizing_mat_D;
	auto &&Bc = basis_mat.transpose() * basis_mat;  // B^T @ B

	// Compute Cholesky.
	// Factorize (B&T@B + eps*D) = eps for numerical stability.
	Eigen::LLT<Eigen::MatrixXd> cholRRT(Bc + 1e-8 * Dc);
	Eigen::MatrixXd &&R = cholRRT.matrixL();  // Take the square root of (B&T@B + eps*D) = R^T@R
	Eigen::MatrixXd &&Rinv = R.inverse();

	//  ns_nmpc_eigen_utils::printEigenMat(Dc);
	//  ns_nmpc_eigen_utils::printEigenMat(Bc);
	//  ns_nmpc_eigen_utils::printEigenMat(Rinv);

	// Compute SVD
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(Rinv.transpose() * Dc * Rinv, Eigen::ComputeFullU | Eigen::ComputeFullV);

	//  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Rinv.transpose() * Dc * Rinv,
	//  Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::MatrixXd Sdiag(svd.singularValues().rows(), svd.singularValues().rows());
	Sdiag.setZero();

	Sdiag.diagonal() = svd.singularValues();
	auto &&A = basis_mat * Rinv * svd.matrixU();  // A = Basis @ Rinverse @ U

	// To be computed in the constructor.
	/**
	 *   y = Ax --> At*y = At*x -- > x = coeffs = inv(At*A)*A*ybase_data and A = A(t parameter)
	 *               -- > ynew = Anew(t) * x  = Anew(t) * coeffs  = Anew(t) * inv(At*A)*A * ydata
	 *               -- > projection matrix p = inv(At*A)*A this is projection_mat_base_.
	 *               ---> projection with base included  Anew(t) * inv(At*A)*A    *
	 *
	 * */

	projection_mat_base_ = Rinv * svd.matrixU() *
		(A.transpose() * A + lambda_ * lambda_ * Sdiag).inverse() *
		A.transpose();                       // y_interp = A(A^T@A + )
}

void ns_nmpc_splines::BSplineInterpolator::InterpolateInCoordinates(const Eigen::MatrixXd &ybase,
																																		Eigen::MatrixXd &data_tobe_interpolated) const
{
	//  std::cout << "ybase passed " << std::endl;
	//  ns_nmpc_eigen_utils::printEigenMat(ybase);

	// Standardize or normalize the data and restore back.
	auto numcols = static_cast<size_t>(ybase.cols());

	if (auto numrows = static_cast<size_t>(ybase.rows()); numrows <= 1)
	{
		std::cout << "\nNumber of rows must be more than 1 " << std::endl;
		return;
	}

	std::vector<double> colmaxvec(numcols);  // keep max(abs) in this vector for re-normalization.
	Eigen::MatrixXd ybase_normalized(ybase.rows(), ybase.cols());

	for (auto k = 0; k < numcols; k++)
	{
		auto &&colmax = ybase.col(k).cwiseAbs().maxCoeff();  // normalize each columns
		colmaxvec[k] = colmax;
		ybase_normalized.col(k) = ybase.col(k).unaryViewExpr([&colmax](auto const &x)
																												 { return x / colmax; });
	}

	//  ns_nmpc_eigen_utils::printEigenMat(ybase);
	//  ns_nmpc_eigen_utils::printEigenMat(ybase_normalized);
	//  ns_nmpc_eigen_utils::printEigenMat(projection_mat_w_new_base_);

	auto &&normalized_interpolated_data = projection_mat_w_new_base_ * ybase_normalized;
	data_tobe_interpolated.resize(normalized_interpolated_data.rows(), normalized_interpolated_data.cols());

	//  ns_nmpc_eigen_utils::printEigenMat(normalized_interpolated_data);
	for (auto k = 0; k < numcols; k++)
	{
		auto const &colmax = colmaxvec[k];
		data_tobe_interpolated.col(k) = normalized_interpolated_data.col(k).unaryExpr([colmax](auto const &x)
																																									{ return x * colmax; });
	}

	//  ns_nmpc_eigen_utils::printEigenMat(data_tobe_interpolated);

	// If we want to skip normalization procedures above, just use the
	// line below and comment out all the previous lines.
	//  data_tobe_interpolated = projection_mat_w_new_base_ * ybase;
}

void ns_nmpc_splines::BSplineInterpolator::solveByQR(Eigen::MatrixXd const &basis_mat,
																										 Eigen::MatrixXd const &penalizing_mat_D)
{
	Eigen::MatrixXd lambdaD = lambda_ * penalizing_mat_D;
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

	//  ns_nmpc_eigen_utils::printEigenMat(RinvQ1T);
	projection_mat_base_ = basis_mat * RinvQ1T;
}

void ns_nmpc_splines::BSplineInterpolator::getFirstDerivative(const Eigen::MatrixXd &ybase,
																															Eigen::MatrixXd &data_dot_tobe_interpolated)
{
	if (!compute_derivatives_)
	{
		std::cout << " The interpolator was not prepared by the compute_derivative_option = True. "
								 "Cannot return the "
								 "derivatives... ";
		return;
	}

	// Standardize or normalize the data and restore back.
	auto numcols = ybase.cols();
	if (auto numrows = ybase.rows(); numrows <= 1)
	{
		std::cout << "\nNumber of rows must be more than 1 " << std::endl;
		return;
	}

	std::vector<double> colmaxvec(static_cast<size_t>(numcols));
	Eigen::MatrixXd ybase_normalized(ybase.rows(), ybase.cols());

	for (auto k = 0; k < numcols; k++)
	{
		auto &&colmax = ybase.col(k).cwiseAbs().maxCoeff();
		colmaxvec[k] = colmax;
		ybase_normalized.col(k) = ybase.col(k).unaryViewExpr([&colmax](auto const &x)
																												 { return x / colmax; });
	}

	//  ns_nmpc_eigen_utils::printEigenMat(ybase);
	//  ns_nmpc_eigen_utils::printEigenMat(ybase_normalized);

	auto &&normalized_interpolated_data = projection_mat_w_new_base_dot_ * ybase_normalized;
	data_dot_tobe_interpolated.resize(normalized_interpolated_data.rows(), normalized_interpolated_data.cols());

	for (auto k = 0; k < numcols; k++)
	{
		auto &&colmax = colmaxvec[k];
		data_dot_tobe_interpolated.col(k) = normalized_interpolated_data.col(k).unaryExpr([&colmax](auto const &x)
																																											{ return x * colmax; });
	}

	// If we want to skip normalization procedures above,
	// just use the line below and comment out all the previous lines.
	// data_dot_tobe_interpolated = projection_mat_w_new_base_ * ybase;
}

void ns_nmpc_splines::BSplineInterpolator::getSecondDerivative(const Eigen::MatrixXd &ybase,
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
	auto numcols = ybase.cols();
	auto numrows = ybase.rows();

	if (numrows <= 1)
	{
		std::cout << "\nNumber of rows must be more than 1 " << std::endl;
		return;
	}

	std::vector<double> colmaxvec(static_cast<size_t>(numcols));
	Eigen::MatrixXd ybase_normalized(ybase.rows(), ybase.cols());

	for (auto k = 0; k < numcols; k++)
	{
		auto &&colmax = ybase.col(k).cwiseAbs().maxCoeff();
		colmaxvec[k] = colmax;
		ybase_normalized.col(k) =
			ybase.col(k).unaryViewExpr([&colmax](auto const &x)
																 { return x / colmax; });
	}

	//  ns_nmpc_eigen_utils::printEigenMat(ybase);
	//  ns_nmpc_eigen_utils::printEigenMat(ybase_normalized);

	auto &&normalized_interpolated_data = projection_mat_w_new_base_dot_dot_ * ybase_normalized;
	data_dot_dot_tobe_interpolated.resize(normalized_interpolated_data.rows(), normalized_interpolated_data.cols());

	for (auto k = 0; k < numcols; k++)
	{
		auto &&colmax = colmaxvec[k];
		data_dot_dot_tobe_interpolated.col(k) = normalized_interpolated_data.col(k).unaryExpr([&colmax](auto const &x)
																																													{ return x * colmax; });
	}

	// If we want to skip normalization procedures above, just use the line below and
	// comment out all the previous lines.
	//  data_dot_dot_tobe_interpolated = projection_mat_w_new_base_dot_dot_ * ybase;
}
