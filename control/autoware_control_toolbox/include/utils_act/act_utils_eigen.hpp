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

#ifndef UTILS__ACT_UTILS_EIGEN_HPP_
#define UTILS__ACT_UTILS_EIGEN_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Sparse>
#include "eigen3/Eigen/Core"

namespace ns_eigen_utils
{
	template<typename T>
	using eigen_dynamic_type = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

	template<typename T>
	using eigen_vector_type = Eigen::Matrix<T, Eigen::Dynamic, 1>;

/**
 * @brief  Gives a new shape to an array without changing its data.
 *         This function is based on numpy.reshape
 *         see: https://docs.scipy.org/doc/numpy/reference/generated/numpy.reshape.html
 *
 * @param x input matrix
 * @param r the number of row elements
 * @param c the number of collum elements
 *
 * @return The new shape matrix
 */

	template<typename T>
	constexpr eigen_dynamic_type<T> reshape(
			Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& x, size_t const r, size_t const c)
	{
		Eigen::Map<eigen_dynamic_type<T>> rx(x.data(), r, c);
		return rx;
	}

/**
 * @brief Stack matrix in sequence vertically
 *        imspired by numpy.vstack
 *        https://docs.scipy.org/doc/numpy/reference/generated/numpy.vstack.html
 *
 * @param m1 first matrix
 * @param m2 second matrix *
 * @return stacked matrix
 */
	template<typename T, class M>
	constexpr eigen_dynamic_type<T> vstack(M const& m1, M const& m2)
	{
		if (m1.rows() == 0)
		{
			return m2;

		}
		if (m2.rows() == 0)
		{
			return m1;
		}

		size_t ncol = m1.cols();
		if (ncol == 0)
		{
			ncol = m2.cols();
		}

		eigen_dynamic_type<T> rm(m1.rows() + m2.rows(), ncol);
		rm << m1, m2;
		return rm;
	}

	template<typename T, class M, class ... Args>
	constexpr eigen_dynamic_type<T> vstack(M const& first, Args const& ... args)
	{
		return vstack(first, vstack(args ...));
	}

/**
 * @brief Stack matrix in sequence horizontally
 *        inspired by numpy.hstack
 *        https://docs.scipy.org/doc/numpy/reference/generated/numpy.hstack.html
 *
 * @param m1 first matrix
 * @param m2 second matrix
 *
 * @return stacked matrix
 */
	template<typename T, class M>
	constexpr eigen_dynamic_type<T> hstack(M const& m1, M const& m2)
	{
		if (m1.cols() == 0)
		{
			return m2;
		}
		if (m2.cols() == 0)
		{
			return m1;
		}

		Eigen::Index nrow = m1.rows();
		if (nrow == 0)
		{
			nrow = m2.rows();
		}

		eigen_dynamic_type<T> rm(nrow, m1.cols() + m2.cols());
		rm << m1, m2;

		return rm;
	}

	template<typename T, class M, class ... Args>
	constexpr eigen_dynamic_type<T> hstack(M const& first, Args const& ... args)
	{
		return hstack(first, hstack(args ...));
	}

	template<typename T>
	constexpr eigen_dynamic_type<T> block_diag(
			eigen_dynamic_type<T> const& m1, eigen_dynamic_type<T> const& m2)
	{
		size_t m1r = m1.rows();
		size_t m1c = m1.cols();
		size_t m2r = m2.rows();
		size_t m2c = m2.cols();

		eigen_dynamic_type<T> mf = eigen_dynamic_type<T>::Zero(m1r + m2r, m1c + m2c);
		mf.block(0, 0, m1r, m1c) = m1;
		mf.block(m1r, m1c, m2r, m2c) = m2;

		return mf;
	}

	template<typename T, class ... Args>
	constexpr eigen_dynamic_type<T> block_diag(
			eigen_dynamic_type<T> const& first, Args const& ... args)
	{
		return block_diag(first, block_diag(args ...));
	}

/**
 * @brief Computes the Kronecker product
 *        A composite array made of blocks of the second array scaled by the first
 *        Inspired numpy.kron
 *        see: https://docs.scipy.org/doc/numpy/reference/generated/numpy.kron.html
 *
 * @param m1 first matrix
 * @param m2 second matrix
 *
 * @return A result of the Kronecker product
 */
	template<typename T, typename Derived>
	constexpr eigen_dynamic_type<T> kron(const Derived& m1, const eigen_dynamic_type<T>& m2)
	{
		size_t const m1r = m1.rows();
		size_t const m1c = m1.cols();

		size_t const m2r = m2.rows();
		size_t const m2c = m2.cols();

		eigen_dynamic_type<T> m3(m1r * m2r, m1c * m2c);

		for (size_t i = 0; i < m1r; i++)
		{
			for (size_t j = 0; j < m1c; j++)
			{
				m3.block(i * m2r, j * m2c, m2r, m2c) = m1(i, j) * m2;
			}
		}

		return m3;
	}

	template<typename T, typename Derived>
	constexpr eigen_dynamic_type<T> kron(const eigen_dynamic_type<T>& m1, Derived const& m2)
	{
		size_t m1r = m1.rows();
		size_t m1c = m1.cols();

		size_t m2r = m2.rows();
		size_t m2c = m2.cols();

		eigen_dynamic_type<T> m3(m1r * m2r, m1c * m2c);

		for (size_t i = 0; i < m1r; i++)
		{
			for (size_t j = 0; j < m1c; j++)
			{
				m3.block(i * m2r, j * m2c, m2r, m2c) = m1(i, j) * m2;
			}
		}

		return m3;
	}

	template<typename T, typename Derived1, typename Derived2>
	constexpr eigen_dynamic_type<T> kron(const Derived1& m1, const Derived2& m2)
	{
		size_t const m1r = m1.rows();
		size_t m1c = m1.cols();

		size_t const m2r = m2.rows();
		size_t const m2c = m2.cols();

		eigen_dynamic_type<T> m3(m1r * m2r, m1c * m2c);

		for (size_t i = 0; i < m1r; i++)
		{
			for (size_t j = 0; j < m1c; j++)
			{
				m3.block(i * m2r, j * m2c, m2r, m2c) = m1(i, j) * m2;
			}
		}

		return m3;
	}

	template<typename T>
	eigen_dynamic_type<T> convertToEigenMatrix(std::vector<std::vector<T>> data)
	{
		eigen_dynamic_type<T> eMatrix(data.size(), data[0].size());
		for (size_t i = 0; i < data.size(); ++i)
		{
			eMatrix.row(i) = eigen_vector_type<T>::Map(&data[i][0], data[0].size());
		}
		return eMatrix;
	}

	template<class M, typename S=std::string_view>
	void printEigenMat(M const& mat, S additional_info = "")
	{
		std::cout << additional_info;
		std::string sep = "\n----------------------------------------\n";
		Eigen::IOFormat CommaInitFmt(
				Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

		Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
		Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
		Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

		std::cout << '\n' << mat.format(CleanFmt) << sep;
	}

	template<typename T, typename S=std::string_view>
	void printEigenMat(eigen_dynamic_type<T> const& mat, S additional_info)
	{
		std::string sep = "\n----------------------------------------\n";
		Eigen::IOFormat CommaInitFmt(
				Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

		Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
		Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
		Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

		std::cout << additional_info << " : \n" << mat.format(CleanFmt) << sep;
	}

/**
 * @brief Computes the cross product given two vectors in the form r0(x, y) = [x0, y0] and r1(x, y) = [x1, y1] .
 *
 * */
	template<typename T>
	eigen_dynamic_type<T> crossProduct(eigen_dynamic_type<T> const r0, eigen_dynamic_type<T> const r1)
	{
		auto m = r0.rows();
		eigen_dynamic_type<T> cross_product(m, 1);
		cross_product.setZero();

		for (size_t k = 0; k < m; k++)
		{
			cross_product(k) = r0(k, 0) * r1(k, 1) - r0(k, 1) * r1(k, 0);
		}

		return cross_product;
	}

/**
 *  Apply the curvature expression defined in the differential geometry.
 * */
	template<typename T>
	eigen_dynamic_type<T> Curvature(
			eigen_dynamic_type<T> const rdot, eigen_dynamic_type<T> const rdotdot)
	{
		auto m = rdot.rows();
		eigen_dynamic_type<T> curvature(m, 1);
		curvature.setZero();

		auto&& EPS = std::numeric_limits<double>::epsilon();
		for (Eigen::Index k = 0; k < m; k++)
		{
			auto&& cubed_norm = std::pow(rdot.row(k).norm(), 3) + EPS;
			curvature(k) = (rdot(k, 0) * rdotdot(k, 1) - rdot(k, 1) * rdotdot(k, 0)) / cubed_norm;
		}
		return curvature;
	}

// Prepare std vector of Eigen triplets.
// ------------------- to Triplet -------------------------------------------------
/**
 * @brief Gets the nonzero elements of a given array or matrix given a EPS threshold.
 * @tparam T scalar type
 * @tparam ArgType Eigen matrix class.
 * */
	template<typename T, class ArgType>
	std::vector<std::vector<double>> ToTriplets(
			const Eigen::MatrixBase<ArgType>& arg_mat, T const& eps_tol, size_t rowstart = 0,
			size_t colstart = 0)
	{
		auto BoolMat = arg_mat.cwiseAbs().array() >= eps_tol;

		// Get nonzero indices.
		// std::vector<unsigned int> NNZind;
		Eigen::VectorXi NNZflatind = Eigen::VectorXi::LinSpaced(BoolMat.size(), 0, BoolMat.size() - 1);

		auto NNZindit = std::stable_partition(
				NNZflatind.data(), NNZflatind.data() + NNZflatind.size(),
				[&BoolMat](int i)
				{ return BoolMat(i); });

		NNZflatind.conservativeResize(NNZindit - NNZflatind.data());

		// printEigenMat(NNZflatind);
		// ColMajor Index and value extraction.
		size_t const& n = arg_mat.rows();
		size_t const& m = arg_mat.cols();

		// Triplet manipulation.
		std::vector<std::vector<T>> triplet_list;
		triplet_list.reserve(NNZflatind.size());

		for (Eigen::Index k = 0; k < NNZflatind.size(); k++)
		{
			auto val = arg_mat(NNZflatind[k]);
			size_t col = NNZflatind[k] / (ArgType::Flags & Eigen::RowMajorBit ? m : n);
			size_t row = NNZflatind[k] % (ArgType::Flags & Eigen::RowMajorBit ? m : n);

			triplet_list.template emplace_back(std::vector<T>{ row + rowstart, col + colstart, val });
			// triplet_list.template emplace_back(triplet_type(row, col, val));
		}

		return triplet_list;
	}

/**
 * @brief prints a trajectory container in the matrix form.
 * */
	template<template<typename, typename> class ContainerType, typename ValueType, typename AllocType>
	void printTrajectory(const ContainerType<ValueType, AllocType>& eigen_matrix_vec)
	{
		auto nX = eigen_matrix_vec.size();
		auto const nrow = eigen_matrix_vec[0].rows();
		Eigen::MatrixXd X(nrow, nX);
		X.setZero();

		for (Eigen::Index k = 0; k < nX; k++)
		{
			X.col(k) = eigen_matrix_vec[k];
		}

		printEigenMat(X);
	}

	template<template<typename, typename> class ContainerType, typename ValueType, typename AllocType>
	Eigen::MatrixXd getTrajectory(const ContainerType<ValueType, AllocType>& eigen_matrix_vec)
	{
		auto nX = eigen_matrix_vec.size();
		auto const nrow = eigen_matrix_vec[0].rows();
		Eigen::MatrixXd X(nrow, nX);
		X.setZero();

		for (size_t k = 0; k < nX; k++)
		{
			X.col(static_cast<Eigen::Index>(k)) = eigen_matrix_vec[k];
		}

		return X;
	}

/**
 *
 * @param char type_R u- for Rupper, l for Rlower
 * */
	template<typename T, class Derived1, class Derived2>
	void backSubstitution(
			eigen_dynamic_type<T> const& R,
			Eigen::MatrixBase<Derived1> const& B,
			Eigen::MatrixBase<Derived2>& Xtobesolved, char type_of_R)
	{
		auto&& mrow = B.rows();
		auto&& ncol = R.cols();


		Xtobesolved.setZero();

		if (type_of_R == 'u')
		{
			for (auto i = 0; i < mrow; ++i)
			{
				Xtobesolved(i, 0) = B(i, 0) / R(0, 0);

				for (auto k = 1; k < ncol; ++k)
				{
					Xtobesolved(i, k) = B(i, k);

					for (auto j = 0; j < k; ++j)
					{
						Xtobesolved(i, k) = Xtobesolved(i, k) - Xtobesolved(i, j) * R(j, k);
					}

					Xtobesolved(i, k) = Xtobesolved(i, k) / R(k, k);
				}
			}
		}
		else
		{
			for (auto i = 0; i < mrow; ++i)
			{
				Xtobesolved(i, ncol - 1) = B(i, ncol - 1) / R(ncol - 1, ncol - 1);

				for (auto k = ncol - 2; k >= 0; --k)
				{
					Xtobesolved(i, k) = B(i, k);

					for (auto j = k + 1; j < ncol; ++j)
					{
						Xtobesolved(i, k) = Xtobesolved(i, k) - Xtobesolved(i, j) * R(j, k);
					}

					Xtobesolved(i, k) = Xtobesolved(i, k) / R(k, k);
				}
			}
		}

	}

/**
        #brief Rank-1 update for the Cholesky factor.
        https://en.wikipedia.org/wiki/Cholesky_decomposition#Rank-one_update
        A = L*L' --> Atilda = Ltilda*Ltilda' where Atilda = L*L' + w*v*v'
        @param R: Upper triangular square matrix cholesky factor
        @param U: the matrix the column of which is used to update  S
        @param weight: sign of which defines update or downdate

 */
	template<typename T, class Derived1, class Derived2>
	void cholesky_update(
			Eigen::MatrixBase<Derived1>& R,
			Eigen::MatrixBase<Derived2> const& U, T const& weight)
	{


		//        Eigen::MatrixXd R{Rsqrt};
		//        std::cout << "Printing input R ... in cholesky update ";
		//        printEigenMat(R);
		//
		//        std::cout << "Printing input U ... in cholesky update ";
		//        printEigenMat(U);

		double rowsum{};

		// Check if R is upper triangular.
		for (auto i = 0; i < R.rows(); ++i)
		{
			rowsum += R(i, 0);
		}

		if (std::fabs(rowsum - R(0, 0)) > std::numeric_limits<double>::epsilon())
		{
			std::cout << " Matrix is not Upper Triangular " << "\n";
			return;
		}

		auto const&& sign = sgn(weight);
		auto const&& weight_abs = std::fabs(weight);
		auto const&& nrows = U.rows();

		for (auto j = 0; j < U.cols(); ++j)
		{
			eigen_dynamic_type<T> x(weight_abs * U.col(j));

			for (auto k = 0; k < nrows; ++k)
			{
				auto const&& r_squared = R(k, k) * R(k, k) + sign * x(k, 0) * x(k, 0);
				auto const&& r = r_squared < 0 ? 0. : std::sqrt(r_squared);
				auto const&& c = r / R(k, k);
				auto const&& s = x(k, 0) / R(k, k);
				R(k, k) = r;

				R.row(k).rightCols(nrows - 1 - k) =
						(R.row(k).rightCols(nrows - 1 - k) + sign * s * x.bottomRows(nrows - 1 - k).transpose()) /
						c;

				x.bottomRows(nrows - k - 1) =
						c * x.bottomRows(nrows - k - 1) - s * R.row(k).rightCols(nrows - 1 - k).transpose();

			}
		}

		//        std::cout << "Printing output R ... in cholesky update ";
		//        printEigenMat(R.eval());
		//
		//        std::cout << "Printing output Rsqrt ... in cholesky update ";
		//        printEigenMat(Rsqrt.eval());

	}
}  // namespace ns_eigen_utils
#endif  // UTILS__ACT_UTILS_EIGEN_HPP_
