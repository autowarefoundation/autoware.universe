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

#ifndef AUTOWARE_CONTROL_TOOLBOX_STATE_SPACE_HPP
#define AUTOWARE_CONTROL_TOOLBOX_STATE_SPACE_HPP

#include "visibility_control.hpp"
#include <vector>
#include "act_utils.hpp"
#include "act_utils_eigen.hpp"
#include "act_definitions.hpp"
#include "transfer_functions.hpp"
#include "balance.hpp"

namespace ns_control_toolbox
{


	/**
	 * @brief tf2ss Converts a transfer function representation in to a state-space form.
	 * We assume the system is SISO type.
	 *
	 * */

	class ACT_PUBLIC tf2ss
	{

	public:
		// Constructors
		tf2ss() = default;

		explicit tf2ss(tf const& sys_tf, const double& Ts = 0.1);

		tf2ss(std::vector<double> const& num, std::vector<double> const& den, const double& Ts = 0.1);


		// Public methods
		// Currently only Tustin - Bilinear discretization is implemented.
		void discretisize(double const& Ts);

		void print() const;

		void print_discrete_system() const;


		// Getters for the system matrices.
		// Discrete time state-space matrices.
		[[nodiscard]] Eigen::MatrixXd Ad() const
		{
			auto&& Ad = sys_matABCD_disc_.topLeftCorner(N_ - 1, N_ - 1);
			return Ad;
		}

		[[nodiscard]] Eigen::MatrixXd Bd() const
		{
			auto&& Bd = sys_matABCD_disc_.topRightCorner(N_ - 1, 1);
			return Bd;
		}

		[[nodiscard]] Eigen::MatrixXd Cd() const
		{
			auto&& Cd = sys_matABCD_disc_.bottomLeftCorner(1, N_ - 1);
			return Cd;
		}

		[[nodiscard]] Eigen::MatrixXd Dd() const
		{
			auto&& Dd = sys_matABCD_disc_.bottomRightCorner(1, 1);
			return Dd;
		}


		// Continuous time state-space matrices.
		[[nodiscard]] Eigen::MatrixXd A() const
		{
			auto&& A = sys_matABCD_cont_.topLeftCorner(N_ - 1, N_ - 1);
			return A;
		}

		[[nodiscard]] Eigen::MatrixXd B() const
		{
			auto&& B = sys_matABCD_cont_.topRightCorner(N_ - 1, 1);
			return B;
		}

		[[nodiscard]] Eigen::MatrixXd C() const
		{
			auto&& C = sys_matABCD_cont_.bottomLeftCorner(1, N_ - 1);
			return C;
		}

		[[nodiscard]] Eigen::MatrixXd D() const
		{
			auto&& D = sys_matABCD_cont_.bottomRightCorner(1, 1);
			return D;
		}

		// Class methods.

		/**
		 * @brief Compute the system continuous time system matrices
		 * */
		void computeSystemMatrices(std::vector<double> const& num,
		                           std::vector<double> const& den);


	private:

		double Ts_{};
		Eigen::Index N_; // system size (A.rows+1).

		// Data members

		// system matrices  in a single matrix form of [A, B;C, D]
		/**
		 *	A_ = ss_system.topLeftCorner(nx, nx);
		 *	B_ = ss_system.topRightCorner(nx, 1);
		 *	C_ = ss_system.bottomLeftCorner(1, nx);
		 *	D_ = ss_system.bottomRightCorner(1, 1);
		 * */
		Eigen::MatrixXd sys_matABCD_cont_{};
		Eigen::MatrixXd sys_matABCD_disc_{};

	};


	// Type definitions.
	template<int nx, int ny>
	using mat_type_t = Eigen::Matrix<double, nx, ny>;

	template<int N>
	using state_type_t = Eigen::Matrix<double, N, 1>;

} // namespace ns_control_toolbox
#endif //AUTOWARE_CONTROL_TOOLBOX_STATE_SPACE_HPP
