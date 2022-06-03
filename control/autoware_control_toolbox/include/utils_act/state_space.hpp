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
	 * @brief Stores the state space model matrices either discrete, or continuous.
	 * */

	struct ss_system
	{
		ss_system() = default;

		explicit ss_system(Eigen::MatrixXd const& Am,
		                   Eigen::MatrixXd const& Bm,
		                   Eigen::MatrixXd const& Cm,
		                   Eigen::MatrixXd const& Dm);

		Eigen::MatrixXd A{};
		Eigen::MatrixXd B{};
		Eigen::MatrixXd C{};
		Eigen::MatrixXd D{};

	};


/**
 * @brief tf2ss Converts a transfer function representation in to a state-space form.
 * We assume the system is SISO type.
 *
 * */

	struct ACT_PUBLIC tf2ss
	{

		// Constructors
		tf2ss();

		tf2ss(tf const& sys_tf, const double& Ts = 0.1);

		tf2ss(std::vector<double> const& num, std::vector<double> const& den, const double& Ts = 0.1);


		// Public methods
		// Currently only Tustin - Bilinear discretization is implemented.
		void discretisize(double const& Ts);

		void print() const;

		void print_discrete_system() const;


		[[nodiscard]] ss_system get_ssABCD_continuous() const;

		[[nodiscard]] ss_system get_ssABCD_discrete() const;

		[[nodiscard]] Eigen::MatrixXd const& Ad() const
		{
			return Ad_;
		}

		[[nodiscard]] Eigen::MatrixXd const& Bd() const
		{
			return Bd_;
		}

		[[nodiscard]] Eigen::MatrixXd const& Cd() const
		{
			return Cd_;
		}

		[[nodiscard]] Eigen::MatrixXd const& Dd() const
		{
			return Dd_;
		}

		[[nodiscard]] Eigen::MatrixXd const& A() const
		{
			return A_;
		}

		[[nodiscard]] Eigen::MatrixXd const& B() const
		{
			return B_;
		}

		[[nodiscard]] Eigen::MatrixXd const& C() const
		{
			return C_;
		}

		[[nodiscard]] Eigen::MatrixXd const& D() const
		{
			return D_;
		}

		// Constant reference to discrete matrices.



	private:

		double Ts_{};

		/**
		 * @brief Compute the system continuous time system matrices
		 * */
		void computeSystemMatrices(std::vector<double> const& num,
		                           std::vector<double> const& den);

		// Data members
		// Continuous time state-space model
		Eigen::MatrixXd A_{};
		Eigen::MatrixXd B_{};
		Eigen::MatrixXd C_{};
		Eigen::MatrixXd D_{};

		// Discrete time state-space model
		Eigen::MatrixXd Ad_{};
		Eigen::MatrixXd Bd_{};
		Eigen::MatrixXd Cd_{};
		Eigen::MatrixXd Dd_{};

	};


} // namespace ns_control_toolbox
#endif //AUTOWARE_CONTROL_TOOLBOX_STATE_SPACE_HPP
