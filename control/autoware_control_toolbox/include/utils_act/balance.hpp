// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef AUTOWARE_CONTROL_TOOLBOX_BALANCE_HPP
#define AUTOWARE_CONTROL_TOOLBOX_BALANCE_HPP

#include "act_utils_eigen.hpp"
#include "act_utils.hpp"

namespace ns_control_toolbox
{
	/**
	 * @brief Reduces a matrix to a upper Hessenberg form.
	 * */
	void permute(Eigen::MatrixXd& P);


	/**
	 * @brief Balances a matrix by similarity transformation of form Ab = D^{-1} A D.
	 * */

	void balance_a_matrix(Eigen::MatrixXd& A, Eigen::MatrixXd& Tsimilarity);

	void balance_ssABCD_(Eigen::MatrixXd& ssABCD);

} // namespace ns_control_toolbox


#endif //AUTOWARE_CONTROL_TOOLBOX_BALANCE_HPP
