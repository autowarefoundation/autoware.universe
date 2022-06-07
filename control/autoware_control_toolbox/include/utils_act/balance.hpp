// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_CONTROL_TOOLBOX_BALANCE_HPP
#define AUTOWARE_CONTROL_TOOLBOX_BALANCE_HPP

#include "act_utils.hpp"
#include "act_utils_eigen.hpp"

namespace ns_control_toolbox
{
/**
 * @brief Reduces a matrix to a upper Hessenberg form.
 * */
void permute(Eigen::MatrixXd & P);

/**
 * @brief Balances a matrix by similarity transformation of form Ab = D^{-1} A D.
 * see http://www.ece.northwestern.edu/local-apps/matlabhelp/toolbox/control/ref/ssbal.html.
 * */

void balance_a_matrix(Eigen::MatrixXd & A, Eigen::MatrixXd & Tsimilarity);
/**
 * @brief Used to balance a single row-r and column-c norm balancing. Finds an alpha such that r=
 * r*alpha and c=c/alpha. In the method, ahat = a*alpha, bhat = b/alpha order is followed.
 * @param c: A column norm.
 * @param r: A row norm. In general l1-norm is used.
 *
 *
 * */
double balance_symmetric(double const & a, double const & b);

}  // namespace ns_control_toolbox

#endif  // AUTOWARE_CONTROL_TOOLBOX_BALANCE_HPP
