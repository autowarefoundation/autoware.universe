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

#include "utils_act/balance.hpp"
#include "utils_act/act_utils_eigen.hpp"
#include "utils_act/act_utils.hpp"
#include "utils_act/act_definitions.hpp"

/**
 * @brief Balances a matrix using the Lapack algorithms.
 * James, R., Langou, J. and Lowery, B.R., 2014. On matrix balancing and eigenvector computation. arXiv preprint
 * arXiv:1401.5766.
 *
 * Numerical Recipes in C: The Art of Scientific Computing, Second Edition Balancing Chapter 11.
 * */

void ns_control_toolbox::balance(Eigen::MatrixXd& A)
{

	ns_utils::print("in balance eigen \n");

	// get the size of the matrix.
	auto const& nx = A.rows();
	bool converged{ false };

	while (!converged)
	{
		converged = true;

		for (auto k = 0; k < nx; ++k)
		{

			// Compute row and column norms.
			auto r = A.row(k).lpNorm<1>(); // instead of 1-norm, one can use other norms as well.
			auto c = A.col(k).lpNorm<1>();

			// In the lapack implementation diagonal element is ignored in the norms.
			r = r - std::abs(A.row(k)(k));
			c = c - std::abs(A.col(k)(k));

			// auto s = std::pow(r, 1) + std::pow(c, 1);
			auto&& s = r + c;
			double f{ 1. };

			if ((r > EPS) && (c > EPS))
			{
				while (c < r / RADIX)
				{
					c = c * RADIX;
					r = r / RADIX;
					f = f * RADIX;
				}


				while (c > r * RADIX)
				{
					c = c / RADIX;
					r = r * RADIX;
					f = f / RADIX;

				}

				// Convergence block.
				if (c + r < 0.95 * s)
				{
					converged = true;

					// Apply similarity transformation to the rows and cols.
					A.col(k).noalias() = A.col(k) * f;
					A.row(k).noalias() = A.row(k) / f;
				}

			}

			else
			{
				// Apply similarity transformation to the rows and cols.
				A.col(k).noalias() = A.col(k) * f;
				A.row(k).noalias() = A.row(k) / f;
			}
		}

	}

}

