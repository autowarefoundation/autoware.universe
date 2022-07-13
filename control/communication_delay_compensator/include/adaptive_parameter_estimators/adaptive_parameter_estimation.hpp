// Copyright 2022 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DELAY_COMPENSATOR_INCLUDE_ADAPTIVE_PARAMETER_ESTIMATORS_ADAPTIVE_PARAMETER_ESTIMATION_HPP
#define DELAY_COMPENSATOR_INCLUDE_ADAPTIVE_PARAMETER_ESTIMATORS_ADAPTIVE_PARAMETER_ESTIMATION_HPP

#include "node_denifitions/node_definitions.hpp"

/**
 * Ioannou, P.A. and Sun, J., 1996. Robust adaptive control (Vol. 1). Upper Saddle River, NJ: PTR Prentice-Hall.
 * Chapter 4.4
 * */

namespace observers
{
		/**
		 * @brief estimates the parameters of a model in the form xdot = a*x + b*u; 
		 * */
		class AdaptiveParameterEstimator
		{
		public:
				AdaptiveParameterEstimator() = default;
				AdaptiveParameterEstimator(float64_t const& amin,
				                           float64_t const& amax,
				                           float64_t const& bmin,
				                           float64_t const& bmax,
				                           float64_t const& dt);

				void updateEstimates(float64_t const& x, float64_t const& u);
				void getCurrentEstimates_ab(float64_t& a_estimate, float64_t& b_estimate);

		private:
				float64_t dt_{};
				float64_t const am_{ 1. };
				float64_t beta_{ 0.0 }; // forgetting factor
				float64_t epsilon_{ 0.1 }; // smoothing factor.
				float64_t M0_{ 200. }; // constraint on parameters dot product

				// Covariance resetting
				float64_t rho_0{ 0.5 };
				float64_t rho_1{ 1e5 };

				// Parameter maximum and minimums.
				float64_t amax_{};
				float64_t amin_{};
				float64_t bmax_{};
				float64_t bmin_{};

				float64_t xhat0_{}; // initial conditions.
				float64_t zhat0_{};
				float64_t ehat0_{};

				// Covariance matrix.
				Eigen::Matrix<double, 2, 2> P_;
				Eigen::Matrix<double, 2, 2> C0_; // normalization factor  xhat = c0*x + c1 where xhat \in[-1, 1]
				Eigen::Matrix<double, 2, 1> C1_;
				Eigen::Matrix<double, 2, 1> phi_;
				Eigen::Matrix<double, 2, 2> I_; // identity matrix
				Eigen::Matrix<double, 2, 2> Z_; // zero matrix

				/**
				 * @brief Parameters to be updated for real-time estimation.
				 * hat{x_dot} = hat{a}x + hat{b}u where hat denotes the predicted variables.
				 *
				 * */
				Eigen::Matrix<double, 2, 1> theta_ab_; // xdot = -a*x + bu where a and b ara unknown.

				// Value of Jacobian with respec to the parameters
				void Jacobian(float64_t const& ymeasured, float64_t const& ypredicted);

				// Class methods.
				/**
				 * @brief project a parameter on to a convex set, using vector projection methods.
				 * */
				bool isProjectionNeeded(Eigen::Matrix<double, 2, 1> const& Pe_phi);

		};
} // namespace observers
#endif //DELAY_COMPENSATOR_INCLUDE_ADAPTIVE_PARAMETER_ESTIMATORS_ADAPTIVE_PARAMETER_ESTIMATION_HPP
