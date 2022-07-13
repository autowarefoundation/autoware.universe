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

#ifndef NONLINEAR_MPC_CORE__NMPC_DATA_DISCRETIZATION_HPP_
#define NONLINEAR_MPC_CORE__NMPC_DATA_DISCRETIZATION_HPP_

#include <Eigen/StdVector>  // must be before the vector but clang re-orders.
#include <vector>

namespace ns_data
{
template<class Model>
struct DiscretizationData
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typename Model::state_matrix_v_t A;  // !<-@brief vector of state transition matrices A.


	// !<-@brief vector of control matrices at the beginning of time interval
	typename Model::control_matrix_v_t B;

	// !<-@brief vector of control matrices at the end of time interval
	typename Model::control_matrix_v_t C;

	typename Model::state_vector_v_t z;  // !<-@brief f0 - Ax0 - Bu0 - Cu1

	void initializeDiscretizationMatrices(size_t const &K, double const &dt_step);
	[[nodiscard]] std::size_t nX() const;

	[[nodiscard]] __attribute__((unused)) std::size_t nU() const;

	double dt{};
};

template<class Model>
void DiscretizationData<Model>::initializeDiscretizationMatrices(size_t const &K, double const &dt_step)
{

	A.resize(K - 1, Model::state_matrix_t::Zero());     // !<-@brief vector of As
	B.resize(K - 1, Model::control_matrix_t::Zero());   // !<-@brief vector of Bs
	C.resize(K - 1, Model::control_matrix_t::Zero());   // !<-@brief vector of Bs at the end of the section.
	z.resize(K - 1, Model::state_vector_t::Zero());     // !<-@brief f0 - Ax0 - Bu0 - Cu1
	dt = dt_step;

}

template<class Model>
std::size_t DiscretizationData<Model>::nX() const
{
	return A.size();
}

template<class Model>
std::size_t DiscretizationData<Model>::nU() const
{
	return B.size();
}

}  // namespace ns_data

#endif  // NONLINEAR_MPC_CORE__NMPC_DATA_DISCRETIZATION_HPP_
