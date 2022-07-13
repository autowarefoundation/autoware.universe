/*
* Use of this source code is governed by an MIT-style license that can be found
* in the LICENSE file or at https://opensource.org/licenses/MIT.
*/

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

#ifndef VEHICLE_MODELS__VEHICLE_DYNAMICS_BASE_HPP_
#define VEHICLE_MODELS__VEHICLE_DYNAMICS_BASE_HPP_

#include <Eigen/StdVector>
#include <cmath>
#include <memory>
#include <vector>
#include "Eigen/Dense"
#include "kinematic_model_definitions.hpp"
#include "utils/codegen_eigen_support.hpp"
#include "utils/nmpc_utils.hpp"

#define CODEGEN true
//#if CODEGEN
//#include <cppad/cg.hpp>
//#else
//#include <cppad/cppad.hpp>
//#endif


/**
 * @brief Vehicle dynamics class that handles Automatic Differentiation tasks for the vehicle dynamics model classes.
 * The AD of dynamics is isolated from the vehicle model classes.
 * @tparam STATE_DIM number of states in the vehicle dynamics or kinematics,
 * @tparam eSTATE_DIM if error states are used in the equations, the number of error states,
 * @tparam INPUT_DIM number of inputs into the model,
 * @tparam PARAM_DIM number of parameters into the model, such as curvature in the Frenet frame models.
 * */
// cppcheck-suppress unknownMacro


template<int STATE_DIM, int INPUT_DIM, int PARAM_DIM, int eSTATE_DIM>
class VehicleDynamicsBase
{
 public:
	using state_vector_t = Eigen::Matrix<double, STATE_DIM, 1>;  // x, weights wx, z for f_0(x,u)
	using state_matrix_t = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;  // A, Q

	using input_vector_t = Eigen::Matrix<double, INPUT_DIM, 1>;          // u, wu
	using input_matrix_t = Eigen::Matrix<double, INPUT_DIM, INPUT_DIM>;  // B

	/**
	 * @brief for error model types to be used in LPV feedback and Kalman filters.
	 * */
	using error_state_vector_t = Eigen::Matrix<double, eSTATE_DIM, 1>;
	using error_state_matrix_t = Eigen::Matrix<double, eSTATE_DIM, eSTATE_DIM>;
	using error_input_matrix_t = Eigen::Matrix<double, eSTATE_DIM, INPUT_DIM>;

	using control_matrix_t = Eigen::Matrix<double, STATE_DIM, INPUT_DIM>;  // B

	// K if we use pre-stabilized feedback.
	using feedback_matrix_t = Eigen::Matrix<double, INPUT_DIM, STATE_DIM>;
	using param_vector_t = Eigen::Matrix<double, PARAM_DIM, 1>;  // kappa

	using dynamic_vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
	using dynamic_matrix_t = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
	using dynamic_vector_map_t = Eigen::Map<dynamic_vector_t>;

	using state_vector_v_t = std::vector<state_vector_t>;
	using input_vector_v_t = std::vector<input_vector_t>;

	using state_matrix_v_t = std::vector<state_matrix_t>;
	using control_matrix_v_t = std::vector<control_matrix_t>;

	// Diagonal matrices
	using state_diag_mat_t = Eigen::DiagonalMatrix<double, STATE_DIM>;
	using estate_diag_mat_t = Eigen::DiagonalMatrix<double, eSTATE_DIM>;
	using control_diag_mat_t = Eigen::DiagonalMatrix<double, INPUT_DIM>;

	// Pose [x, y, yaw]
	using pose_t = Eigen::Matrix<double, 3, 1>;

	// LPV type definitions
	// Feedback are computed using the error model.
	using lpv_state_vector_t = Eigen::Matrix<double, eSTATE_DIM, 1>;

	// Lyap. matrix X [nx, nx]
	using state_matrix_X_t = Eigen::Matrix<double, eSTATE_DIM, eSTATE_DIM>;

	// Lyap. matrix Y [nu, nx]
	using input_matrix_Y_t = Eigen::Matrix<double, INPUT_DIM, eSTATE_DIM>;

	// std Containers for X, and Y Lyap matrices are required to iterate over them.
	using lpv_X_matrix_v_t = std::vector<state_matrix_X_t>;
	using lpv_Y_matrix_v_t = std::vector<input_matrix_Y_t>;

	enum Dimensions : int
	{
		state_dim = STATE_DIM,
		estate_dim = eSTATE_DIM,  // error state dimension.
		input_dim = INPUT_DIM,
		param_dim = PARAM_DIM,
	};

#if CODEGEN
	using scalar_t = CppAD::cg::CG<double>;
#else
	using scalar_t = double;
#endif

	using scalar_ad_t = CppAD::AD<scalar_t>;

	using state_vector_ad_t = Eigen::Matrix<scalar_ad_t, STATE_DIM, 1>;
	using state_matrix_ad_t = Eigen::Matrix<scalar_ad_t, STATE_DIM, STATE_DIM>;

	using input_vector_ad_t = Eigen::Matrix<scalar_ad_t, INPUT_DIM, 1>;
	using control_matrix_ad_t = Eigen::Matrix<scalar_ad_t, STATE_DIM, INPUT_DIM>;

	using dynamic_vector_ad_t = Eigen::Matrix<scalar_ad_t, Eigen::Dynamic, 1>;
	using domain_vector_ad_t = Eigen::Matrix<scalar_ad_t, STATE_DIM + INPUT_DIM, 1>;
	using param_vector_ad_t = Eigen::Matrix<scalar_ad_t, PARAM_DIM, 1>;

	/**
	* @brief Initialize the model_ by compiling the dynamics functions.
	*/

	void InitializeModel();

	[[nodiscard]] bool IsInitialized() const
	{ return initialized_; }

	/**
	 * @brief Vehicle dynamics base class that defines vehicle model nonlinear and linear equations for automatic
	 * differentiation.
	 * @param x [in] Eigen 1D column matrix of states,
	 * @param u [in] Eigen 1d column matrix of inputs,
	 * @param kappa [in] scalar parameter - curvature,
	 * @param xdot [out]  xdot =f(x, u)
	 * */
	virtual void systemEquations(const state_vector_ad_t &x, const input_vector_ad_t &u,
															 const VehicleDynamicsBase::param_vector_ad_t &params,  // curvature, target vx
															 state_vector_ad_t &xdot_f) = 0;

	void computeFx(const state_vector_t &x, const input_vector_t &u,
								 const VehicleDynamicsBase::param_vector_t &params,  // curvature, target vx
								 state_vector_t &f);

	void computeJacobians(const state_vector_t &x, const input_vector_t &u, param_vector_t const &params,
												state_matrix_t &A, control_matrix_t &B);

 private:
	// cppAd function for xdot = f(x, u).
	CppAD::ADFun<scalar_t> f_;

#if CODEGEN
	std::unique_ptr<CppAD::cg::DynamicLib<double>> dynamicLib;
	std::unique_ptr<CppAD::cg::GenericModel<double>> model_;
#endif

	bool initialized_{false};
};

/**
 * @brief run AD recordings to record the process steps and initialize the AD model.
 * */
template<int STATE_DIM, int INPUT_DIM, int PARAM_DIM, int eSTATE_DIM>
void VehicleDynamicsBase<STATE_DIM, INPUT_DIM, PARAM_DIM, eSTATE_DIM>::InitializeModel()
{
	if (initialized_)
	{
		return;
	}

#if CODEGEN
	dynamic_vector_ad_t x(STATE_DIM + INPUT_DIM + PARAM_DIM);
	x.setRandom();

	// START RECORDING/
	CppAD::Independent(x, 0, false);

	const state_vector_ad_t state = x.segment<STATE_DIM>(0);
	const input_vector_ad_t input = x.segment<INPUT_DIM>(STATE_DIM);

	// cppAD doesn't accept scalar dynamic parameter. They are given in a vector form.
	const param_vector_ad_t params = x.segment<PARAM_DIM>(STATE_DIM + INPUT_DIM);

	// Prepare dx differential.
	state_vector_ad_t dx;

	// All the variables in system flow map is cppAD variables.
	systemEquations(state, input, params, dx);

	f_ = CppAD::ADFun<scalar_t>(x, dynamic_vector_ad_t(dx));
	f_.optimize();

	CppAD::cg::ModelCSourceGen cgen(f_, "model_");
	cgen.setCreateForwardZero(true);
	cgen.setCreateJacobian(true);
	CppAD::cg::ModelLibraryCSourceGen libcgen(cgen);

	// Compile source code.
	CppAD::cg::DynamicModelLibraryProcessor p(libcgen);

	CppAD::cg::GccCompiler<double> compiler;
	compiler.addCompileFlag("-O3");
	dynamicLib = p.createDynamicLibrary(compiler);

	model_ = dynamicLib->model("model_");

#else
	CppAD::thread_alloc::hold_memory(true);

	// Put state and input vectors in a dynamic vector concatenated.
	dynamic_vector_ad_t x(STATE_DIM + INPUT_DIM + PARAM_DIM);

	// We Separate since kappa=1 and ey=1 coincides with the singularity.
	//	x.segment<STATE_DIM + INPUT_DIM>(0).setOnes();  //
	//	x.segment<PARAM_DIM>(STATE_DIM + INPUT_DIM).setConstant(0.05);    // Parameter is the curvature, 1 is singularity.
	x.setRandom();

	// START RECORDING. We treat the parameter kappa as a part of independent variables.
	// Two separate parameters from
	// states and control requires two level operation recording.
	// see https://coin-or.github.io/CppAD/doc/change_param.cpp.htm.
	CppAD::Independent(x, 0, false);

	// Get the addresses of state and input vectors from the concatenated vector created above
	const state_vector_ad_t &state = x.template segment<STATE_DIM>(0);
	const input_vector_ad_t &input = x.template segment<INPUT_DIM>(STATE_DIM);
	const param_vector_ad_t &params = x.template segment<PARAM_DIM>(STATE_DIM + INPUT_DIM);

	state_vector_ad_t xdot;  // xdot = f(x)

	// All the variables in system flow map is cppAD variables.
	systemEquations(state, input, params, xdot);


	// store operation sequence in x' = f(x) and STOP RECORDING.
	f_ = CppAD::ADFun<scalar_t>(x, dynamic_vector_ad_t{xdot});
	f_.optimize();
#endif

	// Set initialized true.
	initialized_ = true;
}

/**
 * @brief Implements xdot=f(x) evaluation for Automatic Differentiation.
 * */
template<int STATE_DIM, int INPUT_DIM, int PARAM_DIM, int eSTATE_DIM>
void VehicleDynamicsBase<STATE_DIM,
												 INPUT_DIM,
												 PARAM_DIM,
												 eSTATE_DIM>::computeFx(const VehicleDynamicsBase::state_vector_t &x,
																								const VehicleDynamicsBase::input_vector_t &u,
																								const VehicleDynamicsBase::param_vector_t &params,
																								VehicleDynamicsBase::state_vector_t &f)
{
	assert(initialized_);

#if CODEGEN
	dynamic_vector_t input(STATE_DIM + INPUT_DIM + PARAM_DIM);
	input << x, u, params;

	CppAD::cg::ArrayView<const double> input_view(input.data(), static_cast<int32_t>(input.size()));
	CppAD::cg::ArrayView<double> f_view(f.data(), static_cast<int32_t>(f.size()));

	model_->ForwardZero(input_view, f_view);

#else
	dynamic_vector_t input(STATE_DIM + INPUT_DIM + PARAM_DIM);

	input << x, u, params;

	dynamic_vector_map_t f_map(f.data(), STATE_DIM);  // Maps arguments into the states.
	f_map << f_.Forward(0, input);                    // 0-th order forward operations.#endif
#endif
}

/**
 * @brief Implements Jacobian functions of the form xdot = f(x) = Adx + Bdu.
 * */
template<int STATE_DIM, int INPUT_DIM, int PARAM_DIM, int eSTATE_DIM>
void VehicleDynamicsBase<STATE_DIM, INPUT_DIM, PARAM_DIM, eSTATE_DIM>::computeJacobians(
	const VehicleDynamicsBase::state_vector_t &x, const VehicleDynamicsBase::input_vector_t &u,
	const VehicleDynamicsBase::param_vector_t &params, VehicleDynamicsBase::state_matrix_t &A,
	VehicleDynamicsBase::control_matrix_t &B)
{
	assert(initialized_);

#if CODEGEN
	dynamic_vector_t input(STATE_DIM + INPUT_DIM + PARAM_DIM);
	input << x, u, params;

	using full_jacobian_t =
		Eigen::Matrix<double, STATE_DIM, STATE_DIM + INPUT_DIM + PARAM_DIM, Eigen::RowMajor>;
	full_jacobian_t J;

	CppAD::cg::ArrayView<const double> input_view(input.data(), static_cast<int32_t>(input.size()));
	CppAD::cg::ArrayView<double> J_view(J.data(), static_cast<int32_t>(J.size()));

	model_->Jacobian(input_view, J_view);

#else
	dynamic_vector_t input(STATE_DIM + INPUT_DIM + PARAM_DIM);

	// Input to the AD functions [x, u, kappa]
	input << x, u, params;

	// Computed Jacobian is stored in.
	Eigen::Matrix<double, STATE_DIM, STATE_DIM + INPUT_DIM + PARAM_DIM, Eigen::RowMajor> J;
	dynamic_vector_map_t J_map(J.data(), J.size());

	J_map << f_.Jacobian(input);
#endif

	A = J.template block<STATE_DIM, STATE_DIM>(0, 0);
	B = J.template block<STATE_DIM, INPUT_DIM>(0, STATE_DIM);
}
#endif  // VEHICLE_MODELS__VEHICLE_DYNAMICS_BASE_HPP_
