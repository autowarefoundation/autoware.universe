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

#include "nonlinear_mpc_core/nmpc_discretization.hpp"
#include <vector>
#include "utils/nmpc_utils.hpp"

ns_discretization::ODEfoh::ODEfoh(
	Model::model_ptr_t model, const Model::input_vector_t &u0, const Model::input_vector_t &u1,
	Model::param_vector_t const &params0, Model::param_vector_t const &params1, double dt)
	: model_(model), u0_(u0), u1_(u1), params0_(params0), params1_(params1), dt_(dt)
{
}

ns_discretization::ODEzoh::ODEzoh(
	Model::model_ptr_t model, const Model::input_vector_t &u0, Model::param_vector_t const &params0,
	double dt)
	: model_(model), u0_(u0), params0_(params0), dt_(dt)
{
}

// --------- Integration Operator -------------------

void ns_discretization::ODEfoh::operator()(
	const ode_matrix_t &V, ode_matrix_t &dVdt, const double t)
{
	const Model::state_vector_t x = V.col(0);
	Model::input_vector_t u(Model::input_vector_t::Zero());

	// InterpolateInCoordinates the inputs and the parameters.
	u = u0_ + t / dt_ * (u1_ - u0_);
	auto const &&params = params0_ + t / dt_ * (params1_ - params0_);

	/** @brief  Declare A, B, f */
	Model::state_matrix_t A(Model::state_matrix_t::Zero());
	Model::control_matrix_t B(Model::control_matrix_t::Zero());
	Model::state_vector_t f(Model::state_vector_t::Zero());

	/** @brief Compute Ac, Bc, f. */
	model_->computeJacobians(x, u, params, A, B);
	model_->computeFx(x, u, params, f);

	/**
	 * @brief Compute alpha and beta (lambda_, and lambda+ in Successive convexification)
	 * which are computed from:
	 *
	 *  alpha = (1-lambda), beta = lambda and any vector x = (1-lambda) * x0 +
	 * lambda*(x0). Used to interpolate the input matrices B0 and B1.
	 **/

	// When using FOH integration, we interpolate the inputs.
	// const double alpha = (dt_ - t) / dt_;
	// const double beta = t / dt_;

	// Compute z - residuals of Taylor expansion.
	Model::state_vector_t z = (f - A * x - B * u);

	// Compute Ak, B0k and B1k.
	Eigen::Index cols = 0;

	// Phi in Fundamental matrix.
	const Model::state_matrix_t Phi_A_xi = V.template block<Model::state_dim, Model::state_dim>(0, 1);
	const Model::state_matrix_t Phi_A_xi_inverse = Phi_A_xi.inverse();
	// ns_nmpc_eigen_utils::printEigenMat(Phi_A_xi_inverse);

	// x[k+1]
	dVdt.template block<Model::state_dim, 1>(0, cols) = f;
	cols += 1;

	// Ak
	dVdt.template block<Model::state_dim, Model::state_dim>(0, cols).noalias() = A * Phi_A_xi;
	cols += Model::state_dim;

	// Bk - No input interpolation, ZOH.
	dVdt.template block<Model::state_dim, Model::input_dim>(0, cols).noalias() = Phi_A_xi_inverse * B;
	cols += Model::input_dim;

	// In case we use First Order Hold (FOH).
	//  // Bk
	//  dVdt.template block<Model::state_dim, Model::input_dim>(0, cols).noalias()
	//  = Phi_A_xi_inverse * B * alpha; cols += Model::input_dim;

	//  // Ck
	//  dVdt.template block<Model::state_dim, Model::input_dim>(0, cols).noalias()
	//  = Phi_A_xi_inverse * B * beta; cols += Model::input_dim;

	// zk
	dVdt.template block<Model::state_dim, 1>(0, cols).noalias() = Phi_A_xi_inverse * z;
	dVdt.template block<Model::state_dim, 1>(0, cols).noalias() = Phi_A_xi_inverse * z;
	cols += 1;

	assert(cols == ode_matrix_t::ColsAtCompileTime);
}

void ns_discretization::ODEzoh::operator()(
	const ode_matrix_t &V, ode_matrix_t &dVdt, const double t)
{
	const Model::state_vector_t x = V.col(0);
	Model::input_vector_t u(Model::input_vector_t::Zero());

	/** @brief  Declare A, B, f */
	Model::state_matrix_t A(Model::state_matrix_t::Zero());
	Model::control_matrix_t B(Model::control_matrix_t::Zero());
	Model::state_vector_t f(Model::state_vector_t::Zero());

	/** @brief Compute Ac, Bc, f. */
	model_->computeJacobians(x, u0_, params0_, A, B);
	model_->computeFx(x, u0_, params0_, f);

	// Compute z - residuals of Taylor expansion.
	Model::state_vector_t z(f - A * x - B * u0_);

	// Compute Ak, B0k and B1k.
	Eigen::Index cols = 0;

	// Phi in Fundamental matrix.
	const Model::state_matrix_t Phi_A_xi = V.template block<Model::state_dim, Model::state_dim>(0, 1);
	const Model::state_matrix_t Phi_A_xi_inverse = Phi_A_xi.inverse();

	// x[k+1]
	dVdt.template block<Model::state_dim, 1>(0, cols) = f;
	cols += 1;

	// Ak
	dVdt.template block<Model::state_dim, Model::state_dim>(0, cols).noalias() = A * Phi_A_xi;
	cols += Model::state_dim;

	// Bk - No input interpolation, ZOH.
	dVdt.template block<Model::state_dim, Model::input_dim>(0, cols).noalias() = Phi_A_xi_inverse * B;
	cols += Model::input_dim;

	// zk
	dVdt.template block<Model::state_dim, 1>(0, cols).noalias() = Phi_A_xi_inverse * z;
	dVdt.template block<Model::state_dim, 1>(0, cols).noalias() = Phi_A_xi_inverse * z;
	cols += 1;

	assert(cols == ode_matrix_t::ColsAtCompileTime);
}

bool ns_discretization::multipleShootingTrajectory(
	Model::model_ptr_t const &model_ptr, trajectory_data_t const &trajectory_data,
	trajectory_data_t const &target_states,
	ns_nmpc_splines::InterpolatingSplinePCG const &piecewise_interpolator, double const &dt,
	discretization_data_t &dd)
{

	size_t const &&K = dd.nX();  // number of matrices in the state matrix container.
	double const num_of_tsteps = 2;  // number of time steps for the RK integration. 1 for a single step.

	// Start computing Ak, B0k, B1k and zk where zk is the residual in the Taylor
	// expansion; x[k+1]  = Ax + Bu + f(xk, uk) - Axk - Buk and zk = f(xk, uk) -
	// Axk - Buk. States are ['xw', 'yw', 'psi', 's', 'ey', 'epsi', 'Vx', 'delta']

	// To initialize the Boost integrator, we need a state_type.
	using ode_matrix_t = typename ODEfoh::ode_matrix_t;

	//  runge_kutta_dopri5<ode_matrix_t, double, ode_matrix_t, double,
	//  vector_space_algebra> stepper; euler<ode_matrix_t, double, ode_matrix_t,
	//  double, vector_space_algebra> stepper;

	boost::numeric::odeint::runge_kutta4<
		ode_matrix_t, double, ode_matrix_t, double, boost::numeric::odeint::vector_space_algebra>
		stepper;

	//  boost::numeric::odeint::runge_kutta_fehlberg78<
	//    ode_matrix_t, double, ode_matrix_t, double, boost::numeric::odeint::vector_space_algebra>
	//    stepper;

	// Create an ODEzoh matrix placeholder.
	ode_matrix_t V(ode_matrix_t::Zero());

	// Define param vectors place holders.
	Model::param_vector_t params0(Model::param_vector_t::Zero());
	Model::param_vector_t params1(Model::param_vector_t::Zero());

	for (size_t k = 0; k < K; k++)
	{
		// Compute kappa0, kappa1.
		auto const x0 = trajectory_data.X.at(k);
		auto const x1 = trajectory_data.X.at(k + 1);

		auto const u0 = trajectory_data.U.at(k);
		auto const u1 = trajectory_data.U.at(k + 1);

		auto const &s0 = x0(3);  //  ['xw', 'yw', 'psi', 's', 'ey', 'epsi', 'Vx', 'delta']
		auto const &s1 = x1(3);
		std::vector<double> s0s1{s0, s1};  // we interpolate at two coordinates
		std::vector<double> kappa0_kappa1;

		// InterpolateInCoordinates wit PCG setting reusing_param true.
		auto const &&could_interpolate = piecewise_interpolator.Interpolate(s0s1, kappa0_kappa1);

		if (!could_interpolate)
		{
			// ROS_ERROR("[nonlinear_mpc-discretization]
			// Couldn't InterpolateInCoordinates at time %zu of %zu ", k, K);
			return false;
		}

		params0(0) = kappa0_kappa1[0];
		params0(1) = target_states.X[k](6);  // virtual car speed

		params1(0) = kappa0_kappa1[1];
		params1(1) = target_states.X[k + 1](6);  // virtual car speed

		// Create ODEzoh a functor that keeps internal parameters to facilitate Boost  OdeInt.
		// u1 is redundant in the ZOH method, but we keep there for FOH to switch the methods easily.
		ODEfoh ode_multipleshooting(model_ptr, u0, u1, params0, params1, dt);

		// Prepare ODEzoh matrix V.
		V.setZero();

		// Assign ODEzoh matrix V columns.
		V.col(0) = trajectory_data.X.at(k);
		V.template block<Model::state_dim, Model::state_dim>(0, 1).setIdentity();

		// Set the rest of the columns zero.
		/**
		 *  Number of columns in V; total= 1 for x0, nx for states, 2 * nu for the
		 * controls. total - 1- nx is the rest of the number of columns.
		 * */

		V.template rightCols<ode_matrix_t::ColsAtCompileTime - 1 - Model::state_dim>().setZero();

		// Integrate ODEzoh matrices. We can increase the integration accuracy
		// dt/ nRKorder defines the number of integration steps.
		boost::numeric::odeint::integrate_adaptive(
			stepper, ode_multipleshooting, V, 0., dt, dt / num_of_tsteps);

		// Assign the discrete matrices. Vectorized matrices start at column 1.
		Eigen::Index cols = 1;

		dd.A[k] = V.template block<Model::state_dim, Model::state_dim>(0, cols);
		cols += Model::state_dim;

		dd.B[k].noalias() = dd.A[k] * V.template block<Model::state_dim, Model::input_dim>(0, cols);
		cols += Model::input_dim;

		dd.z[k].noalias() = dd.A[k] * V.template block<Model::state_dim, 1>(0, cols);
		cols += 1;

		assert(cols == ode_matrix_t::ColsAtCompileTime);
	}

	return true;
}

// We can also use the Bi-linear transformation method for discretization.
[[maybe_unused]] bool ns_discretization::bilinearTransformation(
	Model::model_ptr_t const &model_ptr, trajectory_data_t const &trajectory_data,
	trajectory_data_t const &target_state,
	ns_nmpc_splines::InterpolatingSplinePCG const &piecewise_interpolator, double const &ts,
	discretization_data_t &dd)
{
	size_t const &K = dd.nX();  // number of matrices in the state matrix container.

	// Start computing Ak, B0k, B1k and zk where zk is the residual in the Taylor
	// expansion; x[k+1]  = Ax + Bu + f(xk, uk) - Axk - Buk and zk = f(xk, uk) -
	// Axk - Buk. States are ['xw', 'yw', 'psi', 's', 'ey', 'epsi', 'Vx', 'delta']

	//  auto scol = ref_map.col(0);
	//  auto kappa_col = ref_map.col(4);

	auto const &&Id = Model::state_matrix_t::Identity();

	// A, B, f placeholders.
	Model::state_matrix_t Ac(Model::state_matrix_t::Zero());
	Model::control_matrix_t Bc(Model::control_matrix_t::Zero());
	Model::state_vector_t f(Model::state_vector_t::Zero());

	Model::param_vector_t params(Model::param_vector_t::Zero());

	for (size_t k = 0; k < K; k++)
	{
		auto const x_eq = trajectory_data.X.at(k);
		auto const u_eq = trajectory_data.U.at(k);

		auto const &s0 = x_eq(3);
		double kappa0{};

		// InterpolateInCoordinates wit PCG setting reusing_param true.
		auto const &&could_interpolate = piecewise_interpolator.Interpolate(s0, kappa0);

		if (!could_interpolate)
		{
			// ROS_ERROR("[nonlinear_mpc-discretization] Couldn't InterpolateInCoordinates
			// at time %zu of %zu ", k, K);
			return false;
		}

		params(0) = kappa0;
		params(1) = target_state.X[k](6);

		model_ptr->computeJacobians(x_eq, u_eq, params, Ac, Bc);
		model_ptr->computeFx(x_eq, u_eq, params, f);

		Model::state_vector_t z = f - Ac * x_eq - Bc * u_eq;

		// Bi-linear Transformation.
		Model::state_matrix_t &&I_At2_inv = (Id - Ac * ts / 2).inverse();

		dd.A[k] = I_At2_inv * (Id + Ac * ts / 2);
		dd.B[k] = I_At2_inv * Bc * ts;
		dd.z[k] = I_At2_inv * z * ts;
	}

	return true;
}

bool multipleShootingSingleStep(
	Model::model_ptr_t const &model_ptr, Model::state_vector_t const &x0,
	Model::input_vector_t const &u0, double const &curvature_0, double const &dt,
	Model::state_matrix_t &Ad, Model::control_matrix_t &Bd);

// One-step discretization methods.
bool ns_discretization::multipleShootingSingleStep(
	Model::model_ptr_t const &model_ptr, Model::state_vector_t const &x0,
	Model::input_vector_t const &u0, Model::param_vector_t const &params0, double const &dt,
	Model::state_matrix_t &Ad, Model::control_matrix_t &Bd)
{
	double const num_of_tsteps{4};  // Number of time steps for the RK integration. 1 for a single step.

	// Start computing Ak, B0k, B1k and zk where zk is the residual in the Taylor
	// expansion; x[k+1]  = Ax + Bu + f(xk, uk) - Axk - Buk and zk = f(xk, uk) -
	// Axk - Buk. States are ['xw', 'yw', 'psi', 's', 'ey', 'epsi', 'Vx', 'delta']

	// To initialize the Boost integrator, we need a state_type.
	using ode_matrix_t = typename ODEzoh::ode_matrix_t;

	//  runge_kutta_dopri5<ode_matrix_t, double, ode_matrix_t, double,
	//  vector_space_algebra> stepper; euler<ode_matrix_t, double, ode_matrix_t,
	//  double, vector_space_algebra> stepper;

	boost::numeric::odeint::runge_kutta4<
		ode_matrix_t, double, ode_matrix_t, double, boost::numeric::odeint::vector_space_algebra>
		stepper;

	// runge_kutta_fehlberg78<ode_matrix_t, double, ode_matrix_t,
	// double, vector_space_algebra> stepper;

	// Create an ODEzoh matrix placeholder.
	ode_matrix_t V(ode_matrix_t::Zero());

	// Create ODEzoh a functor that keeps internal parameters to facilitate Boost  OdeInt.
	// u1 is redundant in the ZOH method, but we keep there for FOH to switch the methods easily.
	ODEzoh ode_multipleshooting(model_ptr, u0, params0, dt);

	// Prepare ODEzoh matrix V.
	V.setZero();

	// Assign ODEzoh matrix V columns.
	V.col(0) = x0;
	V.template block<Model::state_dim, Model::state_dim>(0, 1).setIdentity();

	// Set the rest of the columns zero.
	/**
		 *  Number of columns in V; total= 1 for x0, nx for states, 2 * nu for the
		 * controls. total - 1- nx is the rest of the number of columns.
		 * */

	V.template rightCols<ode_matrix_t::ColsAtCompileTime - 1 - Model::state_dim>().setZero();

	// Integrate ODEzoh matrices. We can increase the integration accuracy
	// dt/ nRKorder defines the number of integration steps.
	boost::numeric::odeint::integrate_adaptive(
		stepper, ode_multipleshooting, V, 0., dt, dt / num_of_tsteps);

	// Assign the discrete matrices. Vectorized matrices start at column 1.
	Eigen::Index cols = 1;

	Ad = V.template block<Model::state_dim, Model::state_dim>(0, cols);
	cols += Model::state_dim;

	Bd.noalias() = Ad * V.template block<Model::state_dim, Model::input_dim>(0, cols);
	// cols += Model::input_dim;

	// auto zk = Ad * V.template block<Model::state_dim, 1>(0, cols);
	// cols += 1;

	return true;
}

// We can also use the Bi-linear transformation method for discretization.
bool ns_discretization::bilinearTransformationOneStep(
	Model::model_ptr_t const &model_ptr, Model::state_vector_t const &x0,
	Model::input_vector_t const &u0, Model::param_vector_t const &params0, double const &dt,
	Model::state_matrix_t &Ad, Model::control_matrix_t &Bd)
{
	// Start computing Ak, B0k, B1k and zk where zk is the residual in the Taylor
	// expansion; x[k+1]  = Ax + Bu + f(xk, uk) - Axk - Buk and zk = f(xk, uk) -
	// Axk - Buk. States are ['xw', 'yw', 'psi', 's', 'ey', 'epsi', 'Vx', 'delta']

	auto const &&Id = Model::state_matrix_t::Identity();

	// Ac, Bc, fc placeholders. Continuous matrix variables.
	Model::state_matrix_t Ac(Model::state_matrix_t::Zero());
	Model::control_matrix_t Bc(Model::control_matrix_t::Zero());
	Model::state_vector_t f(Model::state_vector_t::Zero());

	model_ptr->computeJacobians(x0, u0, params0, Ac, Bc);

	// model_ptr->computeFx(x0, u0, curvature_0, f);
	// Model::state_vector_t z = f - Ac * x0 - Bc * u0;

	// Bi-linear Transformation.
	Model::state_matrix_t &&I_At2_inv = (Id - Ac * dt / 2).inverse();

	Ad = I_At2_inv * (Id + Ac * dt / 2);
	Bd = I_At2_inv * Bc * dt;

	// auto zd = I_At2_inv * z * dt;
	return true;
}
