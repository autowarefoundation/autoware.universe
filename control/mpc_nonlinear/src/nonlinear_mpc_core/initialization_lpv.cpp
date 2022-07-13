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

#include "nonlinear_mpc_core/initialization_lpv.hpp"
#include <vector>

LPVinitializer::LPVinitializer(const LPVinitializer &other)
{
	if (&other != this)
	{
		ntheta_ = other.ntheta_;
		thetas_ = other.thetas_;
	}
}

LPVinitializer &LPVinitializer::operator=(const LPVinitializer &other)
{
	if (&other != this)
	{
		ntheta_ = other.ntheta_;
		thetas_ = other.thetas_;
	}
	return *this;
}

// LPVinitializer::LPVinitializer(LPVinitializer &&other) noexcept
// {
//    if (&other != this)
//    {
//        ntheta_ = other.ntheta_;
//        thetas_ = std::move(other.thetas_);
//    }
//}
//
// LPVinitializer &LPVinitializer::operator=(LPVinitializer &&other) noexcept
// {
//    if (&other != this)
//    {
//        ntheta_ = other.ntheta_;
//        thetas_ = std::move(other.thetas_);
//    }
//    return *this;
//
// }

bool LPVinitializer::simulateWithFeedback(
	Model::model_ptr_t const &model_ptr,
	ns_nmpc_splines::InterpolatingSplinePCG const &piecewise_interpolator,
	ns_data::param_lpv_type_t const &params_lpv, ns_data::ParamsOptimization const &params_opt,
	ns_data::data_nmpc_core_type_t &nmpc_data)
{
	// Get the size of the trajectory.
	size_t K = nmpc_data.trajectory_data.nX();  // number of state vectors stored in the std::vector.

	// Prepare an error state vector.
	/**
		 * Full states are [x, y, psi, s, e_y, e_yaw, v, delta],
		 * and the error states [e_y, e_yaw, v, delta].
		 * */
	Model::error_state_vector_t x_error;
	x_error.setZero();

	// Set instrumental x and u to keep the results of the simulation (inplace integration).
	auto xk = nmpc_data.trajectory_data.X[0];  // current value is x0.
	auto uk = nmpc_data.trajectory_data.U[0];

	//  x  =[xw, yw, psi, s, e_y, e_yaw, v, delta]
	double dt = nmpc_data.mpc_prediction_dt;

	// Define placeholders for the system matrices.
	Model::state_matrix_t Ac;
	Model::control_matrix_t Bc;

	Ac.setZero();
	Bc.setZero();

	// Define Lyapunov matrices placeholders.
	Model::state_matrix_X_t Xr;
	Model::input_matrix_Y_t Yr;

	Xr.setZero();
	Yr.setZero();

	// Prepare the initial and final cost,
	// so that we can observe whether the controller diverges.
	double initial_error_cost{};
	double final_error_cost{};

	/**
		 * @brief feedback control value resulting in primal_infeasible in the optimization.
		 * We further narrow down the control values by the following percentage.
		 */

	double narrow_boundries = 0.9;  // More saturation on the control prediction.

	// Prepare param vector placeholder.
	Model::param_vector_t params;
	params.setZero();

	for (size_t k = 0; k < K; k++)
	{
		// Compute feedback control using the Lyapunov matrices X[k] = sum(theta_i*X_i) and Y...
		// respectively. We need the nonlinear theta parameters that represent the nonlinear
		// terms in the xdot = A(theta)*x + B(theta)*u

		// Update the error model states. [ey, e_yaw, eV, delta]
		x_error << xk.middleRows(4, Model::estate_dim);

		auto const &vtarget = nmpc_data.target_reference_states_and_controls.X.at(k)(6);
		x_error(2) = xk(6) - vtarget;  // [ey, epsi, error_vx, delta]

		// Get the s-state (distance travelled) and interpolate for the curvature
		// value at this distance point.
		auto const &s0 = xk(3);
		double kappa0{};

		// ns_nmpc_utils::print("s vs curvature in LPV feedback : ", s0, kappa0);

		if (auto const &&could_interpolate = piecewise_interpolator.Interpolate(s0, kappa0);!could_interpolate)
		{
			// std::cerr <<"[nonlinear_mpc]: LPV spline interpolator failed to compute the
			// coefficients..." << std::endl;
			return false;
		}

		// Compute the state transition matrices to get the values of the nonlinear terms
		// in the state transition mat Ac.

		params(0) = kappa0;
		params(1) = vtarget;
		model_ptr->computeJacobians(xk, uk, params, Ac, Bc);

		// if (k == 0) {
		//   ns_nmpc_utils::print("Ac   k =0 ");

		//   ns_nmpc_eigen_utils::printEigenMat(Ac);

		//   ns_nmpc_utils::print("Bc  k =0 ");
		//   ns_nmpc_eigen_utils::printEigenMat(Bc);

		//   ns_nmpc_utils::print("params k =0 ");
		//   ns_nmpc_eigen_utils::printEigenMat(params);
		// }

		// Compute the thetas - values of the nonlinearities in the state transition matrix Ac.
		// We use the only one-block Ac where the error states reside.
		auto const &Ac_error_block = Ac.template block<Model::estate_dim, Model::estate_dim>(4, 4);
		auto const &th1 = Ac_error_block(0, 1);
		auto const &th2 = Ac_error_block(0, 2);
		auto const &th3 = Ac_error_block(0, 3);

		auto const &th4 = Ac_error_block(1, 0);
		auto const &th5 = Ac_error_block(1, 1);
		auto const &th6 = Ac_error_block(1, 2);
		auto const &th7 = Ac_error_block(1, 3);

		// Compute parametric Lyapunov matrices.
		/**
		 * X = sum(theta_i * X_i).
		 * */
		thetas_ = std::vector<double>{th1, th2, th3, th4, th5, th6, th7};

		// Extract the first X0, Y0, we save the first X0 and Y0 at the end.
		Xr = params_lpv.lpvXcontainer.back();  // We keep the first X0, Y0 at the end of the containers.
		Yr = params_lpv.lpvYcontainer.back();

		// if (k == 0) {
		//   ns_nmpc_utils::print("Xr at k =0 ");
		//   ns_nmpc_eigen_utils::printEigenMat(Xr);
		//   ns_nmpc_utils::print_container(thetas_);
		// }

		for (size_t j = 0; j < ntheta_; j++)
		{
			Xr += thetas_[j] * params_lpv.lpvXcontainer[j];
			Yr += thetas_[j] * params_lpv.lpvYcontainer[j];
		}

		// if (k == 0) {
		//   ns_nmpc_utils::print("Xr at k =0 after summing up before the inverse");
		//   ns_nmpc_eigen_utils::printEigenMat(Xr);
		// }

		// Compute Feedback coefficients.
		auto const &Pr = Xr.inverse();  // Cost matrix P.
		auto const &Kfb = Yr * Pr;      // State feedback coefficients matrix.
		uk = Kfb * x_error;              // Feedback control signal.

		uk(0) =
			ns_nmpc_utils::clamp(uk(0), params_opt.ulower(0) * narrow_boundries, params_opt.uupper(0) * narrow_boundries);

		uk(1) =
			ns_nmpc_utils::clamp(uk(1), params_opt.ulower(1) * narrow_boundries, params_opt.uupper(1) * narrow_boundries);

		// Saturate xk(7) delta
		xk(7) =
			ns_nmpc_utils::clamp(xk(7), params_opt.xlower(7) * narrow_boundries, params_opt.xupper(7) * narrow_boundries);

		ns_sim::simulateNonlinearModel_zoh(model_ptr, uk, params, dt, xk);

		// Update xk, uk
		nmpc_data.trajectory_data.X[k] = xk;
		nmpc_data.trajectory_data.U[k] = uk;

		// Assign initial and final costs.
		if (k == 0)
		{
			initial_error_cost = x_error.transpose() * Pr * x_error;

			// ns_nmpc_utils::print("k == 0, initial error xerror");
			// ns_nmpc_eigen_utils::printEigenMat(Eigen::MatrixXd(x_error));
			// ns_nmpc_utils::print("k == 0,  Pr");
			// ns_nmpc_eigen_utils::printEigenMat(Eigen::MatrixXd(Pr));
		}

		if (k == K - 1)
		{
			final_error_cost = x_error.transpose() * Pr * x_error;
		}

		if (final_error_cost > initial_error_cost)
		{
			ns_nmpc_utils::print("[nonlinear_mpc - LPVinit] LPV Feedback control increases the cost ...");
		}

		// DEBUG
		//    ns_nmpc_utils::print("\nIn LPV initialization :");
		//    ns_nmpc_utils::print("\nTarget Vx : ", vtarget);
		//
		//    ns_nmpc_utils::print("\nSystem Matrices in the LPVinit :");
		//    ns_nmpc_eigen_utils::printEigenMat(Ac);
		//    ns_nmpc_eigen_utils::printEigenMat(Bc);
		//
		//    ns_nmpc_utils::print("Interpolated Lyapunov Matrices", ':', " X[k] and Y[k]");
		//    ns_nmpc_eigen_utils::printEigenMat(Xr);
		//    ns_nmpc_eigen_utils::printEigenMat(Yr);
		//
		//    ns_nmpc_utils::print("\nFeedback matrix : ");
		//    ns_nmpc_eigen_utils::printEigenMat(Kfb);
		//    ns_nmpc_utils::print("Feedback controls : ", uk(0), ", ", uk(1));
		// end of debug
	}

	// DEBUG
	// Get trajectories as a matrix and print for debugging purpose.
	// auto && Xtemp = ns_nmpc_eigen_utils::getTrajectory(nmpc_data.trajectory_data.X);
	// auto && Utemp = ns_nmpc_eigen_utils::getTrajectory(nmpc_data.trajectory_data.U);

	// ns_nmpc_utils::print("\nComputed LPV trajectories : ");
	// ns_nmpc_eigen_utils::printEigenMat(Xtemp.transpose());  //  [x, y, psi, s, ey, epsi, v, delta]

	// ns_nmpc_utils::print("\nComputed LPV trajectories U : ");
	// ns_nmpc_eigen_utils::printEigenMat(Utemp.transpose());

	// // Check the initial and final error cost.
	// ns_nmpc_utils::print("Initial and final error costs ; ", initial_error_cost, "  ", final_error_cost);
	// ns_nmpc_utils::print(
	//   "Is final error cost smaller than the initial error cost ? :",
	//   initial_error_cost > final_error_cost);
	// end of DEBUG

	return true;
}

bool LPVinitializer::computeSingleFeedbackControls(Model::model_ptr_t const &model_ptr,
																									 ns_nmpc_splines::InterpolatingSplinePCG const &piecewise_interpolator,
																									 ns_data::param_lpv_type_t const &params_lpv,
																									 ns_data::ParamsOptimization const &params_opt,
																									 ns_data::data_nmpc_core_type_t &nmpc_data,
																									 double const &dt)
{
	// Prepare an error state vector.
	/**
	 * Full states are [x, y, psi, s, ey, epsi, v, delta],
	 *  and the error states [ey, epsi, v, delta].
	 * */
	Model::lpv_state_vector_t x_error;
	x_error.setZero();

	double s0{};
	double kappa0{};  // curvature placeholder
	bool could_interpolate{};

	// Set instrumental x, u to keep the results of the simulation (inplace integration) computations.
	auto xk = nmpc_data.trajectory_data.X[0];  // Copy the current value of x0.

	Model::input_vector_t uk;  // Input to the model is [ax, steering_rate]
	uk.setZero();

	Model::param_vector_t params;
	params.setZero();

	// interval. x  =[xw, yw, psi, s, e_y, e_yaw, v, delta]

	// Define placeholders for the system matrices.
	Model::state_matrix_t Ac;
	Model::control_matrix_t Bc;

	Ac.setZero();
	Bc.setZero();

	// Define Lyapunov matrices placeholders.
	Model::state_matrix_X_t Xr;
	Model::input_matrix_Y_t Yr;

	Xr.setZero();
	Yr.setZero();

	// Get the current error states.
	// Update the error model states. [ey, epsi, eV, delta]
	x_error << xk.middleRows(4, Model::estate_dim);

	// Compute the error in the longitudinal speed and update x_error by this value.
	// [x, y, psi, s, ey, epsi, v, delta] x[6] -> vx.

	auto vx_target = nmpc_data.target_reference_states_and_controls.X[0](6);  //
	x_error(2) = xk(6) - vx_target;  // [ey, epsi, error_vx, delta] - error in vx tracking.

	// Get the s-state (distance travelled) and interpolate for
	// the curvature value at this distance point.
	s0 = xk(3);

	// InterpolateInCoordinates kappa[k].
	could_interpolate = piecewise_interpolator.Interpolate(s0, kappa0);

	if (!could_interpolate)
	{
		std::cerr << "[nonlinear_mpc_core] LPV spline interpolator failed to "
								 "compute the coefficients ..."
							<< std::endl;
		return false;
	}

	params(0) = kappa0;
	params(1) = vx_target;

	// Compute the state transition matrices to get the values of the nonlinear terms
	// in the state transition mat Ac.
	model_ptr->computeJacobians(xk, uk, params, Ac, Bc);

	// Compute the thetas - values of the nonlinearities in the state transition matrix Ac.
	// We use the only one-block Ac where the error states reside.
	auto Ac_error_block = Ac.template block<Model::estate_dim, Model::estate_dim>(4, 4);
	auto th1 = Ac_error_block(0, 1);
	auto th2 = Ac_error_block(0, 2);
	auto th3 = Ac_error_block(0, 3);

	auto th4 = Ac_error_block(1, 0);
	auto th5 = Ac_error_block(1, 1);
	auto th6 = Ac_error_block(1, 2);
	auto th7 = Ac_error_block(1, 3);

	// Compute the parametric Lyapunov matrices.
	/**
	 *   X = sum(theta_i * X_i).
	 *
	 * */

	thetas_ = std::vector<double>{th1, th2, th3, th4, th5, th6, th7};

	// Extract the first X0, Y0.
	Xr = params_lpv.lpvXcontainer.back();  // We keep the first X0, Y0 at the end of the containers.
	Yr = params_lpv.lpvYcontainer.back();

	for (size_t j = 0; j < ntheta_; j++)
	{
		Xr += thetas_[j] * params_lpv.lpvXcontainer[j];
		Yr += thetas_[j] * params_lpv.lpvYcontainer[j];
	}

	// Compute Feedback coefficients.
	auto &&Pr = Xr.inverse();  // Cost matrix P.
	auto Kfb = Yr * Pr;         // State feedback coefficients matrix.
	uk = Kfb * x_error;         // Feedback control signal.

	// Saturate the controls.
	// Saturate the controls. Pay attention to max-upper, and min-lower.
	// - max, min are for scaling. upper-lower
	// for upper-lower bounding.

	// uk(0) = std::max(std::min(params_opt.uupper(0), uk(0)), params_opt.ulower(0));
	// uk(1) = std::max(std::min(params_opt.uupper(1), uk(1)), params_opt.ulower(1));

	uk(0) = ns_nmpc_utils::clamp(uk(0), params_opt.ulower(0), params_opt.uupper(0));
	uk(1) = ns_nmpc_utils::clamp(uk(1), params_opt.ulower(1), params_opt.uupper(1));

	ns_sim::simulateNonlinearModel_zoh(model_ptr, uk, params, dt, xk);

	// Saturate xk(7) delta
	// xk(7) = std::max(std::min(params_opt.xupper(7), xk(7)), params_opt.xlower(7));
	xk(7) = ns_nmpc_utils::clamp(xk(7), params_opt.xlower(7), params_opt.xupper(7));
	xk(8) = ns_nmpc_utils::clamp(xk(8), params_opt.xlower(8), params_opt.xupper(8));

	// Store the [vx, steering]  in the trajectory data
	nmpc_data.trajectory_data.setFeedbackControls(uk);  // [vx, steering]_inputs

	return true;
}