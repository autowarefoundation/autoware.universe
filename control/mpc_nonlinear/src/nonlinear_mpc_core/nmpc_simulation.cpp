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

#include "nonlinear_mpc_core/nmpc_simulation.hpp"

namespace ns_sim
{
// an external observer for observing integration steps
// void observer(const Model::state_vector_t & x, const double t)
// {
//  /**
//  * Prints the integrated states in the intervals
//  */
//
//  std::cout << " The values of the states in the integration interval:" << std::endl;
//  ns_eigen_utils::printEigenMat(x);
//
//  for (int i = 0; i < x.size(); i++) {
//    std::cout << t << "    " << x(i);
//  }
//
//  std::cout << "\n";
// }

// Zero Order Hold ODE.
ODEzoh::ODEzoh(Model::model_ptr_t const &model, const Model::input_vector_t &u,
							 Model::param_vector_t const &params, const double dt)
	: model_(model), u_(u), params_(params), dt_(dt)
{
}

// Boost integration library integration function signature. f(x, dxdt, t)
void ODEzoh::operator()(const Model::state_vector_t &x, Model::state_vector_t &dxdt, double t)
{
	// model->Compute_fx(xk, uk, kappa0, &f);
	model_->computeFx(x, u_, params_, dxdt);
}

// First Order Hold ODE.
ODEfoh::ODEfoh(Model::model_ptr_t const &model, const Model::input_vector_t &u0,
							 const Model::input_vector_t &u1, Model::param_vector_t const &params0,
							 Model::param_vector_t const &params1, double dt)
	: model_(model), u0_(u0), u1_(u1), params0_(params0), params1_(params1), dt_(dt)
{
}

void ODEfoh::operator()(const Model::state_vector_t &x, Model::state_vector_t &dxdt, const double t)
{


	// InterpolateInCoordinates the inputs and the parameters.
	Model::input_vector_t u = u0_ + t / dt_ * (u1_ - u0_);
	auto &&params = params0_ + t / dt_ * (params1_ - params0_);

	// Compute f
	model_->computeFx(x, u, params, dxdt);
}

// Variable speed ODE.
ODEvariableSpeed::ODEvariableSpeed(
	const Model::model_ptr_t &model, const Model::input_vector_t &u0,
	Model::param_vector_t const &params0, const double &v0, const double &v1, double dt)
	: model_(model), u0_(u0), params0_(params0), v0_(v0), v1(v1), dt_(dt)
{
	// Acceleration input is cancelled in this integration as we use varying speed.
	u0_(0) = 0.0;
}

void ODEvariableSpeed::operator()(Model::state_vector_t &x, Model::state_vector_t &dxdt, double t)
{
	auto &&v_interpolated = v0_ + t / dt_ * (v1 - v0_);
	x(6) = v_interpolated;

	// Compute f
	model_->computeFx(x, u0_, params0_, dxdt);
}

// Simulator methods.
void simulateNonlinearModel_zoh(Model::model_ptr_t model,
																Model::input_vector_t const &u0,
																Model::param_vector_t const &params,
																const double &dt,
																Model::state_vector_t &x)
{
	/**
	 * class State ,
	 * class Value = double ,
	 * class Deriv = State ,
	 * class Time = Value ,
	 * class Algebra = typename algebra_dispatcher< State >::algebra_type ,
	 * class Operations = typename operations_dispatcher< State >::operations_type ,
	 * class Resizer = initially_resizer
	 **/

	/** template <state, value type, derivative, time value type>
	 *        runge_kutta_fehlberg78<Model::state_vector_t, double, Model::state_vector_t, double,
	 *                vector_space_algebra> stepper;
	 **/
	boost::numeric::odeint::runge_kutta4<Model::state_vector_t, double, Model::state_vector_t, double,
																			 boost::numeric::odeint::vector_space_algebra> stepper;

	ODEzoh ode(model, u0, params, dt);

	// If we want to use the observer to observe internal integrated values, use observer.
	//  integrate_adaptive(stepper, ode, x, 0., dt, dt / 20., observer);

	// Without observers.
	/**
	 * Boost function signature:
	 *  integrate_adaptive(Stepper stepper, System system,
	 *                                     State & start_state, Time start_time,
	 *                                     Time end_time, Time dt, Observer observer);
	 *
	 * */

	//  ns_utils::print("x before integration \n");
	//  ns_eigen_utils::printEigenMat(x);
	//
	//  ns_utils::print("u0 before integration \n");
	//  ns_eigen_utils::printEigenMat(u0);

	double num_of_tsteps = 10;  // Number of time steps for the RK integration. 1 for a single step.
	boost::numeric::odeint::integrate_adaptive(stepper, ode, x, 0., dt, dt / num_of_tsteps);

	//  ns_print::print("x after integration \n");
	//  ns_eigen_utils::printEigenMat(x);
}

void SimulateNonlinearModel_foh(Model::model_ptr_t model,
																Model::input_vector_t const &u0,
																Model::input_vector_t const &u1,
																Model::param_vector_t const &params0,
																Model::param_vector_t const &params1,
																const double &dt,
																Model::state_vector_t &x)
{
	/**
	 * class State ,
	 * class Value = double ,
	 * class Deriv = State ,
	 * class Time = Value ,
	 * class Algebra = typename algebra_dispatcher< State >::algebra_type ,
	 * class Operations = typename operations_dispatcher< State >::operations_type ,
	 * class Resizer = initially_resizer
	 **/

	// state, value type, derivative, time value type.
	//  boost::numeric::odeint::runge_kutta_fehlberg78<
	//    Model::state_vector_t, double, Model::state_vector_t, double,
	//    boost::numeric::odeint::vector_space_algebra>
	//  stepper;

	boost::numeric::odeint::runge_kutta4<Model::state_vector_t, double, Model::state_vector_t, double,
																			 boost::numeric::odeint::vector_space_algebra> stepper;

	ODEfoh ode(model, u0, u1, params0, params1, dt);

	double &&num_of_tsteps = 5;  // Number of time steps for the RK integration. 1 for a single step.
	boost::numeric::odeint::integrate_adaptive(stepper, ode, x, 0., dt, dt / num_of_tsteps);
}

/**
 *  u0  = [0.0, steering input]. The velocity input is set to zero and planned target speeds are used for the vx
 *  states in the equations.
 * */
void simulateNonlinearModel_variableSpeed(Model::model_ptr_t model,
																					Model::input_vector_t const &u0,
																					Model::param_vector_t const &params0,
																					double const &v0,
																					double const &v1,
																					const double &dt,
																					Model::state_vector_t &x)
{
	// state, value type, derivative, time value type.
	boost::numeric::odeint::runge_kutta4<Model::state_vector_t, double, Model::state_vector_t, double,
																			 boost::numeric::odeint::vector_space_algebra> stepper;

	// runge_kutta_fehlberg78<Model::state_vector_t, double,
	// Model::state_vector_t, double, vector_space_algebra> stepper;

	ODEvariableSpeed ode(model, u0, params0, v0, v1, dt);

	double const &num_of_tsteps = 4;  // Number of time steps for the RK integration. 1 for a single step.
	boost::numeric::odeint::integrate_adaptive(stepper, ode, x, 0., dt, dt / num_of_tsteps);
}
}  // namespace ns_sim
