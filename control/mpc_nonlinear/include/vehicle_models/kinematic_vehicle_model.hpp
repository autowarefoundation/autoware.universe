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

#ifndef VEHICLE_MODELS__KINEMATIC_VEHICLE_MODEL_HPP_
#define VEHICLE_MODELS__KINEMATIC_VEHICLE_MODEL_HPP_

#include "kinematic_model_definitions.hpp"
#include "vehicle_models_base.hpp"

namespace ns_models
{
struct ParamsVehicle
{
	ParamsVehicle() = default;

	// ~ParamsVehicle() = default;
	// Global vehicle parameters
	double wheel_base{2.7};
	double lr{1.4};
	double steering_tau{0.27};   // !<@brief First order steering system model time constant.
	double speed_tau{0.1};      // !<@brief Speed time constant.
	bool use_delay_model{true};  // !<@brief use time constant in the steering and long dynamics.
};

class KinematicModelSingleTrackModel
	: public VehicleModelsBase<KinematicModelSingleTrackModel, STATE_DIM, INPUT_DIM, PARAM_DIM, eSTATE_DIM>
{
 public:
	KinematicModelSingleTrackModel() = default;

	// Destructor
	~KinematicModelSingleTrackModel() override = default;

	// Inherited methods.
	// Dynamical equations overridden from system dynamics.
	void systemEquations(const state_vector_ad_t &x,
											 const input_vector_ad_t &u,
											 const param_vector_ad_t &params,
											 state_vector_ad_t &f_xdot) override;

	// Model methods.
	void updateParameters(ParamsVehicle const &params_vehicle);

	/**
	 * @brief returns rate of change of steering state.
	 * Get steering dynamics delta_dot.
	 * */
	[[nodiscard]] double getSteeringDynamics_deltadot(double const &current_steering,
																										double const &current_steering_input) const;

	/**
	 * @brief returns rate of change of speed Vx state.
	 * Get speed dynamics v_dot.
	 * */
	[[nodiscard]] double getLongSpeedDynamics_vdot(double const &current_long_speed,
																								 double const &current_speed_input) const;

	// Test Vehicle Model
	void testModel();

 private:
	double wheel_base_{2.7};
	double lr_{1.4};
	double steering_tau_{0.3};
	double speed_tau_{0.6};
	bool use_delay_models_{true};

};
}  // namespace ns_models

#endif  // VEHICLE_MODELS__KINEMATIC_VEHICLE_MODEL_HPP_
