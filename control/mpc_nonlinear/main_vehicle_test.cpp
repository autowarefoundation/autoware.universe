//
// Created by ali on 11/07/22.
//

#include "vehicle_models/kinematic_vehicle_model.hpp"
#include "utils/nmpc_utils.hpp"
#include "utils/nmpc_utils_eigen.hpp"
#include "vehicle_models/kinematic_model_definitions.hpp"
#include "vehicle_models/kinematic_vehicle_model.hpp"
#include "nonlinear_mpc_core/active_model.hpp"
#include <limits>

int main()
{
	auto vehicle_model_ptr = std::make_shared<Model>();
	ns_models::ParamsVehicle params_vehicle;
	vehicle_model_ptr->updateParameters(params_vehicle);

	vehicle_model_ptr->testModel();

	return 0;
}