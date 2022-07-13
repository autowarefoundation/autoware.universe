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

#ifndef VEHICLE_MODELS__VEHICLE_MODELS_BASE_HPP_
#define VEHICLE_MODELS__VEHICLE_MODELS_BASE_HPP_

#include <eigen3/Eigen/StdVector>
#include <memory>
#include <vector>
#include "Eigen/Dense"
#include "kinematic_model_definitions.hpp"
#include "nonlinear_mpc_core/nmpc_data_discretization.hpp"
#include "nonlinear_mpc_core/nmpc_data_trajectory.hpp"
#include "vehicle_models/vehicle_dynamics_base.hpp"

/**
 * @brief Defines the data structure to be used for the vehicle models.
 * @tparam DERIVED class of vehicle model derived from the base class.
 * @tparam STATE_DIM dimension of vehicle model states.
 * @tparam eSTATE_DIM dimension of error states.
 * @tparam INPUT_DIM dimension of inputs.
 * @tparam PARAM_DIM dimension of parameters such as reference curvature.
 * */
template<typename DERIVED, int STATE_DIM, int INPUT_DIM, int PARAM_DIM, int eSTATE_DIM>
class VehicleModelsBase : public VehicleDynamicsBase<STATE_DIM, INPUT_DIM, PARAM_DIM, eSTATE_DIM>
{
public:
  VehicleModelsBase() = default;

  virtual ~VehicleModelsBase() = default;

  // Type Definitions
  using BASE = VehicleDynamicsBase<STATE_DIM, INPUT_DIM, PARAM_DIM, eSTATE_DIM>;

  // Trajectory and discrete models are stored in the followings;
  using trajectory_data_t = ns_data::TrajectoryData<BASE>;
  using discretization_data_t = ns_data::DiscretizationData<BASE>;

  using model_ptr_t = std::shared_ptr<DERIVED>;

  using typename BASE::control_matrix_t;
  using typename BASE::control_matrix_v_t;
  using typename BASE::dynamic_matrix_t;
  using typename BASE::dynamic_vector_map_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_vector_t;
  using typename BASE::input_vector_v_t;
  using typename BASE::param_vector_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_matrix_v_t;
  using typename BASE::state_vector_t;
  using typename BASE::state_vector_v_t;

  using typename BASE::control_matrix_ad_t;
  using typename BASE::domain_vector_ad_t;
  using typename BASE::dynamic_vector_ad_t;
  using typename BASE::input_vector_ad_t;
  using typename BASE::param_vector_ad_t;
  using typename BASE::state_matrix_ad_t;
  using typename BASE::state_vector_ad_t;

  // Pose  [x, y, psi]
  using typename BASE::pose_t;
};

#endif  // VEHICLE_MODELS__VEHICLE_MODELS_BASE_HPP_
