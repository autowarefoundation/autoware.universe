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

#ifndef VEHICLE_MODELS__KINEMATIC_MODEL_DEFINITIONS_HPP_
#define VEHICLE_MODELS__KINEMATIC_MODEL_DEFINITIONS_HPP_

constexpr int STATE_DIM = 8 + 1;  // additional state for the virtual car svirtual.
constexpr int INPUT_DIM = 2;
constexpr int PARAM_DIM = 2;  // [kappa, vx_target]
constexpr int eSTATE_DIM = 4;

/**
 * states = ['xw', 'yw', 'psi', 's', 'e_y', 'e_yaw', 'Vx', 'delta', 'ay']
 * controls = [vx input, steering input]
 * */
enum class VehicleStateIds
{
  X = 0,
  Y = 1,
  yaw = 2,
  s = 3,
  ey = 4,
  eyaw = 5,
  vx = 6,
  steering = 7,
  vy = 8
};

enum class VehicleControlIds
{
  u_vx = 0,
  u_steering = 1
};

enum class VehicleParamIds
{
  curvature = 0,
  target_vx = 1
};

#endif  // VEHICLE_MODELS__KINEMATIC_MODEL_DEFINITIONS_HPP_
