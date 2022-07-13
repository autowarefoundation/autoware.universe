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

// enum class Dimensions : int
// {
//    dSTATE_DIM = 8,  // number of states [Xw, Yw, psi, s, e_y, e_yaw, vx, delta]
//    where "e" stands for error.
//    dINPUT_DIM = 2,  // number of inputs [speed input, steering input]
//    dPARAM_DIM = 1,  // number of varying parameters  : if we use it is kappa.
//    deSTATE_DIM = 4  // Error states, [e_y, e_yaw, vx, delta] //
// };

constexpr int STATE_DIM = 8 + 1;  // additional state for the virtual car svirtual.
constexpr int INPUT_DIM = 2;
constexpr int PARAM_DIM = 2;  // [kappa, vx_target]
constexpr int eSTATE_DIM = 4;

#endif  // VEHICLE_MODELS__KINEMATIC_MODEL_DEFINITIONS_HPP_
