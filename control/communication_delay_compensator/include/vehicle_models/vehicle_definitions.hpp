// Copyright 2022 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COMMUNICATION_DELAY_COMPENSATOR__VEHICLE_DEFINITIONS_HPP
#define COMMUNICATION_DELAY_COMPENSATOR__VEHICLE_DEFINITIONS_HPP

#include "eigen3/Eigen/Core"
#include "utils_delay_observer/delay_compensation_utils.hpp"
#include "visibility_control.hpp"

namespace observers {
/**
 * @brief Kinematic Vehicle Lateral Error Model state and control definitions.
 * */
    enum class KinematicErrorDims : int {
        STATE_DIM = 3,
        INPUT_DIM = 1,
        MEASUREMENT_DIM = 3
    };

    using state_vector_vehicle_t =
            Eigen::Matrix<double, toUType(KinematicErrorDims::STATE_DIM), 1>;


    // Lyapunov matrix dimension definitions.
    constexpr size_t cx_NUMBER_OF_LYAP_MATS = 5;
    enum class StateObserverDims : int {
        STATE_DIM = 4,
        INPUT_DIM = 1,
        MEASUREMENT_DIM = 3,

    };

    using state_vector_observer_t =
            Eigen::Matrix<double, toUType(StateObserverDims::STATE_DIM), 1>;

    using input_vector_observer_t =
            Eigen::Matrix<double, toUType(StateObserverDims::INPUT_DIM), 1>;

    using state_matrix_observer_t = Eigen::Matrix<
            double, toUType(StateObserverDims::STATE_DIM),
            toUType(StateObserverDims::STATE_DIM)>;

    using input_matrix_observer_t = Eigen::Matrix<
            double, toUType(StateObserverDims::INPUT_DIM),
            toUType(StateObserverDims::STATE_DIM)>;

    using measurement_matrix_observer_t = Eigen::Matrix<
            double, toUType(StateObserverDims::MEASUREMENT_DIM),
            toUType(StateObserverDims::STATE_DIM)>;


    // General Template for enum class types.
    template<int Nnum_of_states>
    using state_vector_qfilter = Eigen::Matrix<double, Nnum_of_states, 1>;

    template<typename T>
    using func_type = std::function<T(T)>;

    template<int nx, int ny>
    using mat_type_t = Eigen::Matrix<double, nx, ny>;

}  // namespace observers
#endif  // COMMUNICATION_DELAY_COMPENSATOR__VEHICLE_DEFINITIONS_HPP
