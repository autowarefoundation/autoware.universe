// Copyright 2022 Autoware Foundation
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

#ifndef AUTOWARE__EKF_LOCALIZER__STATE_TRANSITION_HPP_
#define AUTOWARE__EKF_LOCALIZER__STATE_TRANSITION_HPP_

#include "autoware/ekf_localizer/matrix_types.hpp"

namespace autoware::ekf_localizer
{

double normalize_yaw(const double & yaw);
Vector6d predict_next_state(const Vector6d & X_curr, const double dt);
Matrix6d create_state_transition_matrix(const Vector6d & X_curr, const double dt);
Matrix6d process_noise_covariance(
  const double proc_cov_yaw_d, const double proc_cov_vx_d, const double proc_cov_wz_d);

}  // namespace autoware::ekf_localizer

#endif  // AUTOWARE__EKF_LOCALIZER__STATE_TRANSITION_HPP_
