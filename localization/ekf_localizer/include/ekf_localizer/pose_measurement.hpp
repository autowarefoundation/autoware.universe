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

#ifndef EKF_LOCALIZER__POSE_MEASUREMENT_HPP_
#define EKF_LOCALIZER__POSE_MEASUREMENT_HPP_

Eigen::Matrix<double, 3, 6> poseMeasurementMatrix();
Eigen::Matrix3d poseMeasurementCovariance(
  const std::array<double, 36ul> & covariance, const double smoothing_step);

#endif  // EKF_LOCALIZER__POSE_MEASUREMENT_HPP_
