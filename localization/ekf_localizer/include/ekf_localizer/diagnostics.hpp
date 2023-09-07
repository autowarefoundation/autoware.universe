// Copyright 2023 Autoware Foundation
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

#ifndef EKF_LOCALIZER__DIAGNOSTICS_HPP_
#define EKF_LOCALIZER__DIAGNOSTICS_HPP_

#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>

void checkProcessActivated(diagnostic_updater::DiagnosticStatusWrapper & stat, const bool* const is_activated_ptr);

void checkMeasurementUpdated(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & measurement_type, const size_t* const no_update_count_ptr, const size_t* const no_update_count_threshold_warn_ptr, const size_t* const no_update_count_threshold_error_ptr);
void checkMeasurementQueueSize(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & measurement_type, const size_t* const queue_size_ptr);
void checkMeasurementDelayGate(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & measurement_type, const bool* const is_passed_delay_gate_ptr, const double* const delay_time_ptr, const double* const delay_time_threshold_ptr);
void checkMeasurementMahalanobisGate(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & measurement_type, const bool* const is_passed_mahalabobis_gate_ptr, const double* const mahalabobis_distance_ptr, const double* const mahalabobis_distance_threshold_ptr);

#endif  // EKF_LOCALIZER__DIAGNOSTICS_HPP_
