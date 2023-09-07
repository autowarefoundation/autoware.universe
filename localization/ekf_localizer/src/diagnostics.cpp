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

#include "ekf_localizer/diagnostics.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <string>

void checkProcessActivated(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const bool * const is_activated_ptr)
{
  if (is_activated_ptr == nullptr) {
    return;
  }

  stat.add("is_activated", (*is_activated_ptr));

  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message;
  if (!(*is_activated_ptr)) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag_message = "[WARN]process is not activated";
  }

  stat.summary(diag_level, diag_message);
}

void checkMeasurementUpdated(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & measurement_type,
  const size_t * const no_update_count_ptr, const size_t * const no_update_count_threshold_warn_ptr,
  const size_t * const no_update_count_threshold_error_ptr)
{
  if (
    no_update_count_ptr == nullptr || no_update_count_threshold_warn_ptr == nullptr ||
    no_update_count_threshold_error_ptr == nullptr) {
    return;
  }

  stat.add(measurement_type + "_no_update_count", *no_update_count_ptr);
  stat.add(
    measurement_type + "_no_update_count_threshold_warn", *no_update_count_threshold_warn_ptr);
  stat.add(
    measurement_type + "_no_update_count_threshold_error", *no_update_count_threshold_error_ptr);

  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message;
  if (*no_update_count_ptr >= *no_update_count_threshold_warn_ptr) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag_message = "[WARN]" + measurement_type + "_queue is empty";
  }
  if (*no_update_count_ptr >= *no_update_count_threshold_error_ptr) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "[ERROR]" + measurement_type + "_queue is empty";
  }

  stat.summary(diag_level, diag_message);
}

void checkMeasurementQueueSize(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & measurement_type,
  const size_t * const queue_size_ptr)
{
  if (queue_size_ptr == nullptr) {
    return;
  }

  stat.add(measurement_type + "_queue_size", *queue_size_ptr);

  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message;

  stat.summary(diag_level, diag_message);
}

void checkMeasurementDelayGate(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & measurement_type,
  const bool * const is_passed_delay_gate_ptr, const double * const delay_time_ptr,
  const double * const delay_time_threshold_ptr)
{
  if (
    is_passed_delay_gate_ptr == nullptr || delay_time_ptr == nullptr ||
    delay_time_threshold_ptr == nullptr) {
    return;
  }

  stat.add(measurement_type + "_is_passed_delay_gate", *is_passed_delay_gate_ptr);
  stat.add(measurement_type + "_delay_time", *delay_time_ptr);
  stat.add(measurement_type + "_delay_time_threshold", *delay_time_threshold_ptr);

  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message;
  if (!(*is_passed_delay_gate_ptr)) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag_message = "[WARN]" + measurement_type + " topic is delay";
  }

  stat.summary(diag_level, diag_message);
}

void checkMeasurementMahalanobisGate(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & measurement_type,
  const bool * const is_passed_mahalabobis_gate_ptr, const double * const mahalabobis_distance_ptr,
  const double * const mahalabobis_distance_threshold_ptr)
{
  if (
    is_passed_mahalabobis_gate_ptr == nullptr || mahalabobis_distance_ptr == nullptr ||
    mahalabobis_distance_threshold_ptr == nullptr) {
    return;
  }

  stat.add(measurement_type + "_is_passed_mahalabobis_gate", *is_passed_mahalabobis_gate_ptr);
  stat.add(measurement_type + "_mahalabobis_distance", *mahalabobis_distance_ptr);
  stat.add(
    measurement_type + "_mahalabobis_distance_threshold", *mahalabobis_distance_threshold_ptr);

  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message;
  if (!(*is_passed_mahalabobis_gate_ptr)) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag_message = "[WARN]mahalabobis distance of " + measurement_type + " topic is large";
  }

  stat.summary(diag_level, diag_message);
}
