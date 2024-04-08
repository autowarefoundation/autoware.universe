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

#include "ndt_scan_matcher/ndt_scan_matcher_diagnostics_updater_core.hpp"

#include "ndt_scan_matcher/ndt_scan_matcher_core.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <algorithm>
#include <string>

NDTScanMatcherDiagnosticsUpdaterCore::NDTScanMatcherDiagnosticsUpdaterCore(
  NDTScanMatcher * ndt_scan_mathcer_ptr)
{
  // diagnostics_func_is_activated_.reset(new diagnostic_updater::FunctionDiagnosticTask(
  // "check_is_activated", std::bind(
  //                         &NDTScanMatcherDiagnosticsUpdaterCore::check_is_activated, this,
  //                         std::placeholders::_1, &(ndt_scan_mathcer_ptr->is_activated_))));
  diagnostics_func_is_succeed_latest_ndt_aling_service.reset(
    new diagnostic_updater::FunctionDiagnosticTask(
      "check_is_succeed_latest_ndt_aling_service",
      std::bind(
        &NDTScanMatcherDiagnosticsUpdaterCore::check_is_succeed_latest_ndt_aling_service, this,
        std::placeholders::_1, &(ndt_scan_mathcer_ptr->is_succeed_latest_ndt_aling_service_))));
  diagnostics_func_is_running_ndt_aling_service.reset(
    new diagnostic_updater::FunctionDiagnosticTask(
      "check_is_running_ndt_aling_service",
      std::bind(
        &NDTScanMatcherDiagnosticsUpdaterCore::check_is_running_ndt_aling_service, this,
        std::placeholders::_1, &(ndt_scan_mathcer_ptr->is_running_ndt_aling_service_))));
  diagnostics_func_latest_ndt_aling_service_best_score.reset(
    new diagnostic_updater::FunctionDiagnosticTask(
      "check_latest_ndt_aling_service_best_score",
      std::bind(
        &NDTScanMatcherDiagnosticsUpdaterCore::check_latest_ndt_aling_service_best_score, this,
        std::placeholders::_1, &(ndt_scan_mathcer_ptr->latest_ndt_aling_service_best_score_))));

  diagnostics_composite_task_.reset(
    new diagnostic_updater::CompositeDiagnosticTask("ndt_scan_matcher_core"));
  // diagnostics_composite_task_->addTask(diagnostics_func_is_activated_.get());
  diagnostics_composite_task_->addTask(diagnostics_func_is_succeed_latest_ndt_aling_service.get());
  diagnostics_composite_task_->addTask(diagnostics_func_is_running_ndt_aling_service.get());
  diagnostics_composite_task_->addTask(diagnostics_func_latest_ndt_aling_service_best_score.get());

  const double timer_period_sec = 0.1;
  diagnostics_updater_.reset(
    new DiagnosticsUpdaterModule(ndt_scan_mathcer_ptr, timer_period_sec, "localization"));
  diagnostics_updater_->add(*diagnostics_composite_task_);
}

void NDTScanMatcherDiagnosticsUpdaterCore::check_is_activated(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const bool * const is_activated_ptr)
{
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "OK";

  if (is_activated_ptr == nullptr) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "[ERROR] is_activated is nullptr";
    stat.add("is_activated", false);
    stat.summary(diag_level, diag_message);
    return;
  }

  bool is_ok = *is_activated_ptr;
  if (!is_ok) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag_message = "[WARN] Node is not activated";
  }

  stat.add("is_activated", *is_activated_ptr);
  stat.summary(diag_level, diag_message);
}

void NDTScanMatcherDiagnosticsUpdaterCore::check_is_succeed_latest_ndt_aling_service(
  diagnostic_updater::DiagnosticStatusWrapper & stat,
  const bool * const is_succeed_latest_ndt_aling_service_ptr)
{
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "OK";

  if (is_succeed_latest_ndt_aling_service_ptr == nullptr) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "[ERROR] is_succeed_latest_ndt_aling_service_ptr is nullptr";
    stat.add("is_succeed_latest_ndt_aling_service", false);
    stat.summary(diag_level, diag_message);
    return;
  }

  bool is_ok = *is_succeed_latest_ndt_aling_service_ptr;
  if (!is_ok) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag_message = "[WARN] latest ndt_aling_service was faild";
  }

  stat.add("is_succeed_latest_ndt_aling_service", *is_succeed_latest_ndt_aling_service_ptr);
  stat.summary(diag_level, diag_message);
}

void NDTScanMatcherDiagnosticsUpdaterCore::check_is_running_ndt_aling_service(
  diagnostic_updater::DiagnosticStatusWrapper & stat,
  const bool * const is_running_ndt_aling_service_ptr)
{
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "OK";

  if (is_running_ndt_aling_service_ptr == nullptr) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "[ERROR] is_running_ndt_aling_service_ptr is nullptr";
    stat.add("is_running_ndt_aling_service_ptr", false);
    stat.summary(diag_level, diag_message);
    return;
  }

  stat.add("is_running_ndt_aling_service", *is_running_ndt_aling_service_ptr);
  stat.summary(diag_level, diag_message);
}

void NDTScanMatcherDiagnosticsUpdaterCore::check_latest_ndt_aling_service_best_score(
  diagnostic_updater::DiagnosticStatusWrapper & stat,
  const double * const latest_ndt_aling_service_best_score_ptr)
{
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "OK";

  if (latest_ndt_aling_service_best_score_ptr == nullptr) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "[ERROR] latest_ndt_aling_service_best_score_ptr is nullptr";
    stat.add("latest_ndt_aling_service_best_score_ptr", false);
    stat.summary(diag_level, diag_message);
    return;
  }

  stat.add("latest_ndt_aling_service_best_score", *latest_ndt_aling_service_best_score_ptr);
  stat.summary(diag_level, diag_message);
}
