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

#ifndef NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_DIAGNOSTICS_UPDATER_CORE_HPP_
#define NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_DIAGNOSTICS_UPDATER_CORE_HPP_

#include "ndt_scan_matcher/diagnostics_updater_module.hpp"

#include <string>

class NDTScanMatcher;

class NDTScanMatcherDiagnosticsUpdaterCore
{
public:
  NDTScanMatcherDiagnosticsUpdaterCore(NDTScanMatcher * ndt_scan_mathcer_ptr);

private:
  void check_is_activated(
    diagnostic_updater::DiagnosticStatusWrapper & stat, const bool * const is_activated_ptr);
  void check_is_succeed_latest_ndt_aling_service(
    diagnostic_updater::DiagnosticStatusWrapper & stat,
    const bool * const is_succeed_latest_ndt_aling_service_ptr);
  void check_is_running_ndt_aling_service(
    diagnostic_updater::DiagnosticStatusWrapper & stat,
    const bool * const is_running_ndt_aling_service_ptr);

  std::unique_ptr<DiagnosticsUpdaterModule> diagnostics_updater_;
  std::shared_ptr<diagnostic_updater::CompositeDiagnosticTask> diagnostics_composite_task_;
  std::shared_ptr<diagnostic_updater::FunctionDiagnosticTask> diagnostics_func_is_activated_;
  std::shared_ptr<diagnostic_updater::FunctionDiagnosticTask>
    diagnostics_func_is_succeed_latest_ndt_aling_service;
  std::shared_ptr<diagnostic_updater::FunctionDiagnosticTask>
    diagnostics_func_is_running_ndt_aling_service;
};
#endif  // NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_DIAGNOSTICS_UPDATER_CORE_HPP_
