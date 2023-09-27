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

#include "ndt_scan_matcher/diagnostics_module.hpp"
#include "ndt_scan_matcher/map_update_module.hpp"

void MapUpdateModule::initialize_diagnostics_key_value()
{
  diagnostics_module_->addKeyValue("is_set_map_points", false);
  diagnostics_module_->addKeyValue("is_set_current_position", false);
  diagnostics_module_->addKeyValue("is_set_last_update_position", false);
  diagnostics_module_->addKeyValue("is_running_update_map", false);
  diagnostics_module_->addKeyValue("distance_last_updete_position_to_current_position", 0.0);
  diagnostics_module_->addKeyValue("distance_last_updete_position_to_current_lidar_range", 0.0);
  diagnostics_module_->addKeyValue("latest_update_execution_time", 0.0);
}

bool MapUpdateModule::validate_map_is_in_lidar_range(
  const double distance, const double warn_distance)
{
  diagnostics_module_->addKeyValue(
    "distance_last_updete_position_to_current_lidar_range", distance);

  bool is_ok = (distance < warn_distance);
  if (!is_ok) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 1, "Dynamic map loading is not keeping up.");
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "[ERROR] Dynamic map loading is not keeping up");
  }

  return is_ok;
}

bool MapUpdateModule::validate_is_set_current_position(const bool is_set_current_position)
{
  diagnostics_module_->addKeyValue("is_set_current_position", is_set_current_position);

  bool is_ok = (is_set_current_position);
  if (!is_ok) {
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, *clock_, 1,
      "Cannot find the reference position for map update. Please check if the EKF odometry is "
      "provided to NDT.");
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "[WARN] Cannot find the reference position for map update. Please check if the EKF odometry "
      "is"
      "provided to NDT.");
  }

  return is_ok;
}
