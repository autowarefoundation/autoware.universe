// Copyright 2022 Tier IV, Inc.
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

#include "trajectory_follower/pid_longitudinal_controller.hpp"

#include <string>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{

void PidLongitudinalController::setupDiagnosticUpdater()
{
  diagnostic_updater_.setHardwareID("pid_longitudinal_controller");
  diagnostic_updater_.add("control_state", this, &PidLongitudinalController::checkControlState);
}

void PidLongitudinalController::checkControlState(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";

  if (m_control_state == ControlState::EMERGENCY) {
    level = DiagnosticStatus::ERROR;
    msg = "emergency occurred";
  }

  stat.add<int32_t>("control_state", static_cast<int32_t>(m_control_state));
  stat.addf(
    "translation deviation threshold", "%lf",
    m_state_transition_params.emergency_state_traj_trans_dev);
  stat.addf("translation deviation", "%lf", m_control_data.trans_deviation);
  stat.addf(
    "rotation deviation threshold", "%lf", m_state_transition_params.emergency_state_traj_rot_dev);
  stat.addf("rotation deviation", "%lf", m_control_data.rot_deviation);
  stat.summary(level, msg);
}

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware