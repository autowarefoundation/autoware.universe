// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include "simple_planning_simulator/vehicle_model/sim_model_util.hpp"

namespace sim_model_util
{
double getDummySteerCommandWithFriction(
  const double steer, const double steer_command, const double deadzone_delta_steer)
{
  const double delta_steer = std::fabs(steer_command - steer);
  // if delta steer is too small, ignore steer command (send current steer as steer command)
  if (delta_steer < deadzone_delta_steer) {
    return steer;
  }
  return steer_command;
}

}  // namespace sim_model_util
