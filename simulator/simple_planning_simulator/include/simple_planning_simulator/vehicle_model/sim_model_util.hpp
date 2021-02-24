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

#ifndef SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_UTIL_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_UTIL_HPP_

#include <math.h>

namespace sim_model_util
{
double getDummySteerCommandWithFriction(
  const double steer, const double steer_command, const double deadzone_delta_steer);
}

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_UTIL_HPP_
