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

#include "control_validator/utils.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace control_validator
{
using tier4_autoware_utils::calcCurvature;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::getPoint;

namespace
{
// TODO(horibe): implement function of calculate max deviation from reference trajectory from the PR
//
}  // namespace

}  // namespace control_validator
