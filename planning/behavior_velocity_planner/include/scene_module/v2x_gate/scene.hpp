// Copyright 2023 The Autoware Contributors
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

#ifndef SCENE_MODULE__V2X_GATE__SCENE_HPP_
#define SCENE_MODULE__V2X_GATE__SCENE_HPP_

#include <scene_module/scene_module_interface.hpp>

namespace behavior_velocity_planner
{

class V2xGateModule : public SceneModuleInterface
{
public:
  struct PlannerParam
  {
    double foo;  // TODO(Takagi, Isamu)
  };

public:
  V2xGateModule();
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__V2X_GATE__SCENE_HPP_
