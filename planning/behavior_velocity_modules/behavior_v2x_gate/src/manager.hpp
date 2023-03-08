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

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include <behavior_velocity_planner/scene_manager_plugin.hpp>

#include <memory>

namespace behavior_v2x_gate
{

class SceneManager : public behavior_velocity_planner::SceneManagerPlugin
{
public:
  using PlannerData = behavior_velocity_planner::PlannerData;
  void init(rclcpp::Node * node);
  void updateSceneModuleInstances(
    const std::shared_ptr<const PlannerData> & data, const PathWithLaneId & path) override;
  void plan(PathWithLaneId * path) override;
  boost::optional<int> getFirstStopPathPointIndex() override;
  const char * getModuleName() override;

private:
  rclcpp::Node * node_;
};

}  // namespace behavior_v2x_gate

#endif  // MANAGER_HPP_
