/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <scene_module/scene_module_interface.h>

class BehaviorVelocityPlannerManager
{
public:
  void launchSceneModule(
    const std::shared_ptr<SceneModuleManagerInterface> & scene_module_manager_ptr);

  autoware_planning_msgs::PathWithLaneId planPathVelocity(
    const std::shared_ptr<const PlannerData> & planner_data,
    const autoware_planning_msgs::PathWithLaneId & input_path_msg);

  diagnostic_msgs::DiagnosticStatus getStopReasonDiag();

private:
  std::vector<std::shared_ptr<SceneModuleManagerInterface>> scene_manager_ptrs_;
  diagnostic_msgs::DiagnosticStatus stop_reason_diag_;
};
