/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include <string>
#include <unordered_map>

#include <boost/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.h>
#include <scene_module/traffic_light/scene.h>

class TrafficLightModuleManager : public SceneModuleManagerInterface
{
public:
  TrafficLightModuleManager();

  const char * getModuleName() override { return "traffic_light"; }

  virtual void modifyPathVelocity(autoware_planning_msgs::PathWithLaneId * path) override;

private:
  TrafficLightModule::PlannerParam planner_param_;
  std::set<std::shared_ptr<TrafficLightModule>> scene_modules_;

  void launchNewModules(const autoware_planning_msgs::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_planning_msgs::PathWithLaneId & path) override;

  // Debug
  ros::Publisher pub_tl_state_;
};
