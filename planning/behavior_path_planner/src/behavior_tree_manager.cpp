// Copyright 2021 Tier IV, Inc.
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

#include "behavior_path_planner/behavior_tree_manager.hpp"

#include "behavior_path_planner/scene_module/scene_module_bt_node_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_path_planner
{
PathWithLaneId createPath(const std::shared_ptr<RouteHandler> & route_handler)
{
  constexpr double backward_length = 1.0;

  lanelet::ConstLanelet goal_lane;
  if (!route_handler->getGoalLanelet(&goal_lane)) {
    PathWithLaneId path{};
    return path;
  }
  lanelet::ConstLanelets goal_lanes = route_handler->getLaneletSequence(
    goal_lane, route_handler->getGoalPose(), backward_length, 0.0);

  const auto arc_coord =
    lanelet::utils::getArcCoordinates(goal_lanes, route_handler->getGoalPose());
  const double s_start = std::max(arc_coord.length - backward_length, 0.0);
  const double s_end = arc_coord.length;

  return route_handler->getCenterLinePath(goal_lanes, s_start, s_end);
}

BehaviorTreeManager::BehaviorTreeManager(
  rclcpp::Node & node, const BehaviorTreeManagerParam & param)
: bt_manager_param_(param),
  logger_(node.get_logger().get_child("behavior_tree_manager")),
  clock_(*node.get_clock())
{
}

void BehaviorTreeManager::createBehaviorTree()
{
  blackboard_ = BT::Blackboard::create();
  try {
    bt_tree_ = bt_factory_.createTreeFromFile(bt_manager_param_.bt_tree_config_path, blackboard_);
  } catch (...) {
    RCLCPP_ERROR(
      logger_, "Failed to create BT from: %s", bt_manager_param_.bt_tree_config_path.c_str());
    exit(EXIT_FAILURE);
  }
  addGrootMonitoring(
    &bt_tree_, bt_manager_param_.groot_zmq_publisher_port, bt_manager_param_.groot_zmq_server_port);
}

void BehaviorTreeManager::registerSceneModule(const std::shared_ptr<SceneModuleInterface> & module)
{
  const std::string & name = module->name();
  const auto status = std::make_shared<SceneModuleStatus>(name);

  // simple condition node for "isRequested"
  bt_factory_.registerSimpleCondition(name + "_Request", [module, status](BT::TreeNode &) {
    return isExecutionRequested(module, status);
  });

  // simple action node for "planCandidate"
  auto bt_node =
    std::make_shared<SceneModuleBTNodeInterface>("", BT::NodeConfiguration{}, module, status);
  bt_factory_.registerSimpleAction(
    name + "_PlanCandidate",
    [bt_node](BT::TreeNode & tree_node) { return bt_node->planCandidate(tree_node); },
    SceneModuleBTNodeInterface::providedPorts());

  // register builder with default tick functor for "plan"
  auto builder = [module, status](
                   const std::string & _name, const BT::NodeConfiguration & _config) {
    return std::make_unique<SceneModuleBTNodeInterface>(_name, _config, module, status);
  };
  bt_factory_.registerBuilder<SceneModuleBTNodeInterface>(name + "_Plan", builder);

  scene_modules_.push_back(module);
  modules_status_.push_back(status);
}

BehaviorModuleOutput BehaviorTreeManager::run(const std::shared_ptr<PlannerData> & data)
{
  current_planner_data_ = data;

  // set planner_data & reset status
  std::for_each(
    scene_modules_.begin(), scene_modules_.end(), [&data](const auto & m) { m->setData(data); });
  std::for_each(modules_status_.begin(), modules_status_.end(), [](const auto & s) {
    *s = SceneModuleStatus{s->module_name};
  });

  if (isEgoOutOfRoute(data)) {
    BehaviorModuleOutput output{};
    output.path = std::make_shared<PathWithLaneId>(createPath(data->route_handler));
    output.reference_path = std::make_shared<PathWithLaneId>(createPath(data->route_handler));
    return output;
  }

  // reset blackboard
  blackboard_->set<BehaviorModuleOutput>("output", BehaviorModuleOutput{});

  const auto res = bt_tree_.tickRoot();

  const auto output = blackboard_->get<BehaviorModuleOutput>("output");

  RCLCPP_DEBUG(logger_, "BehaviorPathPlanner::run end status = %s", BT::toStr(res).c_str());

  resetNotRunningModulePathCandidate();

  std::for_each(scene_modules_.begin(), scene_modules_.end(), [](const auto & m) {
    m->publishDebugMarker();
    m->publishVirtualWall();
    if (!m->isExecutionRequested()) {
      m->onExit();
    }
    m->publishRTCStatus();
    m->publishSteeringFactor();
  });
  return output;
}

std::vector<std::shared_ptr<behavior_path_planner::SceneModuleStatus>>
BehaviorTreeManager::getModulesStatus()
{
  return modules_status_;
}

void BehaviorTreeManager::resetNotRunningModulePathCandidate()
{
  const bool is_any_module_running = std::any_of(
    scene_modules_.begin(), scene_modules_.end(),
    [](const auto & module) { return module->getCurrentStatus() == BT::NodeStatus::RUNNING; });

  for (auto & module : scene_modules_) {
    if (is_any_module_running && (module->getCurrentStatus() != BT::NodeStatus::RUNNING)) {
      module->resetPathCandidate();
    }
  }
}

void BehaviorTreeManager::resetBehaviorTree()
{
  bt_tree_.haltTree();
}

void BehaviorTreeManager::addGrootMonitoring(
  BT::Tree * tree, uint16_t publisher_port, uint16_t server_port, uint16_t max_msg_per_second)
{
  groot_monitor_ =
    std::make_unique<BT::PublisherZMQ>(*tree, max_msg_per_second, publisher_port, server_port);
}

void BehaviorTreeManager::resetGrootMonitor()
{
  if (groot_monitor_) {
    groot_monitor_.reset();
  }
}

bool BehaviorTreeManager::isEgoOutOfRoute(const std::shared_ptr<PlannerData> & data) const
{
  const auto self_pose = data->self_odometry->pose.pose;
  lanelet::ConstLanelet goal_lane;
  if (!data->route_handler->getGoalLanelet(&goal_lane)) {
    RCLCPP_WARN_STREAM(logger_, "cannot find goal lanelet");
    return true;
  }

  // If ego vehicle is over goal on goal lane, return true
  if (lanelet::utils::isInLanelet(self_pose, goal_lane)) {
    const auto ego_arc_coord = lanelet::utils::getArcCoordinates({goal_lane}, self_pose);
    const auto goal_arc_coord =
      lanelet::utils::getArcCoordinates({goal_lane}, data->route_handler->getGoalPose());
    if (ego_arc_coord.length > goal_arc_coord.length) {
      return true;
    } else {
      return false;
    }
  }

  // If ego vehicle is out of the closest lanelet, return false
  lanelet::ConstLanelet closest_lane;
  if (!data->route_handler->getClosestLaneletWithinRoute(self_pose, &closest_lane)) {
    RCLCPP_WARN_STREAM(logger_, "cannot find closest lanelet");
    return true;
  }
  if (!lanelet::utils::isInLanelet(self_pose, closest_lane)) {
    return true;
  }

  return false;
}

std::shared_ptr<SceneModuleVisitor> BehaviorTreeManager::getAllSceneModuleDebugMsgData()
{
  scene_module_visitor_ptr_ = std::make_shared<SceneModuleVisitor>();
  for (const auto & module : scene_modules_) {
    module->acceptVisitor(scene_module_visitor_ptr_);
  }
  return scene_module_visitor_ptr_;
}
}  // namespace behavior_path_planner
