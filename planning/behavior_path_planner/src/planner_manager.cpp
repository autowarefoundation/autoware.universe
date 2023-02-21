// Copyright 2022 TIER IV, Inc.
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

#include "behavior_path_planner/planner_manager.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

#include <boost/format.hpp>

#include <memory>
#include <string>

namespace behavior_path_planner
{

PlannerManager::PlannerManager(rclcpp::Node & node, const bool verbose)
: logger_(node.get_logger().get_child("planner_manager")),
  clock_(*node.get_clock()),
  verbose_{verbose}
{
  processing_time_.emplace("total_time", 0.0);
}

BehaviorModuleOutput PlannerManager::run(const std::shared_ptr<PlannerData> & data)
{
  resetProcessingTime();
  stop_watch_.tic("total_time");

  if (!root_lanelet_) {
    root_lanelet_ = updateRootLanelet(data);
  }

  std::for_each(scene_manager_ptrs_.begin(), scene_manager_ptrs_.end(), [&data](const auto & m) {
    m->setData(data);
  });

  while (rclcpp::ok()) {
    /**
     * STEP1: get approved modules' output
     */
    const auto approved_path_data = update(data);

    /**
     * STEP2: check modules that need to be launched
     */
    const auto candidate_module_id = getCandidateModuleID(approved_path_data);

    /**
     * STEP3: if there is no module that need to be launched, return approved modules' output
     */
    if (!candidate_module_id) {
      candidate_module_id_ = boost::none;
      processing_time_.at("total_time") = stop_watch_.toc("total_time", true);
      return approved_path_data;
    }

    /**
     * STEP4: if there is module that should be launched, execute the module
     */
    const auto name = candidate_module_id.get().first->getModuleName();
    stop_watch_.tic(name);
    const auto result = run(candidate_module_id.get(), approved_path_data);
    processing_time_.at(name) += stop_watch_.toc(name, true);

    /**
     * STEP5: if the candidate module's modification is NOT approved yet, return the result.
     * NOTE: the result is output of the candidate module, but the output path don't contains path
     * shape modification that needs approval. On the other hand, it could include velocity profile
     * modification.
     */
    if (isWaitingApproval(candidate_module_id.get())) {
      candidate_module_id_ = candidate_module_id.get();
      processing_time_.at("total_time") = stop_watch_.toc("total_time", true);
      return result;
    }

    /**
     * STEP6: if the candidate module is approved, push the module into approved_modules_
     */
    candidate_module_id_ = boost::none;
    addApprovedModule(candidate_module_id.get());
  }

  processing_time_.at("total_time") = stop_watch_.toc("total_time", true);

  return {};
}

boost::optional<ModuleID> PlannerManager::getCandidateModuleID(
  const BehaviorModuleOutput & previous_module_output) const
{
  std::vector<ModuleID> request_modules{};

  const auto block_simultaneous_execution = [this]() {
    for (const auto & m : approved_modules_) {
      const auto & manager = m.first;
      if (!manager->isSimultaneousExecutable()) {
        return true;
      }
    }
    return false;
  }();

  // pickup execution requested modules
  for (const auto & m : scene_manager_ptrs_) {
    stop_watch_.tic(m->getModuleName());

    const auto uuid = generateUUID();  // generate temporary uuid

    // already exist the modules that don't support simultaneous execution. -> DO NOTHING.
    if (block_simultaneous_execution) {
      processing_time_.at(m->getModuleName()) += stop_watch_.toc(m->getModuleName(), true);
      continue;
    }

    // the module doesn't support simultaneous execution. -> DO NOTHING.
    if (!approved_modules_.empty() && !m->isSimultaneousExecutable()) {
      processing_time_.at(m->getModuleName()) += stop_watch_.toc(m->getModuleName(), true);
      continue;
    }

    // check there is a launched module which is waiting approval as candidate.
    if (!candidate_module_id_) {
      // launched module num reach limit. -> CAN'T LAUNCH NEW MODULE. DO NOTHING.
      if (!m->canLaunchNewModule()) {
        processing_time_.at(m->getModuleName()) += stop_watch_.toc(m->getModuleName(), true);
        continue;
      }

      // the module requests it to be launch. -> CAN LAUNCH THE MODULE. PUSH BACK AS REQUEST
      // MODULES.
      if (m->isExecutionRequested(previous_module_output)) {
        request_modules.emplace_back(m, uuid);
      }

      processing_time_.at(m->getModuleName()) += stop_watch_.toc(m->getModuleName(), true);
      continue;
    }

    // candidate module already exist
    const auto & manager = candidate_module_id_.get().first;
    const auto & name = manager->getModuleName();

    // when the approved module throw another approval request, block all. -> CLEAR REQUEST MODULES
    // AND PUSH BACK.
    if (manager->isAlreadyApproved(candidate_module_id_.get().second)) {
      request_modules.clear();
      request_modules.push_back(candidate_module_id_.get());
      processing_time_.at(m->getModuleName()) += stop_watch_.toc(m->getModuleName(), true);
      break;
    }

    // same name module already launched as candidate.
    if (name == m->getModuleName()) {
      // the module launched as candidate and is running now. the module hasn't thrown any approval
      // yet. -> PUSH BACK AS REQUEST MODULES.
      if (isRunning(candidate_module_id_.get())) {
        request_modules.push_back(candidate_module_id_.get());
      } else {
        deleteExpiredModules(
          candidate_module_id_
            .get());  // TODO(Satoshi OTA) this line is no longer needed? think later.
      }

      processing_time_.at(m->getModuleName()) += stop_watch_.toc(m->getModuleName(), true);
      continue;
    }

    // different name module already launched as candidate.
    // launched module num reach limit. -> CAN'T LAUNCH NEW MODULE. DO NOTHING.
    if (!m->canLaunchNewModule()) {
      processing_time_.at(m->getModuleName()) += stop_watch_.toc(m->getModuleName(), true);
      continue;
    }

    // the module requests it to be launch. -> CAN LAUNCH THE MODULE. PUSH BACK AS REQUEST MODULES.
    if (!m->isExecutionRequested(previous_module_output)) {
      processing_time_.at(m->getModuleName()) += stop_watch_.toc(m->getModuleName(), true);
      continue;
    }

    processing_time_.at(m->getModuleName()) += stop_watch_.toc(m->getModuleName(), true);
    request_modules.emplace_back(m, uuid);
  }

  // select one module to run as candidate module.
  const auto high_priority_module = selectHighestPriorityModule(request_modules);

  if (!high_priority_module) {
    return {};
  }

  // post process
  {
    const auto & manager = high_priority_module.get().first;
    const auto & uuid = high_priority_module.get().second;

    // if the selected module is NOT registered in manager, registered the module.
    if (!manager->exist(uuid)) {
      manager->registerNewModule(previous_module_output, uuid);
    }

    // if the current candidate module is NOT selected as high priority module, delete the candidate
    // module from manager.
    for (const auto & m : request_modules) {
      if (m.first->getModuleName() != manager->getModuleName() && m.first->exist(m.second)) {
        deleteExpiredModules(m);
      }
    }
  }

  return high_priority_module;
}

boost::optional<ModuleID> PlannerManager::selectHighestPriorityModule(
  std::vector<ModuleID> & request_modules) const
{
  if (request_modules.empty()) {
    return {};
  }

  // TODO(someone) enhance this priority decision method.
  std::sort(request_modules.begin(), request_modules.end(), [](auto a, auto b) {
    return a.first->getPriority() < b.first->getPriority();
  });

  return request_modules.front();
}

BehaviorModuleOutput PlannerManager::update(const std::shared_ptr<PlannerData> & data)
{
  BehaviorModuleOutput output = getReferencePath(data);  // generate root reference path.

  bool remove_after_module = false;

  for (auto itr = approved_modules_.begin(); itr != approved_modules_.end();) {
    const auto & manager = itr->first;
    const auto & name = manager->getModuleName();

    stop_watch_.tic(name);

    // if one of the approved modules changes to waiting approval, remove all behind modules.
    if (remove_after_module) {
      itr = approved_modules_.erase(itr);
      processing_time_.at(name) += stop_watch_.toc(name, true);
      continue;
    }

    const auto result = run(*itr, output);  // execute approved module planning.

    // check the module is necessary or not.
    if (!isRunning(*itr)) {
      if (itr == approved_modules_.begin()) {
        // update root lanelet when the lane change is done.
        if (name == "lane_change") {
          root_lanelet_ = updateRootLanelet(data);
        }

        deleteExpiredModules(*itr);  // unregister the expired module from manager.
        itr = approved_modules_.erase(itr);
        output = result;
        processing_time_.at(name) += stop_watch_.toc(name, true);
        continue;
      }
    }

    // if one of the approved modules is waiting approval, the module is popped as candidate
    // module again.
    if (isWaitingApproval(*itr)) {
      if (!!candidate_module_id_) {
        deleteExpiredModules(candidate_module_id_.get());
      }
      candidate_module_id_ = *itr;
      itr = approved_modules_.erase(itr);
      remove_after_module = true;
      processing_time_.at(name) += stop_watch_.toc(name, true);
      continue;
    }

    processing_time_.at(name) += stop_watch_.toc(name, true);
    output = result;
    itr++;
  }

  return output;
}

BehaviorModuleOutput PlannerManager::getReferencePath(
  const std::shared_ptr<PlannerData> & data) const
{
  PathWithLaneId reference_path{};

  constexpr double extra_margin = 10.0;

  const auto & route_handler = data->route_handler;
  const auto & pose = data->self_odometry->pose.pose;
  const auto p = data->parameters;

  reference_path.header = route_handler->getRouteHeader();

  const auto backward_length =
    std::max(p.backward_path_length, p.backward_path_length + extra_margin);

  const auto lanelet_sequence = route_handler->getLaneletSequence(
    root_lanelet_.get(), pose, backward_length, std::numeric_limits<double>::max());

  lanelet::ConstLanelet closest_lane{};
  if (!lanelet::utils::query::getClosestLanelet(lanelet_sequence, pose, &closest_lane)) {
    return {};
  }

  const auto current_lanes =
    route_handler->getLaneletSequence(closest_lane, pose, backward_length, p.forward_path_length);

  reference_path = util::getCenterLinePath(
    *route_handler, current_lanes, pose, backward_length, p.forward_path_length, p);

  // clip backward length
  const size_t current_seg_idx = data->findEgoSegmentIndex(reference_path.points);
  util::clipPathLength(
    reference_path, current_seg_idx, p.forward_path_length, p.backward_path_length);
  const auto drivable_lanelets = util::getLaneletsFromPath(reference_path, route_handler);
  const auto drivable_lanes = util::generateDrivableLanes(drivable_lanelets);

  {
    const int num_lane_change =
      std::abs(route_handler->getNumLaneToPreferredLane(current_lanes.back()));

    const double lane_change_buffer = util::calcLaneChangeBuffer(p, num_lane_change);

    reference_path = util::setDecelerationVelocity(
      *route_handler, reference_path, current_lanes, 2.0, lane_change_buffer);
  }

  const auto shorten_lanes = util::cutOverlappedLanes(reference_path, drivable_lanes);

  const auto expanded_lanes = util::expandLanelets(shorten_lanes, 0.0, 0.0);

  util::generateDrivableArea(reference_path, expanded_lanes, p.vehicle_length, data);

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(reference_path);
  output.reference_path = std::make_shared<PathWithLaneId>(reference_path);

  return output;
}

void PlannerManager::resetRootLanelet(const std::shared_ptr<PlannerData> & data)
{
  if (!root_lanelet_) {
    root_lanelet_ = updateRootLanelet(data);
    return;
  }

  const auto root_lanelet = updateRootLanelet(data);

  // check ego is in same lane
  if (root_lanelet_.get().id() == root_lanelet.id()) {
    return;
  }

  const auto route_handler = data->route_handler;
  const auto next_lanelets = route_handler->getRoutingGraphPtr()->following(root_lanelet_.get());

  // check ego is in next lane
  for (const auto & next : next_lanelets) {
    if (next.id() == root_lanelet_.get().id()) {
      return;
    }
  }

  if (!approved_modules_.empty()) {
    return;
  }

  if (!!candidate_module_id_) {
    const auto & manager = candidate_module_id_.get().first;
    const auto & uuid = candidate_module_id_.get().second;
    if (manager->isAlreadyApproved(uuid)) {
      return;
    }
  }

  reset();

  RCLCPP_INFO_EXPRESSION(logger_, verbose_, "change ego's following lane. reset.");
}

void PlannerManager::print() const
{
  if (!verbose_) {
    return;
  }

  std::ostringstream string_stream;
  string_stream << "\n";
  string_stream << "***********************************************************\n";
  string_stream << "                  planner manager status\n";
  string_stream << "-----------------------------------------------------------\n";
  string_stream << "registered modules: ";
  for (const auto & m : scene_manager_ptrs_) {
    string_stream << "[" << m->getModuleName() << "]";
  }

  string_stream << "\n";
  string_stream << "approved modules  : ";
  for (const auto & id : approved_modules_) {
    const auto & manager = id.first;
    string_stream << "[" << manager->getModuleName() << "]->";
  }

  string_stream << "\n";
  string_stream << "candidate module  : ";
  if (!!candidate_module_id_) {
    const auto & manager = candidate_module_id_.get().first;
    string_stream << "[" << manager->getModuleName() << "]";
  }

  string_stream << "\n" << std::fixed << std::setprecision(1);
  string_stream << "processing time   : ";
  for (const auto & t : processing_time_) {
    string_stream << std::right << "[" << std::setw(16) << std::left << t.first << ":"
                  << std::setw(4) << std::right << t.second << "ms]\n"
                  << std::setw(21);
  }

  RCLCPP_INFO_STREAM(logger_, string_stream.str());
}

}  // namespace behavior_path_planner
