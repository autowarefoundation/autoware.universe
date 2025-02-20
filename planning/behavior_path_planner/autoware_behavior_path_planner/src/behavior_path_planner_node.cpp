// Copyright 2021-2023 Tier IV, Inc.
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

#include "autoware/behavior_path_planner/behavior_path_planner_node.hpp"

#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"

#include <autoware/universe_utils/ros/update_param.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <tier4_planning_msgs/msg/path_change_module_id.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::vehicle_info_utils::VehicleInfoUtils;
using tier4_planning_msgs::msg::PathChangeModuleId;
using DebugStringMsg = autoware_internal_debug_msgs::msg::StringStamped;

BehaviorPathPlannerNode::BehaviorPathPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("behavior_path_planner", node_options),
  planning_factor_interface_{
    std::make_unique<PlanningFactorInterface>(this, "behavior_path_planner")}
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  // data_manager
  {
    planner_data_ = std::make_shared<PlannerData>();
    planner_data_->init_parameters(*this);
  }

  // publisher
  path_publisher_ = create_publisher<PathWithLaneId>("~/output/path", 1);
  turn_signal_publisher_ =
    create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators_cmd", 1);
  hazard_signal_publisher_ = create_publisher<HazardLightsCommand>("~/output/hazard_lights_cmd", 1);
  const auto durable_qos = rclcpp::QoS(1).transient_local();
  modified_goal_publisher_ =
    create_publisher<PoseWithUuidStamped>("~/output/modified_goal", durable_qos);
  reroute_availability_publisher_ =
    create_publisher<RerouteAvailability>("~/output/is_reroute_available", 1);

  debug_avoidance_msg_array_publisher_ =
    create_publisher<AvoidanceDebugMsgArray>("~/debug/avoidance_debug_message_array", 1);

  debug_start_planner_evaluation_table_publisher_ptr_ =
    std::make_unique<DebugPublisher>(this, "~/debug/start_planner_evaluation_table");

  debug_turn_signal_info_publisher_ = create_publisher<MarkerArray>("~/debug/turn_signal_info", 1);

  bound_publisher_ = create_publisher<MarkerArray>("~/debug/bound", 1);

  {
    const std::string path_candidate_name_space = "/planning/path_candidate/";
    const std::string path_reference_name_space = "/planning/path_reference/";

    const std::lock_guard<std::mutex> lock(mutex_manager_);  // for planner_manager_

    const auto slots = declare_parameter<std::vector<std::string>>("slots");
    /* cppcheck-suppress syntaxError */
    std::vector<std::vector<std::string>> slot_configuration{slots.size()};
    for (size_t i = 0; i < slots.size(); ++i) {
      const auto & slot = slots.at(i);
      const auto modules = declare_parameter<std::vector<std::string>>(slot);
      for (const auto & module_name : modules) {
        slot_configuration.at(i).push_back(module_name);
      }
    }

    planner_manager_ = std::make_shared<PlannerManager>(*this);

    for (const auto & name : declare_parameter<std::vector<std::string>>("launch_modules")) {
      // workaround: Since ROS 2 can't get empty list, launcher set [''] on the parameter.
      if (name == "") {
        break;
      }
      planner_manager_->launchScenePlugin(*this, name);
    }

    // NOTE: this needs to be after launchScenePlugin()
    planner_manager_->configureModuleSlot(slot_configuration);

    for (const auto & manager : planner_manager_->getSceneModuleManagers()) {
      path_candidate_publishers_.emplace(
        manager->name(), create_publisher<Path>(path_candidate_name_space + manager->name(), 1));
      path_reference_publishers_.emplace(
        manager->name(), create_publisher<Path>(path_reference_name_space + manager->name(), 1));
    }
  }

  m_set_param_res = this->add_on_set_parameters_callback(
    std::bind(&BehaviorPathPlannerNode::onSetParam, this, std::placeholders::_1));

  // turn signal decider
  {
    const double turn_signal_intersection_search_distance =
      planner_data_->parameters.turn_signal_intersection_search_distance;
    const double turn_signal_intersection_angle_threshold_deg =
      planner_data_->parameters.turn_signal_intersection_angle_threshold_deg;
    const double turn_signal_search_time = planner_data_->parameters.turn_signal_search_time;
    planner_data_->turn_signal_decider.setParameters(
      planner_data_->parameters.base_link2front, turn_signal_intersection_search_distance,
      turn_signal_search_time, turn_signal_intersection_angle_threshold_deg);
  }

  // Start timer
  {
    const auto planning_hz = declare_parameter<double>("planning_hz");
    const auto period_ns = rclcpp::Rate(planning_hz).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&BehaviorPathPlannerNode::run, this));
  }

  logger_configure_ = std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this);
  published_time_publisher_ =
    std::make_unique<autoware::universe_utils::PublishedTimePublisher>(this);
}

std::vector<std::string> BehaviorPathPlannerNode::getWaitingApprovalModules()
{
  auto all_scene_module_ptr = planner_manager_->getSceneModuleStatus();
  std::vector<std::string> waiting_approval_modules;
  for (const auto & module : all_scene_module_ptr) {
    if (module->is_waiting_approval == true) {
      waiting_approval_modules.push_back(module->module_name);
    }
  }
  return waiting_approval_modules;
}

std::vector<std::string> BehaviorPathPlannerNode::getRunningModules()
{
  auto all_scene_module_ptr = planner_manager_->getSceneModuleStatus();
  std::vector<std::string> running_modules;
  for (const auto & module : all_scene_module_ptr) {
    if (module->status == ModuleStatus::RUNNING) {
      running_modules.push_back(module->module_name);
    }
  }
  return running_modules;
}

void BehaviorPathPlannerNode::takeData()
{
  // route
  {
    const auto msg = route_subscriber_.takeData();
    if (msg) {
      if (msg->segments.empty()) {
        RCLCPP_ERROR(get_logger(), "input route is empty. ignored");
      } else {
        route_ptr_ = msg;
        has_received_route_ = true;
      }
    }
  }
  // map
  {
    const auto msg = vector_map_subscriber_.takeData();
    if (msg) {
      map_ptr_ = msg;
      has_received_map_ = true;
    }
  }
  // velocity
  {
    const auto msg = velocity_subscriber_.takeData();
    if (msg) {
      planner_data_->self_odometry = msg;
    }
  }
  // acceleration
  {
    const auto msg = acceleration_subscriber_.takeData();
    if (msg) {
      planner_data_->self_acceleration = msg;
    }
  }
  // scenario
  {
    const auto msg = scenario_subscriber_.takeData();
    if (msg) {
      current_scenario_ = msg;
    }
  }
  // perception
  {
    const auto msg = perception_subscriber_.takeData();
    if (msg) {
      planner_data_->dynamic_object = msg;
    }
  }
  // occupancy_grid
  {
    const auto msg = occupancy_grid_subscriber_.takeData();
    if (msg) {
      planner_data_->occupancy_grid = msg;
    }
  }
  // costmap
  {
    const auto msg = costmap_subscriber_.takeData();
    if (msg) {
      planner_data_->costmap = msg;
    }
  }
  // traffic_signal
  {
    const auto msg = traffic_signals_subscriber_.takeData();
    if (msg) {
      onTrafficSignals(msg);
    }
  }
  // lateral_offset
  {
    const auto msg = lateral_offset_subscriber_.takeData();
    if (msg) {
      onLateralOffset(msg);
    }
  }
  // operation_mode
  {
    const auto msg = operation_mode_subscriber_.takeData();
    if (msg) {
      planner_data_->operation_mode = msg;
    }
  }
  // external_velocity_limiter
  {
    const auto msg = external_limit_max_velocity_subscriber_.takeData();
    if (msg) {
      planner_data_->external_limit_max_velocity = msg;
    }
  }
}

// wait until mandatory data is ready
bool BehaviorPathPlannerNode::isDataReady()
{
  const auto missing = [this](const auto & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", name);
    return false;
  };

  if (!current_scenario_) {
    return missing("scenario_topic");
  }

  {
    if (!route_ptr_) {
      return missing("route");
    }
  }

  {
    if (!map_ptr_) {
      return missing("map");
    }
  }

  if (!planner_data_->dynamic_object) {
    return missing("dynamic_object");
  }

  if (!planner_data_->self_odometry) {
    return missing("self_odometry");
  }

  if (!planner_data_->self_acceleration) {
    return missing("self_acceleration");
  }

  if (!planner_data_->operation_mode) {
    return missing("operation_mode");
  }

  if (!planner_data_->occupancy_grid) {
    return missing("occupancy_grid");
  }

  return true;
}

void BehaviorPathPlannerNode::run()
{
  takeData();

  if (!isDataReady()) {
    return;
  }

  RCLCPP_DEBUG(get_logger(), "----- BehaviorPathPlannerNode start -----");

  // behavior_path_planner runs only in LANE DRIVING scenario.
  if (current_scenario_->current_scenario != Scenario::LANEDRIVING) {
    return;
  }

  // check for map update
  LaneletMapBin::ConstSharedPtr map_ptr{nullptr};
  {
    if (has_received_map_) {
      // Note: duplicating the shared_ptr prevents the data from being deleted by another thread!
      map_ptr = map_ptr_;
      has_received_map_ = false;
    }
  }

  // check for route update
  LaneletRoute::ConstSharedPtr route_ptr{nullptr};
  {
    if (has_received_route_) {
      // Note: duplicating the shared_ptr prevents the data from being deleted by another thread!
      route_ptr = route_ptr_;
      has_received_route_ = false;
    }
  }

  std::unique_lock<std::mutex> lk_pd(mutex_pd_);  // for planner_data_

  // update map
  if (map_ptr) {
    planner_data_->route_handler->setMap(*map_ptr);
  }

  std::unique_lock<std::mutex> lk_manager(mutex_manager_);  // for planner_manager_

  // update route
  const bool is_first_time = !(planner_data_->route_handler->isHandlerReady());
  if (route_ptr) {
    planner_data_->route_handler->setRoute(*route_ptr);
    // uuid is not changed when rerouting with modified goal,
    // in this case do not need to reset modules.
    const bool has_same_route_id =
      planner_data_->prev_route_id && route_ptr->uuid == planner_data_->prev_route_id;
    // Reset behavior tree when new route is received,
    // so that the each modules do not have to care about the "route jump".
    if (!is_first_time && !has_same_route_id) {
      RCLCPP_INFO(get_logger(), "New uuid route is received. Resetting modules.");
      planner_manager_->reset();
      planner_manager_->resetCurrentRouteLanelet(planner_data_);
      planner_data_->prev_modified_goal.reset();
    }
  }
  const auto controlled_by_autoware_autonomously =
    planner_data_->operation_mode->mode == OperationModeState::AUTONOMOUS &&
    planner_data_->operation_mode->is_autoware_control_enabled;
  if (
    !controlled_by_autoware_autonomously &&
    !planner_manager_->hasPossibleRerouteApprovedModules(planner_data_))
    planner_manager_->resetCurrentRouteLanelet(planner_data_);

  // run behavior planner
  const auto output = planner_manager_->run(planner_data_);

  // path handling
  const auto path = getPath(output, planner_data_, planner_manager_);
  // update planner data
  planner_data_->prev_output_path = path;

  // compute turn signal
  computeTurnSignal(planner_data_, *path, output);

  // publish reroute availability
  publish_reroute_availability();

  // publish drivable bounds
  publish_bounds(*path);

  // NOTE: In order to keep backward_path_length at least, resampling interval is added to the
  // backward.
  const auto current_pose = planner_data_->self_odometry->pose.pose;
  if (!path->points.empty()) {
    const size_t current_seg_idx = planner_data_->findEgoSegmentIndex(path->points);
    path->points = autoware::motion_utils::cropPoints(
      path->points, current_pose.position, current_seg_idx,
      planner_data_->parameters.forward_path_length,
      planner_data_->parameters.backward_path_length +
        planner_data_->parameters.input_path_interval);

    if (!path->points.empty()) {
      path_publisher_->publish(*path);
      published_time_publisher_->publish_if_subscribed(path_publisher_, path->header.stamp);
    } else {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "behavior path output is empty! Stop publish.");
    }
  } else {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "behavior path output is empty! Stop publish.");
  }

  publishSceneModuleDebugMsg(planner_manager_->getDebugMsg());
  publishPathCandidate(planner_manager_->getSceneModuleManagers(), planner_data_);
  publishPathReference(planner_manager_->getSceneModuleManagers(), planner_data_);

  // publish modified goal only when it is updated
  if (
    output.modified_goal &&
    /* has changed modified goal */ (
      !planner_data_->prev_modified_goal || autoware::universe_utils::calcDistance2d(
                                              planner_data_->prev_modified_goal->pose.position,
                                              output.modified_goal->pose.position) > 0.01)) {
    PoseWithUuidStamped modified_goal = *(output.modified_goal);
    modified_goal.header.stamp = path->header.stamp;
    planner_data_->prev_modified_goal = modified_goal;
    modified_goal_publisher_->publish(modified_goal);
  }

  planner_data_->prev_route_id = planner_data_->route_handler->getRouteUuid();

  lk_pd.unlock();  // release planner_data_

  planner_manager_->print();
  planner_manager_->publishProcessingTime();
  planner_manager_->publishMarker();
  planner_manager_->publishVirtualWall();
  lk_manager.unlock();  // release planner_manager_

  RCLCPP_DEBUG(get_logger(), "----- behavior path planner end -----\n\n");
}

void BehaviorPathPlannerNode::computeTurnSignal(
  const std::shared_ptr<PlannerData> planner_data, const PathWithLaneId & path,
  const BehaviorModuleOutput & output)
{
  TurnIndicatorsCommand turn_signal;
  TurnSignalDebugData debug_data;
  HazardLightsCommand hazard_signal;
  if (output.turn_signal_info.hazard_signal.command == HazardLightsCommand::ENABLE) {
    turn_signal.command = TurnIndicatorsCommand::DISABLE;
    hazard_signal.command = output.turn_signal_info.hazard_signal.command;
  } else {
    turn_signal = planner_data->getTurnSignal(path, output.turn_signal_info, debug_data);
    hazard_signal.command = HazardLightsCommand::DISABLE;
  }
  turn_signal.stamp = get_clock()->now();
  hazard_signal.stamp = get_clock()->now();
  turn_signal_publisher_->publish(turn_signal);
  hazard_signal_publisher_->publish(hazard_signal);

  publish_turn_signal_debug_data(debug_data);
  publish_steering_factor(planner_data, turn_signal);
}

void BehaviorPathPlannerNode::publish_steering_factor(
  const std::shared_ptr<PlannerData> & planner_data, const TurnIndicatorsCommand & turn_signal)
{
  const auto [intersection_flag, approaching_intersection_flag] =
    planner_data->turn_signal_decider.getIntersectionTurnSignalFlag();
  if (intersection_flag || approaching_intersection_flag) {
    const auto [intersection_pose, intersection_distance] =
      planner_data->turn_signal_decider.getIntersectionPoseAndDistance();

    const uint16_t planning_factor_direction = std::invoke([&turn_signal]() {
      if (turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
        return PlanningFactor::TURN_LEFT;
      }
      return PlanningFactor::TURN_RIGHT;
    });

    planning_factor_interface_->add(
      intersection_distance, intersection_distance, intersection_pose, intersection_pose,
      planning_factor_direction, SafetyFactorArray{});
  }

  planning_factor_interface_->publish();
}

void BehaviorPathPlannerNode::publish_reroute_availability() const
{
  // In the current behavior path planner, we might encounter unexpected behavior when rerouting
  // while modules other than lane following are active. If non-lane-following module except
  // always-executable module is approved and running, rerouting will not be possible.
  RerouteAvailability is_reroute_available;
  is_reroute_available.stamp = this->now();
  if (planner_manager_->hasPossibleRerouteApprovedModules(planner_data_)) {
    is_reroute_available.availability = false;
  } else {
    is_reroute_available.availability = true;
  }

  reroute_availability_publisher_->publish(is_reroute_available);
}

void BehaviorPathPlannerNode::publish_turn_signal_debug_data(const TurnSignalDebugData & debug_data)
{
  MarkerArray marker_array;

  const auto current_time = rclcpp::Time();
  constexpr double scale_x = 1.0;
  constexpr double scale_y = 1.0;
  constexpr double scale_z = 1.0;
  const auto scale = autoware::universe_utils::createMarkerScale(scale_x, scale_y, scale_z);
  const auto desired_section_color =
    autoware::universe_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999);
  const auto required_section_color =
    autoware::universe_utils::createMarkerColor(1.0, 0.0, 1.0, 0.999);

  // intersection turn signal info
  {
    const auto & turn_signal_info = debug_data.intersection_turn_signal_info;

    auto desired_start_marker = autoware::universe_utils::createDefaultMarker(
      "map", current_time, "intersection_turn_signal_desired_start", 0L, Marker::SPHERE, scale,
      desired_section_color);
    auto desired_end_marker = autoware::universe_utils::createDefaultMarker(
      "map", current_time, "intersection_turn_signal_desired_end", 0L, Marker::SPHERE, scale,
      desired_section_color);
    desired_start_marker.pose = turn_signal_info.desired_start_point;
    desired_end_marker.pose = turn_signal_info.desired_end_point;

    auto required_start_marker = autoware::universe_utils::createDefaultMarker(
      "map", current_time, "intersection_turn_signal_required_start", 0L, Marker::SPHERE, scale,
      required_section_color);
    auto required_end_marker = autoware::universe_utils::createDefaultMarker(
      "map", current_time, "intersection_turn_signal_required_end", 0L, Marker::SPHERE, scale,
      required_section_color);
    required_start_marker.pose = turn_signal_info.required_start_point;
    required_end_marker.pose = turn_signal_info.required_end_point;

    marker_array.markers.push_back(desired_start_marker);
    marker_array.markers.push_back(desired_end_marker);
    marker_array.markers.push_back(required_start_marker);
    marker_array.markers.push_back(required_end_marker);
  }

  // behavior turn signal info
  {
    const auto & turn_signal_info = debug_data.behavior_turn_signal_info;

    auto desired_start_marker = autoware::universe_utils::createDefaultMarker(
      "map", current_time, "behavior_turn_signal_desired_start", 0L, Marker::CUBE, scale,
      desired_section_color);
    auto desired_end_marker = autoware::universe_utils::createDefaultMarker(
      "map", current_time, "behavior_turn_signal_desired_end", 0L, Marker::CUBE, scale,
      desired_section_color);
    desired_start_marker.pose = turn_signal_info.desired_start_point;
    desired_end_marker.pose = turn_signal_info.desired_end_point;

    auto required_start_marker = autoware::universe_utils::createDefaultMarker(
      "map", current_time, "behavior_turn_signal_required_start", 0L, Marker::CUBE, scale,
      required_section_color);
    auto required_end_marker = autoware::universe_utils::createDefaultMarker(
      "map", current_time, "behavior_turn_signal_required_end", 0L, Marker::CUBE, scale,
      required_section_color);
    required_start_marker.pose = turn_signal_info.required_start_point;
    required_end_marker.pose = turn_signal_info.required_end_point;

    marker_array.markers.push_back(desired_start_marker);
    marker_array.markers.push_back(desired_end_marker);
    marker_array.markers.push_back(required_start_marker);
    marker_array.markers.push_back(required_end_marker);
  }

  debug_turn_signal_info_publisher_->publish(marker_array);
}

void BehaviorPathPlannerNode::publish_bounds(const PathWithLaneId & path)
{
  constexpr double scale_x = 0.2;
  constexpr double scale_y = 0.2;
  constexpr double scale_z = 0.2;
  constexpr double color_r = 0.0 / 256.0;
  constexpr double color_g = 148.0 / 256.0;
  constexpr double color_b = 205.0 / 256.0;
  constexpr double color_a = 0.999;

  const auto current_time = path.header.stamp;
  auto left_marker = autoware::universe_utils::createDefaultMarker(
    "map", current_time, "left_bound", 0L, Marker::LINE_STRIP,
    autoware::universe_utils::createMarkerScale(scale_x, scale_y, scale_z),
    autoware::universe_utils::createMarkerColor(color_r, color_g, color_b, color_a));
  for (const auto lb : path.left_bound) {
    left_marker.points.push_back(lb);
  }

  auto right_marker = autoware::universe_utils::createDefaultMarker(
    "map", current_time, "right_bound", 0L, Marker::LINE_STRIP,
    autoware::universe_utils::createMarkerScale(scale_x, scale_y, scale_z),
    autoware::universe_utils::createMarkerColor(color_r, color_g, color_b, color_a));
  for (const auto rb : path.right_bound) {
    right_marker.points.push_back(rb);
  }

  MarkerArray msg;
  msg.markers.push_back(left_marker);
  msg.markers.push_back(right_marker);
  bound_publisher_->publish(msg);
}

void BehaviorPathPlannerNode::publishSceneModuleDebugMsg(
  const std::shared_ptr<SceneModuleVisitor> & debug_messages_data_ptr)
{
  const auto avoidance_debug_message = debug_messages_data_ptr->getAvoidanceModuleDebugMsg();
  if (avoidance_debug_message) {
    debug_avoidance_msg_array_publisher_->publish(*avoidance_debug_message);
  }
  const auto start_planner_debug_message = debug_messages_data_ptr->getStartPlannerModuleDebugMsg();
  if (start_planner_debug_message) {
    debug_start_planner_evaluation_table_publisher_ptr_->publish<DebugStringMsg>(
      "start_planner_evaluation_table", *(start_planner_debug_message));
  }
}

void BehaviorPathPlannerNode::publishPathCandidate(
  const std::vector<std::shared_ptr<SceneModuleManagerInterface>> & managers,
  const std::shared_ptr<PlannerData> & planner_data)
{
  for (auto & manager : managers) {
    if (path_candidate_publishers_.count(manager->name()) == 0) {
      continue;
    }

    if (manager->getSceneModuleObservers().empty()) {
      path_candidate_publishers_.at(manager->name())
        ->publish(convertToPath(nullptr, false, planner_data));
      continue;
    }

    for (auto & observer : manager->getSceneModuleObservers()) {
      if (observer.expired()) {
        continue;
      }
      const auto & status = observer.lock()->getCurrentStatus();
      const auto candidate_path = std::invoke([&]() {
        if (status == ModuleStatus::SUCCESS || status == ModuleStatus::FAILURE) {
          // clear candidate path if the module is finished
          return convertToPath(nullptr, false, planner_data);
        }
        return convertToPath(
          observer.lock()->getPathCandidate(), observer.lock()->isExecutionReady(), planner_data);
      });

      path_candidate_publishers_.at(observer.lock()->name())->publish(candidate_path);
    }
  }
}

void BehaviorPathPlannerNode::publishPathReference(
  const std::vector<std::shared_ptr<SceneModuleManagerInterface>> & managers,
  const std::shared_ptr<PlannerData> & planner_data)
{
  for (auto & manager : managers) {
    if (path_reference_publishers_.count(manager->name()) == 0) {
      continue;
    }

    if (manager->getSceneModuleObservers().empty()) {
      path_reference_publishers_.at(manager->name())
        ->publish(convertToPath(nullptr, false, planner_data));
      continue;
    }

    for (auto & observer : manager->getSceneModuleObservers()) {
      if (observer.expired()) {
        continue;
      }
      path_reference_publishers_.at(observer.lock()->name())
        ->publish(convertToPath(observer.lock()->getPathReference(), true, planner_data));
    }
  }
}

Path BehaviorPathPlannerNode::convertToPath(
  const std::shared_ptr<PathWithLaneId> & path_candidate_ptr, const bool is_ready,
  const std::shared_ptr<PlannerData> & planner_data)
{
  Path output;
  output.header = planner_data->route_handler->getRouteHeader();
  output.header.stamp = this->now();

  if (!path_candidate_ptr) {
    return output;
  }

  output =
    autoware::motion_utils::convertToPath<autoware_internal_planning_msgs::msg::PathWithLaneId>(
      *path_candidate_ptr);
  // header is replaced by the input one, so it is substituted again
  output.header = planner_data->route_handler->getRouteHeader();
  output.header.stamp = this->now();

  if (!is_ready) {
    for (auto & point : output.points) {
      point.longitudinal_velocity_mps = 0.0;
    }
  }

  return output;
}

PathWithLaneId::SharedPtr BehaviorPathPlannerNode::getPath(
  const BehaviorModuleOutput & output, const std::shared_ptr<PlannerData> & planner_data,
  const std::shared_ptr<PlannerManager> & planner_manager)
{
  // TODO(Horibe) do some error handling when path is not available.

  auto path = !output.path.points.empty() ? std::make_shared<PathWithLaneId>(output.path)
                                          : planner_data->prev_output_path;
  path->header = planner_data->route_handler->getRouteHeader();
  path->header.stamp = this->now();

  PathWithLaneId connected_path;
  const auto module_status_ptr_vec = planner_manager->getSceneModuleStatus();

  const auto resampled_path = utils::resamplePathWithSpline(
    *path, planner_data->parameters.output_path_interval, keepInputPoints(module_status_ptr_vec));
  return std::make_shared<PathWithLaneId>(resampled_path);
}

// This is a temporary process until motion planning can take the terminal pose into account
bool BehaviorPathPlannerNode::keepInputPoints(
  const std::vector<std::shared_ptr<SceneModuleStatus>> & statuses) const
{
  const std::vector<std::string> target_modules = {"goal_planner", "avoidance"};

  const auto target_status = ModuleStatus::RUNNING;

  for (auto & status : statuses) {
    if (status->is_waiting_approval || status->status == target_status) {
      if (
        std::find(target_modules.begin(), target_modules.end(), status->module_name) !=
        target_modules.end()) {
        return true;
      }
    }
  }
  return false;
}

void BehaviorPathPlannerNode::onTrafficSignals(const TrafficLightGroupArray::ConstSharedPtr msg)
{
  planner_data_->traffic_light_id_map.clear();
  for (const auto & signal : msg->traffic_light_groups) {
    TrafficSignalStamped traffic_signal;
    traffic_signal.stamp = msg->stamp;
    traffic_signal.signal = signal;
    planner_data_->traffic_light_id_map[signal.traffic_light_group_id] = traffic_signal;
  }
}

void BehaviorPathPlannerNode::onLateralOffset(const LateralOffset::ConstSharedPtr msg)
{
  if (!planner_data_->lateral_offset) {
    planner_data_->lateral_offset = msg;
    return;
  }

  const auto & new_offset = msg->lateral_offset;
  const auto & old_offset = planner_data_->lateral_offset->lateral_offset;

  // offset is not changed.
  if (std::abs(old_offset - new_offset) < 1e-4) {
    return;
  }

  planner_data_->lateral_offset = msg;
}

SetParametersResult BehaviorPathPlannerNode::onSetParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware::universe_utils::updateParam;

  rcl_interfaces::msg::SetParametersResult result;

  {
    const std::lock_guard<std::mutex> lock(mutex_manager_);  // for planner_manager_
    planner_manager_->updateModuleParams(parameters);
  }

  result.successful = true;
  result.reason = "success";

  try {
    // Drivable area expansion parameters
    using drivable_area_expansion::DrivableAreaExpansionParameters;
    const std::lock_guard<std::mutex> lock(mutex_pd_);  // for planner_data_
    updateParam(
      parameters, DrivableAreaExpansionParameters::DRIVABLE_AREA_RIGHT_BOUND_OFFSET_PARAM,
      planner_data_->drivable_area_expansion_parameters.drivable_area_right_bound_offset);
    updateParam(
      parameters, DrivableAreaExpansionParameters::DRIVABLE_AREA_LEFT_BOUND_OFFSET_PARAM,
      planner_data_->drivable_area_expansion_parameters.drivable_area_left_bound_offset);
    updateParam(
      parameters, DrivableAreaExpansionParameters::DRIVABLE_AREA_TYPES_TO_SKIP_PARAM,
      planner_data_->drivable_area_expansion_parameters.drivable_area_types_to_skip);
    updateParam(
      parameters, DrivableAreaExpansionParameters::ENABLED_PARAM,
      planner_data_->drivable_area_expansion_parameters.enabled);
    updateParam(
      parameters, DrivableAreaExpansionParameters::AVOID_DYN_OBJECTS_PARAM,
      planner_data_->drivable_area_expansion_parameters.avoid_dynamic_objects);
    updateParam(
      parameters, DrivableAreaExpansionParameters::AVOID_LINESTRING_TYPES_PARAM,
      planner_data_->drivable_area_expansion_parameters.avoid_linestring_types);
    updateParam(
      parameters, DrivableAreaExpansionParameters::AVOID_LINESTRING_DIST_PARAM,
      planner_data_->drivable_area_expansion_parameters.avoid_linestring_dist);
    updateParam(
      parameters, DrivableAreaExpansionParameters::EGO_EXTRA_FRONT_OVERHANG,
      planner_data_->drivable_area_expansion_parameters.extra_front_overhang);
    updateParam(
      parameters, DrivableAreaExpansionParameters::EGO_EXTRA_WHEELBASE,
      planner_data_->drivable_area_expansion_parameters.extra_wheelbase);
    updateParam(
      parameters, DrivableAreaExpansionParameters::EGO_EXTRA_WIDTH,
      planner_data_->drivable_area_expansion_parameters.extra_width);
    updateParam(
      parameters, DrivableAreaExpansionParameters::DYN_OBJECTS_EXTRA_OFFSET_FRONT,
      planner_data_->drivable_area_expansion_parameters.dynamic_objects_extra_front_offset);
    updateParam(
      parameters, DrivableAreaExpansionParameters::DYN_OBJECTS_EXTRA_OFFSET_REAR,
      planner_data_->drivable_area_expansion_parameters.dynamic_objects_extra_rear_offset);
    updateParam(
      parameters, DrivableAreaExpansionParameters::DYN_OBJECTS_EXTRA_OFFSET_LEFT,
      planner_data_->drivable_area_expansion_parameters.dynamic_objects_extra_left_offset);
    updateParam(
      parameters, DrivableAreaExpansionParameters::DYN_OBJECTS_EXTRA_OFFSET_RIGHT,
      planner_data_->drivable_area_expansion_parameters.dynamic_objects_extra_right_offset);
    updateParam(
      parameters, DrivableAreaExpansionParameters::MAX_EXP_DIST_PARAM,
      planner_data_->drivable_area_expansion_parameters.max_expansion_distance);
    updateParam(
      parameters, DrivableAreaExpansionParameters::MAX_PATH_ARC_LENGTH_PARAM,
      planner_data_->drivable_area_expansion_parameters.max_path_arc_length);
    updateParam(
      parameters, DrivableAreaExpansionParameters::RESAMPLE_INTERVAL_PARAM,
      planner_data_->drivable_area_expansion_parameters.resample_interval);
    updateParam(
      parameters, DrivableAreaExpansionParameters::MAX_REUSE_DEVIATION_PARAM,
      planner_data_->drivable_area_expansion_parameters.max_reuse_deviation);
    updateParam(
      parameters, DrivableAreaExpansionParameters::SMOOTHING_CURVATURE_WINDOW_PARAM,
      planner_data_->drivable_area_expansion_parameters.curvature_average_window);
    updateParam(
      parameters, DrivableAreaExpansionParameters::SMOOTHING_MAX_BOUND_RATE_PARAM,
      planner_data_->drivable_area_expansion_parameters.max_bound_rate);
    updateParam(
      parameters, DrivableAreaExpansionParameters::SMOOTHING_ARC_LENGTH_RANGE_PARAM,
      planner_data_->drivable_area_expansion_parameters.arc_length_range);
    updateParam(
      parameters, DrivableAreaExpansionParameters::MIN_BOUND_INTERVAL,
      planner_data_->drivable_area_expansion_parameters.min_bound_interval);
    updateParam(
      parameters, DrivableAreaExpansionParameters::PRINT_RUNTIME_PARAM,
      planner_data_->drivable_area_expansion_parameters.print_runtime);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}
}  // namespace autoware::behavior_path_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::behavior_path_planner::BehaviorPathPlannerNode)
