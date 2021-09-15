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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "autoware_utils/autoware_utils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "behavior_path_planner/behavior_path_planner_node.hpp"
#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_module.hpp"
#include "behavior_path_planner/scene_module/side_shift/side_shift_module.hpp"

namespace behavior_path_planner
{
using autoware_planning_msgs::msg::PathChangeModuleId;
using vehicle_info_util::VehicleInfoUtil;

BehaviorPathPlannerNode::BehaviorPathPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("behavior_path_planner", node_options)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  // data_manager
  {
    planner_data_ = std::make_shared<PlannerData>();
    planner_data_->parameters = getCommonParam();
  }

  velocity_subscriber_ = create_subscription<TwistStamped>(
    "~/input/velocity", 1, std::bind(&BehaviorPathPlannerNode::onVelocity, this, _1));
  perception_subscriber_ = create_subscription<DynamicObjectArray>(
    "~/input/perception", 1, std::bind(&BehaviorPathPlannerNode::onPerception, this, _1));
  external_approval_subscriber_ = create_subscription<ApprovalMsg>(
    "~/input/external_approval", 1, std::bind(
      &BehaviorPathPlannerNode::onExternalApproval, this,
      _1));
  force_approval_subscriber_ = create_subscription<PathChangeModule>(
    "~/input/force_approval", 1, std::bind(&BehaviorPathPlannerNode::onForceApproval, this, _1));

  // route_handler
  vector_map_subscriber_ = create_subscription<MapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&BehaviorPathPlannerNode::onMap, this, _1));
  route_subscriber_ = create_subscription<Route>(
    "~/input/route", 1, std::bind(&BehaviorPathPlannerNode::onRoute, this, _1));

  // publisher
  path_publisher_ = create_publisher<PathWithLaneId>("~/output/path", 1);
  path_candidate_publisher_ = create_publisher<Path>("~/output/path_candidate", 1);
  turn_signal_publisher_ = create_publisher<TurnSignal>("~/output/turn_signal_cmd", 1);
  debug_drivable_area_publisher_ = create_publisher<OccupancyGrid>("~/debug/drivable_area", 1);
  debug_path_publisher_ = create_publisher<Path>("~/debug/path_for_visualize", 1);

  // For remote operation
  plan_ready_publisher_ = create_publisher<PathChangeModule>("~/output/ready", 1);
  plan_running_publisher_ = create_publisher<PathChangeModuleArray>("~/output/running", 1);
  force_available_publisher_ =
    create_publisher<PathChangeModuleArray>("~/output/force_available", 1);

  // Debug
  debug_marker_publisher_ = create_publisher<MarkerArray>("~/debug/markers", 1);

  // behavior tree manager
  {
    bt_manager_ = std::make_shared<BehaviorTreeManager>(getBehaviorTreeManagerParam());

    auto side_shift_module = std::make_shared<SideShiftModule>(
      "SideShift", *this,
      getSideShiftParam());
    bt_manager_->registerSceneModule(side_shift_module);

    auto avoidance_module = std::make_shared<AvoidanceModule>(
      "Avoidance", *this,
      getAvoidanceParam());
    bt_manager_->registerSceneModule(avoidance_module);

    auto lane_following_module =
      std::make_shared<LaneFollowingModule>("LaneFollowing", *this, getLaneFollowingParam());
    bt_manager_->registerSceneModule(lane_following_module);

    const auto lane_change_param = getLaneChangeParam();

    auto lane_change_module =
      std::make_shared<LaneChangeModule>("LaneChange", *this, lane_change_param);
    bt_manager_->registerSceneModule(lane_change_module);

    auto force_lane_change_module =
      std::make_shared<LaneChangeModule>("ForceLaneChange", *this, lane_change_param);
    bt_manager_->registerSceneModule(force_lane_change_module);

    bt_manager_->registerForceApproval("ForceLaneChange");

    bt_manager_->createBehaviorTree();
  }

  // turn signal decider
  {
    double intersection_search_distance{declare_parameter("intersection_search_distance", 30.0)};
    turn_signal_decider_.setParameters(
      planner_data_->parameters.base_link2front, intersection_search_distance);
  }

  waitForData();

  // Start timer. This must be done after all data (e.g. vehicle pose, velocity) are ready.
  {
    auto on_timer = std::bind(&BehaviorPathPlannerNode::run, this);
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer)>>(
      this->get_clock(), 100ms, std::move(on_timer),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, nullptr);
  }
}

BehaviorPathPlannerParameters BehaviorPathPlannerNode::getCommonParam()
{
  // ROS parameters
  BehaviorPathPlannerParameters p{};
  p.backward_path_length = declare_parameter("backward_path_length", 5.0);
  p.forward_path_length = declare_parameter("forward_path_length", 100.0);
  p.backward_length_buffer_for_end_of_lane = declare_parameter(
    "backward_length_buffer_for_end_of_lane", 5.0);
  p.minimum_lane_change_length = declare_parameter("minimum_lane_change_length", 8.0);
  p.drivable_area_resolution = declare_parameter("drivable_area_resolution", 0.1);
  p.drivable_area_width = declare_parameter("drivable_area_width", 100.0);
  p.drivable_area_height = declare_parameter("drivable_area_height", 50.0);
  p.refine_goal_search_radius_range = declare_parameter("refine_goal_search_radius_range", 7.5);

  // vehicle info
  const auto vehicle_info = VehicleInfoUtil(*this).getVehicleInfo();
  p.vehicle_width = vehicle_info.vehicle_width_m;
  p.vehicle_length = vehicle_info.vehicle_length_m;
  p.wheel_base = vehicle_info.wheel_base_m;
  p.front_overhang = vehicle_info.front_overhang_m;
  p.rear_overhang = vehicle_info.rear_overhang_m;
  p.base_link2front = vehicle_info.max_longitudinal_offset_m;
  p.base_link2rear = p.rear_overhang;

  return p;
}

SideShiftParameters BehaviorPathPlannerNode::getSideShiftParam()
{
  const auto dp = [this](const std::string & str, auto def_val) {
      std::string name = "side_shift." + str;
      return this->declare_parameter(name, def_val);
    };

  SideShiftParameters p{};
  p.min_distance_to_start_shifting = dp("min_distance_to_start_shifting", 5.0);
  p.time_to_start_shifting = dp("time_to_start_shifting", 1.0);
  p.shifting_lateral_jerk = dp("shifting_lateral_jerk", 0.2);
  p.min_shifting_distance = dp("min_shifting_distance", 5.0);
  p.min_shifting_speed = dp("min_shifting_speed", 5.56);

  return p;
}

AvoidanceParameters BehaviorPathPlannerNode::getAvoidanceParam()
{
  const auto dp = [this](const std::string & str, auto def_val) {
      std::string name = "avoidance." + str;
      return this->declare_parameter(name, def_val);
    };

  AvoidanceParameters p{};
  p.threshold_distance_object_is_on_center = dp("threshold_distance_object_is_on_center", 1.0);
  p.threshold_speed_object_is_stopped = dp("threshold_speed_object_is_stopped", 1.0);
  p.object_check_forward_distance = dp("object_check_forward_distance", 100.0);
  p.object_check_backward_distance = dp("object_check_backward_distance", 2.0);
  p.lateral_collision_margin = dp("lateral_collision_margin", 2.0);
  p.time_to_start_avoidance = dp("time_to_start_avoidance", 3.0);
  p.min_distance_to_start_avoidance = dp("min_distance_to_start_avoidance", 10.0);
  p.time_avoiding = dp("time_avoiding", 3.0);
  p.min_distance_avoiding = dp("min_distance_avoiding", 10.0);
  p.max_shift_length = dp("max_shift_length", 1.5);
  p.min_distance_avoidance_end_to_object = dp("min_distance_avoidance_end_to_object", 5.0);
  p.time_avoidance_end_to_object = dp("time_avoidance_end_to_object", 1.0);

  p.nominal_lateral_jerk = dp("nominal_lateral_jerk", 0.3);
  p.max_lateral_jerk = dp("max_lateral_jerk", 2.0);

  return p;
}

LaneFollowingParameters BehaviorPathPlannerNode::getLaneFollowingParam()
{
  LaneFollowingParameters p{};
  p.expand_drivable_area = declare_parameter("lane_following.expand_drivable_area", false);
  p.right_bound_offset = declare_parameter("lane_following.right_bound_offset", 0.5);
  p.left_bound_offset = declare_parameter("lane_following.left_bound_offset", 0.5);
  p.lane_change_prepare_duration = declare_parameter(
    "lane_following.lane_change_prepare_duration", 2.0);
  return p;
}

LaneChangeParameters BehaviorPathPlannerNode::getLaneChangeParam()
{
  const auto dp = [this](const std::string & str, auto def_val) {
      std::string name = "lane_change." + str;
      return this->declare_parameter(name, def_val);
    };

  LaneChangeParameters p{};
  p.min_stop_distance = dp("min_stop_distance", 5.0);
  p.stop_time = dp("stop_time", 2.0);
  p.hysteresis_buffer_distance = dp("hysteresis_buffer_distance", 2.0);
  p.lane_change_prepare_duration = dp("lane_change_prepare_duration", 2.0);
  p.lane_changing_duration = dp("lane_changing_duration", 4.0);
  p.lane_change_finish_judge_buffer = dp("lane_change_finish_judge_buffer", 3.0);
  p.minimum_lane_change_velocity = dp("minimum_lane_change_velocity", 8.3);
  p.prediction_duration = dp("prediction_duration", 8.0);
  p.prediction_time_resolution = dp("prediction_time_resolution", 0.5);
  p.static_obstacle_velocity_thresh = dp("static_obstacle_velocity_thresh", 0.1);
  p.maximum_deceleration = dp("maximum_deceleration", 1.0);
  p.lane_change_sampling_num = dp("lane_change_sampling_num", 10);
  p.enable_abort_lane_change = dp("enable_abort_lane_change", true);
  p.enable_collision_check_at_prepare_phase = dp("enable_collision_check_at_prepare_phase", true);
  p.use_predicted_path_outside_lanelet = dp("use_predicted_path_outside_lanelet", true);
  p.use_all_predicted_path = dp("use_all_predicted_path", false);
  p.abort_lane_change_velocity_thresh = dp("abort_lane_change_velocity_thresh", 0.5);
  p.abort_lane_change_angle_thresh =
    dp("abort_lane_change_angle_thresh", autoware_utils::deg2rad(10.0));
  p.abort_lane_change_distance_thresh = dp("abort_lane_change_distance_thresh", 0.3);
  p.enable_blocked_by_obstacle = dp("enable_blocked_by_obstacle", false);
  p.lane_change_search_distance = dp("lane_change_search_distance", 30.0);

  // validation of parameters
  if (p.lane_change_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      get_logger(),
      "lane_change_sampling_num must be positive integer. Given parameter: " <<
        p.lane_change_sampling_num << std::endl <<
        "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  if (p.maximum_deceleration < 0.0) {
    RCLCPP_FATAL_STREAM(
      get_logger(),
      "maximum_deceleration cannot be negative value. Given parameter: " <<
        p.maximum_deceleration << std::endl <<
        "Terminating the program...");
    exit(EXIT_FAILURE);
  }

  return p;
}

BehaviorTreeManagerParam BehaviorPathPlannerNode::getBehaviorTreeManagerParam()
{
  BehaviorTreeManagerParam p{};
  p.bt_tree_config_path = declare_parameter("bt_tree_config_path", "default");
  p.groot_zmq_publisher_port = declare_parameter("groot_zmq_publisher_port", 1666);
  p.groot_zmq_server_port = declare_parameter("groot_zmq_server_port", 1667);
  return p;
}

void BehaviorPathPlannerNode::waitForData()
{
  // wait until mandatory data is ready
  while (!planner_data_->route_handler->isHandlerReady() && rclcpp::ok()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "waiting for route to be ready");
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::Rate(100).sleep();
  }

  while (rclcpp::ok()) {
    if (planner_data_->dynamic_object && planner_data_->self_velocity) {
      break;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "waiting for vehicle pose, vehicle_velocity, and obstacles");
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::Rate(100).sleep();
  }

  self_pose_listener_.waitForFirstPose();
  planner_data_->self_pose = self_pose_listener_.getCurrentPose();
}

void BehaviorPathPlannerNode::run()
{
  RCLCPP_DEBUG(get_logger(), "----- BehaviorPathPlannerNode start -----");

  // update planner data
  updateCurrentPose();

  // run behavior planner
  const auto output = bt_manager_->run(planner_data_);

  // path handling
  const auto path = getPath(output);
  const auto path_candidate = getPathCandidate(output);
  planner_data_->prev_output_path = path;

  auto clipped_path = clipPathByGoal(*path);
  clipPathLength(clipped_path);

  // TODO(Horibe) the path must have points. Needs to be fix.
  if (!clipped_path.points.empty()) {
    path_publisher_->publish(clipped_path);
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "behavior path output is empty! Stop publish. path = %lu, clipped = %lu", path->points.size(),
      clipped_path.points.size());
  }
  path_candidate_publisher_->publish(util::toPath(*path_candidate));
  // debug_path_publisher_->publish(util::toPath(path));
  debug_drivable_area_publisher_->publish(path->drivable_area);

  // for turn signal
  const auto turn_signal = turn_signal_decider_.getTurnSignal(
    *path, planner_data_->self_pose->pose, *(planner_data_->route_handler),
    output.turn_signal_info.turn_signal, output.turn_signal_info.signal_distance);
  turn_signal_publisher_->publish(turn_signal);

  // for remote operation
  publishModuleStatus(bt_manager_->getModulesStatus());

  publishDebugMarker(bt_manager_->getDebugMarkers());

  RCLCPP_DEBUG(get_logger(), "----- behavior path planner end -----\n\n");
}

PathWithLaneId::SharedPtr BehaviorPathPlannerNode::getPath(
  const BehaviorModuleOutput & bt_output)
{
  // TODO(Horibe) do some error handling when path is not available.

  auto path = bt_output.path ? bt_output.path : planner_data_->prev_output_path;
  path->header = planner_data_->route_handler->getRouteHeader();
  path->header.stamp = this->now();
  RCLCPP_DEBUG(
    get_logger(), "BehaviorTreeManager: output is %s.", bt_output.path ? "FOUND" : "NOT FOUND");
  return path;
}

PathWithLaneId::SharedPtr BehaviorPathPlannerNode::getPathCandidate(
  const BehaviorModuleOutput & bt_output)
{
  auto path_candidate = bt_output.path_candidate ?
    bt_output.path_candidate :
    std::make_shared<PathWithLaneId>();
  path_candidate->header = planner_data_->route_handler->getRouteHeader();
  path_candidate->header.stamp = this->now();
  RCLCPP_DEBUG(
    get_logger(),
    "BehaviorTreeManager: path candidate is %s.", bt_output.path_candidate ? "FOUND" : "NOT FOUND");
  return path_candidate;
}

void BehaviorPathPlannerNode::publishModuleStatus(
  const std::vector<std::shared_ptr<SceneModuleStatus>> & statuses)
{
  auto getModuleType = [](std::string name) {
      if (name == "LaneChange") {
        return PathChangeModuleId::LANE_CHANGE;
      } else if (name == "Avoidance") {
        return PathChangeModuleId::AVOIDANCE;
      } else if (name == "ForceLaneChange") {
        return PathChangeModuleId::FORCE_LANE_CHANGE;
      } else {
        return PathChangeModuleId::NONE;
      }
    };

  const auto now = this->now();

  PathChangeModule ready_module{};
  PathChangeModuleArray running_modules{};
  PathChangeModuleArray force_available{};

  bool is_ready{false};
  for (auto & status : statuses) {
    if (status->status == BT::NodeStatus::RUNNING) {
      PathChangeModuleId module{};
      module.type = getModuleType(status->module_name);
      running_modules.modules.push_back(module);
    }
    if (status->module_name == "LaneChange") {
      const auto force_approval = planner_data_->approval.is_force_approved;
      if (
        force_approval.module_name == "ForceLaneChange" &&
        (now - force_approval.stamp).seconds() < 0.5)
      {
        is_ready = true;
        ready_module.module.type = getModuleType("ForceLaneChange");
      }
      if (status->is_requested && !status->is_ready) {
        PathChangeModuleId module;
        module.type = getModuleType("ForceLaneChange");
        force_available.modules.push_back(module);
        break;
      }
    }
    if (status->is_ready && status->is_waiting_approval) {
      if (status->module_name == "LaneFollowing" || status->module_name == "SideShift") {continue;}
      is_ready = true;
      RCLCPP_DEBUG(
        get_logger(),
        "%s is Ready : ready = %s, is_approved = %s", status->module_name.c_str(),
        status->is_ready ? "true" : "false", status->is_waiting_approval ? "true" : "false");
      ready_module.module.type = getModuleType(status->module_name);
    }
  }

  if (!is_ready) {
    prev_ready_module_name_ = "NONE";
    ready_module.module.type = PathChangeModuleId::NONE;
  }

  ready_module.header.stamp = now;
  plan_ready_publisher_->publish(ready_module);

  running_modules.header.stamp = now;
  plan_running_publisher_->publish(running_modules);

  force_available.header.stamp = now;
  force_available_publisher_->publish(force_available);
}

void BehaviorPathPlannerNode::publishDebugMarker(const std::vector<MarkerArray> & debug_markers)
{
  MarkerArray msg{};
  for (const auto & markers : debug_markers) {
    autoware_utils::appendMarkerArray(markers, &msg);
  }
  debug_marker_publisher_->publish(msg);
}

void BehaviorPathPlannerNode::updateCurrentPose()
{
  auto self_pose = self_pose_listener_.getCurrentPose();
  planner_data_->self_pose = self_pose;
}

void BehaviorPathPlannerNode::onVelocity(const TwistStamped::ConstSharedPtr msg)
{
  planner_data_->self_velocity = msg;
}
void BehaviorPathPlannerNode::onPerception(const DynamicObjectArray::ConstSharedPtr msg)
{
  planner_data_->dynamic_object = msg;
}
void BehaviorPathPlannerNode::onExternalApproval(const ApprovalMsg::ConstSharedPtr msg)
{
  planner_data_->approval.is_approved.data = msg->approval;
  // TODO(wep21): Replace msg stamp after {stamp: now} is implemented in ros2 topic pub
  planner_data_->approval.is_approved.stamp = this->now();
}
void BehaviorPathPlannerNode::onForceApproval(const PathChangeModule::ConstSharedPtr msg)
{
  auto getModuleName = [](PathChangeModuleId module) {
      if (module.type == PathChangeModuleId::FORCE_LANE_CHANGE) {
        return "ForceLaneChange";
      } else {
        return "NONE";
      }
    };
  planner_data_->approval.is_force_approved.module_name = getModuleName(msg->module);
  planner_data_->approval.is_force_approved.stamp = msg->header.stamp;
}
void BehaviorPathPlannerNode::onMap(const MapBin::ConstSharedPtr msg)
{
  planner_data_->route_handler->setMap(*msg);
}
void BehaviorPathPlannerNode::onRoute(const Route::ConstSharedPtr msg)
{
  const bool is_first_time = !(planner_data_->route_handler->isHandlerReady());

  planner_data_->route_handler->setRoute(*msg);

  // Reset behavior tree when new route is received,
  // so that the each modules do not have to care about the "route jump".
  if (!is_first_time) {
    RCLCPP_DEBUG(get_logger(), "new route is received. reset behavior tree.");
    bt_manager_->resetBehaviorTree();
  }
}

void BehaviorPathPlannerNode::clipPathLength(PathWithLaneId & path) const
{
  const auto ego_pos = planner_data_->self_pose->pose.position;
  const double forward = planner_data_->parameters.forward_path_length;
  const double backward = planner_data_->parameters.backward_path_length;

  const auto start_idx = util::getIdxByArclength(path, ego_pos, -backward);
  const auto end_idx = util::getIdxByArclength(path, ego_pos, forward);

  const std::vector<PathPointWithLaneId> clipped_points{path.points.begin() + start_idx,
    path.points.begin() + end_idx};

  path.points = clipped_points;
}

PathWithLaneId BehaviorPathPlannerNode::clipPathByGoal(const PathWithLaneId & path) const
{
  const auto goal = planner_data_->route_handler->getGoalPose();
  const auto goal_lane_id = planner_data_->route_handler->getGoalLaneId();

  Pose refined_goal{};
  {
    lanelet::ConstLanelet goal_lanelet;
    if (planner_data_->route_handler->getGoalLanelet(&goal_lanelet)) {
      refined_goal = util::refineGoal(goal, goal_lanelet);
    } else {
      refined_goal = goal;
    }
  }

  auto refined_path = util::refinePathForGoal(
    planner_data_->parameters.refine_goal_search_radius_range, M_PI * 0.5, path, refined_goal,
    goal_lane_id);
  refined_path.header.frame_id = "map";
  refined_path.header.stamp = this->now();

  return refined_path;
}

}  // namespace behavior_path_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(behavior_path_planner::BehaviorPathPlannerNode)
