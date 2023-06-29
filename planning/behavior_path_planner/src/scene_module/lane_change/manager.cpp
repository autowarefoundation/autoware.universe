// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/scene_module/lane_change/manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{

using route_handler::Direction;
using utils::convertToSnakeCase;

namespace
{
template <typename T>
T get_parameter(rclcpp::Node * node, const std::string & ns)
{
  if (node->has_parameter(ns)) {
    return node->get_parameter(ns).get_value<T>();
  }

  return node->declare_parameter<T>(ns);
}
}  // namespace

LaneChangeModuleManager::LaneChangeModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config,
  const Direction direction, const LaneChangeModuleType type)
: SceneModuleManagerInterface(node, name, config, {""}), direction_{direction}, type_{type}
{
  LaneChangeParameters p{};

  const auto parameter = [](std::string && name) { return "lane_change." + name; };

  // trajectory generation
  p.backward_lane_length = get_parameter<double>(node, parameter("backward_lane_length"));
  p.prediction_time_resolution =
    get_parameter<double>(node, parameter("prediction_time_resolution"));
  p.longitudinal_acc_sampling_num =
    get_parameter<int>(node, parameter("longitudinal_acceleration_sampling_num"));
  p.lateral_acc_sampling_num =
    get_parameter<int>(node, parameter("lateral_acceleration_sampling_num"));

  // parked vehicle detection
  p.object_check_min_road_shoulder_width =
    get_parameter<double>(node, parameter("object_check_min_road_shoulder_width"));
  p.object_shiftable_ratio_threshold =
    get_parameter<double>(node, parameter("object_shiftable_ratio_threshold"));

  // turn signal
  p.min_length_for_turn_signal_activation =
    get_parameter<double>(node, parameter("min_length_for_turn_signal_activation"));
  p.length_ratio_for_turn_signal_deactivation =
    get_parameter<double>(node, parameter("length_ratio_for_turn_signal_deactivation"));

  // acceleration
  p.min_longitudinal_acc = get_parameter<double>(node, parameter("min_longitudinal_acc"));
  p.max_longitudinal_acc = get_parameter<double>(node, parameter("max_longitudinal_acc"));

  // collision check
  p.enable_prepare_segment_collision_check =
    get_parameter<bool>(node, parameter("enable_prepare_segment_collision_check"));
  p.prepare_segment_ignore_object_velocity_thresh =
    get_parameter<double>(node, parameter("prepare_segment_ignore_object_velocity_thresh"));
  p.use_predicted_path_outside_lanelet =
    get_parameter<bool>(node, parameter("use_predicted_path_outside_lanelet"));
  p.use_all_predicted_path = get_parameter<bool>(node, parameter("use_all_predicted_path"));

  // target object
  {
    std::string ns = "lane_change.target_object.";
    p.check_car = get_parameter<bool>(node, ns + "car");
    p.check_truck = get_parameter<bool>(node, ns + "truck");
    p.check_bus = get_parameter<bool>(node, ns + "bus");
    p.check_trailer = get_parameter<bool>(node, ns + "trailer");
    p.check_unknown = get_parameter<bool>(node, ns + "unknown");
    p.check_bicycle = get_parameter<bool>(node, ns + "bicycle");
    p.check_motorcycle = get_parameter<bool>(node, ns + "motorcycle");
    p.check_pedestrian = get_parameter<bool>(node, ns + "pedestrian");
  }

  // lane change cancel
  p.cancel.enable_on_prepare_phase =
    get_parameter<bool>(node, parameter("cancel.enable_on_prepare_phase"));
  p.cancel.enable_on_lane_changing_phase =
    get_parameter<bool>(node, parameter("cancel.enable_on_lane_changing_phase"));
  p.cancel.delta_time = get_parameter<double>(node, parameter("cancel.delta_time"));
  p.cancel.duration = get_parameter<double>(node, parameter("cancel.duration"));
  p.cancel.max_lateral_jerk = get_parameter<double>(node, parameter("cancel.max_lateral_jerk"));
  p.cancel.overhang_tolerance = get_parameter<double>(node, parameter("cancel.overhang_tolerance"));

  p.finish_judge_lateral_threshold =
    get_parameter<double>(node, parameter("finish_judge_lateral_threshold"));

  // debug marker
  p.publish_debug_marker = get_parameter<bool>(node, parameter("publish_debug_marker"));

  // validation of parameters
  if (p.longitudinal_acc_sampling_num < 1 || p.lateral_acc_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      logger_, "lane_change_sampling_num must be positive integer. Given longitudinal parameter: "
                 << p.longitudinal_acc_sampling_num
                 << "Given lateral parameter: " << p.lateral_acc_sampling_num << std::endl
                 << "Terminating the program...");
    exit(EXIT_FAILURE);
  }

  if (p.cancel.delta_time < 1.0) {
    RCLCPP_WARN_STREAM(
      logger_, "cancel.delta_time: " << p.cancel.delta_time
                                     << ", is too short. This could cause a danger behavior.");
  }

  parameters_ = std::make_shared<LaneChangeParameters>(p);
}

std::shared_ptr<SceneModuleInterface> LaneChangeModuleManager::createNewSceneModuleInstance()
{
  if (type_ == LaneChangeModuleType::NORMAL) {
    return std::make_shared<LaneChangeInterface>(
      name_, *node_, parameters_, rtc_interface_ptr_map_,
      std::make_unique<NormalLaneChange>(parameters_, LaneChangeModuleType::NORMAL, direction_));
  }
  return std::make_shared<LaneChangeInterface>(
    name_, *node_, parameters_, rtc_interface_ptr_map_,
    std::make_unique<ExternalRequestLaneChange>(parameters_, direction_));
}

void LaneChangeModuleManager::updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto p = parameters_;

  const std::string ns = name_ + ".";
  updateParam<double>(
    parameters, ns + "finish_judge_lateral_threshold", p->finish_judge_lateral_threshold);

  std::for_each(registered_modules_.begin(), registered_modules_.end(), [&p](const auto & m) {
    m->updateModuleParams(p);
  });
}

AvoidanceByLaneChangeModuleManager::AvoidanceByLaneChangeModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config,
  std::shared_ptr<AvoidanceParameters> avoidance_parameters,
  std::shared_ptr<AvoidanceByLCParameters> avoidance_by_lane_change_parameters)
: LaneChangeModuleManager(
    node, name, config, Direction::NONE, LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE),
  avoidance_parameters_(std::move(avoidance_parameters)),
  avoidance_by_lane_change_parameters_(std::move(avoidance_by_lane_change_parameters))
{
  rtc_interface_ptr_map_.clear();
  const std::vector<std::string> rtc_types = {"left", "right"};
  for (const auto & rtc_type : rtc_types) {
    const auto snake_case_name = convertToSnakeCase(name);
    const std::string rtc_interface_name = snake_case_name + "_" + rtc_type;
    rtc_interface_ptr_map_.emplace(
      rtc_type, std::make_shared<RTCInterface>(node, rtc_interface_name));
  }
}

std::shared_ptr<SceneModuleInterface>
AvoidanceByLaneChangeModuleManager::createNewSceneModuleInstance()
{
  return std::make_shared<AvoidanceByLaneChangeInterface>(
    name_, *node_, parameters_, avoidance_parameters_, avoidance_by_lane_change_parameters_,
    rtc_interface_ptr_map_);
}

}  // namespace behavior_path_planner
