// Copyright 2024 TIER IV, Inc.
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

#include "node.hpp"

#include "autoware/path_generator/utils.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/route_checker.hpp>

namespace autoware::path_generator
{
PathGenerator::PathGenerator(const rclcpp::NodeOptions & node_options)
: Node("path_generator", node_options)
{
  param_listener_ =
    std::make_shared<::path_generator::ParamListener>(this->get_node_parameters_interface());

  // publisher
  path_publisher_ = create_publisher<PathWithLaneId>("~/output/path", 1);

  {
    const auto planning_hz = declare_parameter<double>("planning_hz");
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(planning_hz).period(), std::bind(&PathGenerator::run, this));
  }
}

void PathGenerator::run()
{
  const auto input_data = takeData();

  const auto path = planPath(input_data);
  if (!path) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is invalid");
  }

  path_publisher_->publish(*path);
}

PathGenerator::InputData PathGenerator::takeData()
{
  InputData input_data;

  // route
  if (const auto msg = route_subscriber_.takeData()) {
    if (msg->segments.empty()) {
      RCLCPP_ERROR(get_logger(), "input route is empty, ignoring...");
    } else {
      input_data.route_ptr = msg;
    }
  }

  // map
  if (const auto msg = vector_map_subscriber_.takeData()) {
    input_data.lanelet_map_bin_ptr = msg;
  }

  // velocity
  if (const auto msg = odometry_subscriber_.takeData()) {
    input_data.odometry_ptr = msg;
  }

  return input_data;
}

std::optional<PathWithLaneId> PathGenerator::planPath(const InputData & input_data)
{
  if (!updatePlannerData(input_data, param_listener_->get_params())) {
    return std::nullopt;
  }

  auto path = utils::generateCenterLinePath(planner_data_);
  if (!path) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is invalid");
    return std::nullopt;
  } else if (path->points.empty()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is empty");
    return std::nullopt;
  }

  return path;
}

bool PathGenerator::updatePlannerData(
  const InputData & input_data, const ::path_generator::Params & param)
{
  if (!planner_data_.lanelet_map_ptr && !input_data.lanelet_map_bin_ptr) {
    return false;
  }

  if (!planner_data_.route_ptr && !input_data.route_ptr) {
    return false;
  }

  if (input_data.lanelet_map_bin_ptr) {
    lanelet::utils::conversion::fromBinMsg(
      *input_data.lanelet_map_bin_ptr, planner_data_.lanelet_map_ptr,
      &planner_data_.traffic_rules_ptr, &planner_data_.routing_graph_ptr);
  }

  if (input_data.route_ptr) {
    if (!lanelet::utils::route::isRouteValid(
          *input_data.route_ptr, planner_data_.lanelet_map_ptr)) {
      return false;
    }
    setRoute(input_data.route_ptr);
  }

  if (input_data.odometry_ptr) {
    planner_data_.current_pose = input_data.odometry_ptr->pose.pose;
  }

  planner_data_.forward_path_length = param.forward_path_length;
  planner_data_.backward_path_length = param.backward_path_length;
  planner_data_.input_path_interval = param.input_path_interval;
  planner_data_.enable_akima_spline_first = param.enable_akima_spline_first;
  planner_data_.ego_nearest_dist_threshold = param.ego_nearest_dist_threshold;
  planner_data_.ego_nearest_yaw_threshold = param.ego_nearest_yaw_threshold;

  return true;
}

void PathGenerator::setRoute(const LaneletRoute::ConstSharedPtr & route_ptr)
{
  planner_data_.route_ptr = route_ptr;

  planner_data_.route_lanelets.clear();
  planner_data_.preferred_lanelets.clear();
  planner_data_.start_lanelets.clear();
  planner_data_.goal_lanelets.clear();

  if (!planner_data_.route_ptr->segments.empty()) {
    size_t primitives_num = 0;
    for (const auto & route_section : planner_data_.route_ptr->segments) {
      primitives_num += route_section.primitives.size();
    }
    planner_data_.route_lanelets.reserve(primitives_num);

    for (const auto & route_section : planner_data_.route_ptr->segments) {
      for (const auto & primitive : route_section.primitives) {
        const auto id = primitive.id;
        const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(id);
        planner_data_.route_lanelets.push_back(lanelet);
        if (id == route_section.preferred_primitive.id) {
          planner_data_.preferred_lanelets.push_back(lanelet);
        }
      }
    }

    const auto set_lanelets_from_segment =
      [&](
        const autoware_planning_msgs::msg::LaneletSegment & segment,
        lanelet::ConstLanelets & lanelets) {
        lanelets.reserve(segment.primitives.size());
        for (const auto & primitive : segment.primitives) {
          const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(primitive.id);
          lanelets.push_back(lanelet);
        }
      };
    set_lanelets_from_segment(
      planner_data_.route_ptr->segments.front(), planner_data_.start_lanelets);
    set_lanelets_from_segment(
      planner_data_.route_ptr->segments.back(), planner_data_.goal_lanelets);
  }
}
}  // namespace autoware::path_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_generator::PathGenerator)
