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

#include "autoware/path_generator/node.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

namespace autoware::path_generator
{
PathGenerator::PathGenerator(const rclcpp::NodeOptions & node_options)
: Node("path_generator", node_options)
{
  param_listener_ =
    std::make_shared<::path_generator::ParamListener>(this->get_node_parameters_interface());

  path_handler_ = std::make_unique<PathHandler>(
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo());

  // publisher
  path_publisher_ = create_publisher<PathWithLaneId>("~/output/path", 1);

  {
    const auto planning_hz = declare_parameter<double>("planning_hz");
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(planning_hz).period(), std::bind(&PathGenerator::run, this));
  }
}

void PathGenerator::takeData()
{
  // route
  if (const auto msg = route_subscriber_.takeData()) {
    if (msg->segments.empty()) {
      RCLCPP_ERROR(get_logger(), "input route is empty, ignoring...");
    } else {
      route_ptr_ = msg;
      if (lanelet_map_bin_ptr_) {
        path_handler_->setRoute(lanelet_map_bin_ptr_, route_ptr_);
      }
    }
  }

  // map
  if (const auto msg = vector_map_subscriber_.takeData()) {
    lanelet_map_bin_ptr_ = msg;
    if (route_ptr_) {
      path_handler_->setRoute(lanelet_map_bin_ptr_, route_ptr_);
    }
  }

  // velocity
  if (const auto msg = odometry_subscriber_.takeData()) {
    self_odometry_ptr_ = msg;
  }
}

// wait until mandatory data is ready
bool PathGenerator::isDataReady()
{
  const auto is_missing = [this](const std::string & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for %s...", name.c_str());
    return false;
  };

  if (!route_ptr_) {
    return is_missing("route");
  }
  if (!lanelet_map_bin_ptr_) {
    return is_missing("map");
  }

  return true;
}

void PathGenerator::run()
{
  takeData();
  if (!isDataReady()) {
    return;
  }

  const auto & current_pose = self_odometry_ptr_->pose.pose;
  const auto param = param_listener_->get_params();

  auto path = path_handler_->generateCenterLinePath(current_pose, param);

  if (!path.points.empty()) {
    const auto current_seg_idx =
      autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        path.points, current_pose, param.ego_nearest_dist_threshold,
        param.ego_nearest_yaw_threshold);
    path.points = autoware::motion_utils::cropPoints(
      path.points, current_pose.position, current_seg_idx, param.forward_path_length,
      param.backward_path_length + param.input_path_interval);

    if (!path.points.empty()) {
      path_publisher_->publish(path);
    } else {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "path output is empty!");
    }
  } else {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "path output is empty!");
  }
}
}  // namespace autoware::path_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_generator::PathGenerator)
