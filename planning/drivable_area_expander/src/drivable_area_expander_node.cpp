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

#include "drivable_area_expander/drivable_area_expander_node.hpp"

#include "drivable_area_expander/debug.hpp"
#include "drivable_area_expander/drivable_area_expander.hpp"
#include "drivable_area_expander/grid_utils.hpp"
#include "drivable_area_expander/map_utils.hpp"
#include "drivable_area_expander/parameters.hpp"
#include "drivable_area_expander/path_preprocessing.hpp"
#include "drivable_area_expander/types.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <motion_utils/motion_utils.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <chrono>

namespace drivable_area_expander
{
DrivableAreaExpanderNode::DrivableAreaExpanderNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("drivable_area_expander", node_options),
  preprocessing_params_(*this),
  expansion_params_(*this)
{
  sub_path_ = create_subscription<Path>(
    "~/input/path", 1, [this](const Path::ConstSharedPtr msg) { onPath(msg); });
  sub_objects_ = create_subscription<PredictedObjects>(
    "~/input/dynamic_objects", 1,
    [this](const PredictedObjects::ConstSharedPtr msg) { dynamic_objects_ptr_ = msg; });
  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/map", rclcpp::QoS{1}.transient_local(),
    [this](const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg) {
      lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_);
      uncrossable_lines_ =
        extractUncrossableLines(*lanelet_map_ptr_, expansion_params_.avoid_linestring_types);
    });

  pub_path_ = create_publisher<Path>("~/output/path", 1);
  pub_debug_markers_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_markers", 1);
  pub_runtime_ = create_publisher<std_msgs::msg::Int64>("~/output/runtime_microseconds", 1);

  set_param_res_ =
    add_on_set_parameters_callback([this](const auto & params) { return onParameter(params); });
}

rcl_interfaces::msg::SetParametersResult DrivableAreaExpanderNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    // Preprocessing parameters
    if (parameter.get_name() == PreprocessingParameters::MAX_LENGTH_PARAM) {
      preprocessing_params_.max_length = parameter.as_double();
    } else if (parameter.get_name() == PreprocessingParameters::DOWNSAMPLING_PARAM) {
      if (!preprocessing_params_.updateDownsampleFactor(parameter.as_int())) {
        result.successful = false;
        result.reason = "bad downsample factor";
      }
      // Expansion parameters
    } else if (parameter.get_name() == ExpansionParameters::AVOID_DYN_OBJECTS_PARAM) {
      expansion_params_.avoid_dynamic_objects = parameter.as_bool();
    } else if (parameter.get_name() == ExpansionParameters::AVOID_LINESTRING_TYPES_PARAM) {
      expansion_params_.avoid_linestring_types = parameter.as_string_array();
    } else if (parameter.get_name() == ExpansionParameters::EXTRA_FOOTPRINT_OFFSET_PARAM) {
      expansion_params_.extra_footprint_offset = parameter.as_double();
    } else if (parameter.get_name() == ExpansionParameters::MAX_EXP_DIST_PARAM) {
      expansion_params_.max_expansion_distance = parameter.as_double();
    } else {
      RCLCPP_WARN(get_logger(), "Unknown parameter %s", parameter.get_name().c_str());
      result.successful = false;
    }
  }
  return result;
}

void DrivableAreaExpanderNode::onPath(const Path::ConstSharedPtr msg)
{
  const auto t_start = std::chrono::system_clock::now();
  const auto current_pose_ptr = self_pose_listener_.getCurrentPose();
  if (!current_pose_ptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), rcutils_duration_value_t(1000), "Waiting for current pose");
    return;
  }
  const auto ego_idx = motion_utils::findNearestIndex(msg->points, current_pose_ptr->pose);
  if (!validInputs(ego_idx)) return;
  auto input_path = *msg;
  auto grid_map = convertToGridMap(input_path.drivable_area);
  const auto end_idx = calculateEndIndex(input_path, *ego_idx, preprocessing_params_.max_length);
  const auto downsampled_path_points =
    downsamplePath(input_path, *ego_idx, end_idx, preprocessing_params_.downsample_factor);
  const auto path_footprint = createPathFootprint(downsampled_path_points, expansion_params_);
  /*
  // remove footprint beyond uncrossable lines
  expandDrivableArea(grid_map, path_footprint);
  if(expansion_params_.avoid_dynamic_objects)
    maskDynamicObjects(grid_map, *dynamic_objects_ptr_);
  input_path.drivable_area = convertToOccupancyGrid(grid_map);
  pub_path_->publish(input_path);
  */

  const auto t_end = std::chrono::system_clock::now();
  const auto runtime = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
  std::cout << "[RUNTIME] " << runtime.count() << "us" << std::endl;
  pub_runtime_->publish(std_msgs::msg::Int64().set__data(runtime.count()));

  if (pub_debug_markers_->get_subscription_count() > 0) {
    const auto z = msg->points.empty() ? 0.0 : msg->points.front().pose.position.z;
    pub_debug_markers_->publish(makeDebugMarkers(path_footprint, uncrossable_lines_, z));
  }
}

bool DrivableAreaExpanderNode::validInputs(const boost::optional<size_t> & ego_idx)
{
  constexpr auto one_sec = rcutils_duration_value_t(1000);
  const auto missing_dynamic_objects =
    expansion_params_.avoid_dynamic_objects && !dynamic_objects_ptr_;
  if (missing_dynamic_objects)
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), one_sec, "Dynamic obstacles not yet received");
  if (!ego_idx)
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), one_sec, "Cannot calculate ego index on the path");

  return !missing_dynamic_objects && ego_idx;
}
}  // namespace drivable_area_expander

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drivable_area_expander::DrivableAreaExpanderNode)
