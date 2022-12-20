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
    if (parameter.get_name() == PreprocessingParameters::FORWARD_LENGTH_PARAM) {
      preprocessing_params_.forward_length = parameter.as_double();
    } else if (parameter.get_name() == PreprocessingParameters::BACKWARD_LENGTH_PARAM) {
      preprocessing_params_.backward_length = parameter.as_double();
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
      uncrossable_lines_ =
        extractUncrossableLines(*lanelet_map_ptr_, expansion_params_.avoid_linestring_types);
    } else if (parameter.get_name() == ExpansionParameters::EGO_EXTRA_OFFSET_FRONT) {
      expansion_params_.ego_extra_front_offset = parameter.as_double();
    } else if (parameter.get_name() == ExpansionParameters::EGO_EXTRA_OFFSET_REAR) {
      expansion_params_.ego_extra_rear_offset = parameter.as_double();
    } else if (parameter.get_name() == ExpansionParameters::EGO_EXTRA_OFFSET_LEFT) {
      expansion_params_.ego_extra_left_offset = parameter.as_double();
    } else if (parameter.get_name() == ExpansionParameters::EGO_EXTRA_OFFSET_RIGHT) {
      expansion_params_.ego_extra_right_offset = parameter.as_double();
    } else if (parameter.get_name() == ExpansionParameters::DYN_OBJECTS_EXTRA_OFFSET_FRONT) {
      expansion_params_.dynamic_objects_extra_front_offset = parameter.as_double();
    } else if (parameter.get_name() == ExpansionParameters::DYN_OBJECTS_EXTRA_OFFSET_REAR) {
      expansion_params_.dynamic_objects_extra_rear_offset = parameter.as_double();
    } else if (parameter.get_name() == ExpansionParameters::DYN_OBJECTS_EXTRA_OFFSET_LEFT) {
      expansion_params_.dynamic_objects_extra_left_offset = parameter.as_double();
    } else if (parameter.get_name() == ExpansionParameters::DYN_OBJECTS_EXTRA_OFFSET_RIGHT) {
      expansion_params_.dynamic_objects_extra_right_offset = parameter.as_double();
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
  const auto start_idx =
    calculateStartIndex(input_path, *ego_idx, preprocessing_params_.backward_length);
  const auto end_idx =
    calculateEndIndex(input_path, *ego_idx, preprocessing_params_.forward_length);
  const auto downsampled_path_points =
    downsamplePath(input_path, start_idx, end_idx, preprocessing_params_.downsample_factor);
  auto path_footprint = createPathFootprint(downsampled_path_points, expansion_params_);
  const auto max_expansion_line =
    createMaxExpansionLine(input_path, expansion_params_.max_expansion_distance);
  auto uncrossable_lines = uncrossable_lines_;
  uncrossable_lines.push_back(max_expansion_line);
  const auto predicted_paths =
    createPredictedPathPolygons(*dynamic_objects_ptr_, expansion_params_);
  const point_t origin = {
    downsampled_path_points.points.front().pose.position.x,
    downsampled_path_points.points.front().pose.position.y};

  const auto diff_ls = expandDrivableArea(
    input_path.left_bound, input_path.right_bound, path_footprint, predicted_paths,
    uncrossable_lines, origin);
  pub_path_->publish(input_path);

  const auto t_end = std::chrono::system_clock::now();
  const auto runtime = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
  // TODO(Maxime): remove before merging
  std::cout << "[RUNTIME] " << runtime.count() << "us" << std::endl;
  pub_runtime_->publish(std_msgs::msg::Int64().set__data(runtime.count()));

  if (pub_debug_markers_->get_subscription_count() > 0) {
    const auto z = msg->points.empty() ? 0.0 : msg->points.front().pose.position.z;
    auto debug_markers = makeDebugMarkers(path_footprint, uncrossable_lines, predicted_paths, z);
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.scale.x = 0.1;
    m.color.a = 1.0;
    m.color.r = 1.0;
    for (const auto & ls : diff_ls) {
      Point point;
      for (const auto & p : ls) {
        point.x = p.x();
        point.y = p.y();
        point.z = msg->points.front().pose.position.z;
        m.points.push_back(point);
      }
      m.ns = "diff" + std::to_string(m.id);
      debug_markers.markers.push_back(m);
      m.id += 1;
      m.color.b += 0.05;
      m.color.g += 0.05;
    }
    pub_debug_markers_->publish(debug_markers);
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
