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

#ifndef DRIVABLE_AREA_EXPANDER__DRIVABLE_AREA_EXPANDER_NODE_HPP_
#define DRIVABLE_AREA_EXPANDER__DRIVABLE_AREA_EXPANDER_NODE_HPP_

#include "drivable_area_expander/obstacles.hpp"
#include "drivable_area_expander/parameters.hpp"
#include "drivable_area_expander/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/self_pose_listener.hpp>
#include <tier4_autoware_utils/ros/transform_listener.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <std_msgs/msg/int64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace drivable_area_expander
{

class DrivableAreaExpanderNode : public rclcpp::Node
{
public:
  explicit DrivableAreaExpanderNode(const rclcpp::NodeOptions & node_options);

private:
  tier4_autoware_utils::TransformListener transform_listener_{this};
  tier4_autoware_utils::SelfPoseListener self_pose_listener_{this};
  rclcpp::Publisher<Path>::SharedPtr pub_path_;  //!< @brief publisher for output path
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pub_debug_markers_;  //!< @brief publisher for debug markers
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr
    pub_runtime_;                                   //!< @brief publisher for callback runtime
  rclcpp::Subscription<Path>::SharedPtr sub_path_;  //!< @brief subscriber for reference path
  rclcpp::Subscription<PredictedObjects>::SharedPtr
    sub_objects_;  //!< @brief subscribe for dynamic objects
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;

  // cached inputs
  PredictedObjects::ConstSharedPtr dynamic_objects_ptr_;
  lanelet::LaneletMapPtr lanelet_map_ptr_{new lanelet::LaneletMap};
  multilinestring_t uncrossable_lines_;

  // parameters
  PreprocessingParameters preprocessing_params_;
  ExpansionParameters expansion_params_;
  double vehicle_lateral_offset_;
  double vehicle_front_offset_;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /// @brief callback for parameter updates
  /// @param[in] parameters updated parameters and their new values
  /// @return result of parameter update
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  /// @brief callback for input path. Publishes a path with an expanded drivable area
  /// @param[in] msg input path message
  void onPath(const Path::ConstSharedPtr msg);

  /// @brief validate the inputs of the node
  /// @param[in] ego_idx path index closest to the current ego pose
  /// @return true if the inputs are valid
  bool validInputs(const boost::optional<size_t> & ego_idx);
};
}  // namespace drivable_area_expander

#endif  // DRIVABLE_AREA_EXPANDER__DRIVABLE_AREA_EXPANDER_NODE_HPP_
