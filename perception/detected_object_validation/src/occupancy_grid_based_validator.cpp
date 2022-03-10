// Copyright 2022 Tier IV, Inc.
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

#include "occupancy_grid_based_validator/occupancy_grid_based_validator.hpp"

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace occupancy_grid_based_validator
{
using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

OccupancyGridBasedValidator::OccupancyGridBasedValidator(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("occupancy_grid_based_validator", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  objects_sub_(this, "~input/detected_objects", rclcpp::QoS{1}.get_rmw_qos_profile()),
  occ_grid_sub_(this, "~input/occupancy_grid_map", rclcpp::QoS{1}.get_rmw_qos_profile()),
  sync_(SyncPolicy(10), objects_sub_, occ_grid_sub_)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_.registerCallback(
    std::bind(&OccupancyGridBasedValidator::onObjectsAndOccGrid, this, _1, _2));

  objects_pub_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "~output/object", rclcpp::QoS{1});
}

void OccupancyGridBasedValidator::onObjectsAndOccGrid(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects,
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & input_occ_grid)
{
  for (const auto & object : input_objects.objects) {
    const auto & label = object.classification.front().label;
    const bool is_vehicle = Label::CAR == label || Label::TRUCK == label || Label::BUS == label ||
                            Label::TRAILER == label;
    if (is_vehicle) {
      object.shape
    }
  }
}
}  // namespace occupancy_grid_based_validator

RCLCPP_COMPONENTS_REGISTER_NODE(occupancy_grid_based_validator::OccupancyGridBasedValidator)
