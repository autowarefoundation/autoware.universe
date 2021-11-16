// Copyright 2020 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <common/types.hpp>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "lgsvl_interface/lgsvl_interface_node.hpp"
#include "lgsvl_interface/lgsvl_interface.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;
using autoware::drivers::vehicle_interface::ViFeature;

namespace lgsvl_interface
{

LgsvlInterfaceNode::LgsvlInterfaceNode(
  const rclcpp::NodeOptions & options)
: VehicleInterfaceNode{
    "lgsvl_interface",
    std::unordered_set<ViFeature> {
      ViFeature::HEADLIGHTS,
      ViFeature::HORN,
      ViFeature::WIPERS,
    },
    options
}
{
  const auto sim_ctrl_cmd_topic = "vehicle_control_cmd";
  const auto sim_state_cmd_topic = "vehicle_state_cmd";
  const auto sim_state_rpt_topic = "state_report";
  const auto sim_veh_odom_topic = "vehicle_odom";
  // Optional
  const std::string sim_nav_odom_topic =
    declare_parameter("use_nav_odometry_topic", true) ?
    "gnss_odom" : "";
  const auto kinematic_state_topic = "vehicle_kinematic_state";
  const std::string sim_odom_child_frame =
    declare_parameter("lgsvl.odom_child_frame", "base_link");
  const auto table = [this](const std::string & prefix_raw) -> Table1D {
      const std::string prefix = "lgsvl." + prefix_raw + ".";
      return Table1D{
      declare_parameter(prefix + "domain").get<std::vector<float64_t>>(),
      declare_parameter(prefix + "range").get<std::vector<float64_t>>()
      };
    };
  const auto pub_pose_param = declare_parameter("lgsvl.publish_pose");
  const bool pub_pose = rclcpp::ParameterType::PARAMETER_NOT_SET == pub_pose_param.get_type() ?
    PUBLISH : pub_pose_param.get<bool>();
  const auto pub_tf_param = declare_parameter("lgsvl.publish_tf");
  const bool pub_tf = rclcpp::ParameterType::PARAMETER_NOT_SET == pub_tf_param.get_type() ?
    NO_PUBLISH : pub_tf_param.get<bool>();

  // Set up interface
  set_interface(
    std::make_unique<LgsvlInterface>(
      *this,
      sim_ctrl_cmd_topic,
      sim_state_cmd_topic,
      sim_state_rpt_topic,
      sim_nav_odom_topic,
      sim_veh_odom_topic,
      kinematic_state_topic,
      sim_odom_child_frame,
      table("throttle"),
      table("brake"),
      table("steer"),
      pub_tf,
      pub_pose
  ));
  // TODO(c.ho) low pass filter and velocity controller
}

}  // namespace lgsvl_interface

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(lgsvl_interface::LgsvlInterfaceNode)
