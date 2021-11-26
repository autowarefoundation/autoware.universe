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

#include "lgsvl_interface/lgsvl_interface_node.hpp"

#include "lgsvl_interface/lgsvl_interface.hpp"

#include <common/types.hpp>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;
using autoware::drivers::vehicle_interface::ViFeature;

namespace lgsvl_interface
{
LgsvlInterfaceNode::LgsvlInterfaceNode(const rclcpp::NodeOptions & options)
: VehicleInterfaceNode{
    "lgsvl_interface",
    std::unordered_set<ViFeature>{
      ViFeature::HEADLIGHTS,
      ViFeature::HORN,
      ViFeature::WIPERS,
    },
    options}
{
  const auto sim_ctrl_cmd_topic = "vehicle_control_cmd";
  const auto sim_state_cmd_topic = "vehicle_state_cmd";
  const auto sim_state_rpt_topic = "state_report";
  const auto sim_veh_odom_topic = "vehicle_odom";
  const auto gear_report_topic = "gear_report";
  const auto steer_report_topic = "steer_report";
  const auto control_mode_report_topic = "control_mode_report";
  const auto twist_topic = "twist";
  const auto odom_topic = "odom";
  // Optional
  const std::string sim_nav_odom_topic =
    declare_parameter("use_nav_odometry_topic", true) ? "gnss_odom" : "";
  const auto kinematic_state_topic = "vehicle_kinematic_state";
  const std::string sim_odom_child_frame = declare_parameter("lgsvl.odom_child_frame", "base_link");
  const auto table = [this](const std::string & prefix_raw) -> Table1D {
    const std::string prefix = "lgsvl." + prefix_raw + ".";
    return Table1D{
      declare_parameter<std::vector<float64_t>>(prefix + "domain"),
      declare_parameter<std::vector<float64_t>>(prefix + "range")};
  };
  const bool8_t pub_pose = declare_parameter<bool8_t>("lgsvl.publish_pose", PUBLISH);
  const bool8_t pub_tf = declare_parameter<bool8_t>("lgsvl.publish_tf", NO_PUBLISH);

  // Set up interface
  set_interface(std::make_unique<LgsvlInterface>(
    *this, sim_ctrl_cmd_topic, sim_state_cmd_topic, sim_state_rpt_topic, sim_nav_odom_topic,
    sim_veh_odom_topic, kinematic_state_topic, gear_report_topic, steer_report_topic,
    control_mode_report_topic, twist_topic, odom_topic, sim_odom_child_frame, table("throttle"),
    table("brake"), table("steer"), pub_tf, pub_pose));
  // TODO(c.ho) low pass filter and velocity controller
}

}  // namespace lgsvl_interface

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(lgsvl_interface::LgsvlInterfaceNode)
