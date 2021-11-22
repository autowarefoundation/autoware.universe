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
      declare_parameter<std::vector<float64_t>>(prefix + "domain"),
      declare_parameter<std::vector<float64_t>>(prefix + "range")
      };
    };
  const bool8_t pub_pose = declare_parameter<bool8_t>("lgsvl.publish_pose", PUBLISH);
  const bool8_t pub_tf = declare_parameter<bool8_t>("lgsvl.publish_tf", NO_PUBLISH);

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

  // Replicate published topics from the SimplePlanningSimulator
  // TODO(Maxime CLEMENT): move to another node ?
  using rclcpp::QoS;
  pub_control_mode_report_ =
    create_publisher<ControlModeReport>("output/control_mode_report", QoS{1});
  pub_gear_report_ = create_publisher<GearReport>("output/gear_report", QoS{1});
  pub_velocity_ = create_publisher<VelocityReport>("output/twist", QoS{1});
  pub_odom_ = create_publisher<Odometry>("output/odometry", QoS{1});
  pub_steer_ = create_publisher<SteeringReport>("output/steering", QoS{1});

  sub_odom_ = create_subscription<Odometry>(
    "/lgsvl/gnss_odom", QoS{1}, [&](const Odometry::ConstSharedPtr odom_msg) {
      pub_odom_->publish(*odom_msg);

      VelocityReport velocity;
      velocity.longitudinal_velocity = static_cast<float>(odom_msg->twist.twist.linear.x);
      velocity.lateral_velocity = 0.0F;
      velocity.heading_rate = static_cast<float>(odom_msg->twist.twist.angular.z);
      pub_velocity_->publish(velocity);
    });
  sub_vehicle_odom_ = create_subscription<VehicleOdometry>(
    "/lgsvl/vehicle_odom", QoS{1}, [&](const VehicleOdometry::ConstSharedPtr odom_msg) {
      autoware_auto_vehicle_msgs::msg::SteeringReport steer;
      steer.steering_tire_angle = static_cast<float>(odom_msg->front_wheel_angle_rad);
      pub_steer_->publish(steer);
    });
  sub_state_ = create_subscription<lgsvl_msgs::msg::CanBusData>(
    "/lgsvl/vehicle_odom", QoS{1}, [&](const lgsvl_msgs::msg::CanBusData::ConstSharedPtr state_msg) {
      {
        GearReport msg;
        msg.stamp = get_clock()->now();
        switch(state_msg->selected_gear) {
          case (lgsvl_msgs::msg::CanBusData::GEAR_DRIVE):
            msg.report = GearReport::DRIVE;
            break;
          case (lgsvl_msgs::msg::CanBusData::GEAR_REVERSE):
            msg.report = GearReport::REVERSE;
            break;
          case (lgsvl_msgs::msg::CanBusData::GEAR_LOW):
            msg.report = GearReport::LOW;
            break;
          case (lgsvl_msgs::msg::CanBusData::GEAR_NEUTRAL):
          case (lgsvl_msgs::msg::CanBusData::GEAR_PARKING):
          default:
            msg.report = GearReport::PARK;
            break;
        }
        pub_gear_report_->publish(msg);
      }
      {
        ControlModeReport msg;
        msg.stamp = get_clock()->now();
        msg.mode = ControlModeReport::AUTONOMOUS;
        pub_control_mode_report_->publish(msg);
      }
    });
}

}  // namespace lgsvl_interface

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(lgsvl_interface::LgsvlInterfaceNode)
