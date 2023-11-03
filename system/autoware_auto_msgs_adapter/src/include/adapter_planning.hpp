// Copyright 2023 The Autoware Foundation
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
#ifndef ADAPTER_PLANNING_HPP_
#define ADAPTER_PLANNING_HPP_

#include "adapter_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <string>
 
namespace autoware_auto_msgs_adapter
{
using Trajectory_auto = autoware_auto_planning_msgs::msg::Trajectory;
using Point_auto = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using Trajectory = autoware_planning_msgs::msg::Trajectory;

class AdapterPlanning : public autoware_auto_msgs_adapter::AdapterBase<Trajectory, Trajectory_auto>
{
public:
  AdapterPlanning(
    rclcpp::Node & node, const std::string & topic_name_source,
    const std::string & topic_name_target, const rclcpp::QoS & qos = rclcpp::QoS{1})
  : AdapterBase(node, topic_name_source, topic_name_target, qos)
  {
    RCLCPP_DEBUG(
      node.get_logger(), "AdapterPlanning is initialized to convert: %s -> %s",
      topic_name_source.c_str(), topic_name_target.c_str());
  }

protected:
  Trajectory_auto convert(const Trajectory & msg_source) override
  {
    Trajectory_auto msg_auto;
    msg_auto.header = msg_source.header;
    Point_auto TrajectoryPoint_auto;
    msg_auto.points.reserve(msg_source.points.size());
    for(size_t i = 0;i < msg_source.points.size();i++)
    {       
        TrajectoryPoint_auto.time_from_start = msg_source.points.at(i).time_from_start;
        TrajectoryPoint_auto.pose = msg_source.points.at(i).pose;
        TrajectoryPoint_auto.longitudinal_velocity_mps = msg_source.points.at(i).longitudinal_velocity_mps;
        TrajectoryPoint_auto.lateral_velocity_mps = msg_source.points.at(i).lateral_velocity_mps;
        TrajectoryPoint_auto.acceleration_mps2 = msg_source.points.at(i).acceleration_mps2;
        TrajectoryPoint_auto.heading_rate_rps = msg_source.points.at(i).heading_rate_rps;
        TrajectoryPoint_auto.front_wheel_angle_rad = msg_source.points.at(i).front_wheel_angle_rad;
        TrajectoryPoint_auto.rear_wheel_angle_rad = msg_source.points.at(i).rear_wheel_angle_rad;
        msg_auto.points.push_back(TrajectoryPoint_auto);
    }

    return msg_auto;
  }
};
}  // namespace autoware_auto_msgs_adapter

#endif  // ADAPTER_PLANNING_HPP_
