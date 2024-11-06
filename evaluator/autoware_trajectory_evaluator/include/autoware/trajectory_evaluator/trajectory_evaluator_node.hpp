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

#ifndef AUTOWARE__TRAJECTORY_EVALUATOR__TRAJECTORY_EVALUATOR_NODE_HPP_
#define AUTOWARE__TRAJECTORY_EVALUATOR__TRAJECTORY_EVALUATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <vector>
#include <cmath>
#include <chrono>
#include <deque>

namespace trajectory_evaluator {

struct TrajectoryPointWithTime {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    rclcpp::Duration time_from_start;
    rclcpp::Time time_stamp;

    TrajectoryPointWithTime(const autoware_planning_msgs::msg::TrajectoryPoint& p, 
                            const rclcpp::Duration& time_start, 
                            const rclcpp::Time& ts)
        : point(p),
          time_from_start(time_start), 
          time_stamp(ts) {}
};

struct TrajectoryWithTimestamp {
    std::vector<TrajectoryPointWithTime> trajectory_points;
    rclcpp::Time time_stamp;
};

std::vector<TrajectoryWithTimestamp> trajectory_history_;

class TrajectoryEvaluatorNode : public rclcpp::Node
{
public:
    explicit TrajectoryEvaluatorNode(const rclcpp::NodeOptions& node_options);
void onTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr traj_msg, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
void onKinematicState(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
bool is_pose_equal(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2);

private:
    std::shared_ptr<rclcpp::TimerBase> traj_timer_;
    std::shared_ptr<rclcpp::TimerBase> odom_timer_;
    void onTimer();

    // void onKinematicState(const Odometry::ConstSharedPtr odom_msg);
    void calculate_time_from_start(
 TrajectoryWithTimestamp & trajectory,
  const geometry_msgs::msg::Point & current_ego_point, const float min_velocity);
    
    // Subscribers
    autoware::universe_utils::InterProcessPollingSubscriber<autoware_planning_msgs::msg::Trajectory> traj_sub_{
        this, "/planning/scenario_planning/trajectory"};
    autoware::universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> kinematic_state_sub_{
        this, "/localization/kinematic_state"};

    // Store past trajectory messages
    std::deque<TrajectoryWithTimestamp> trajectory_history_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Pose prev_pose;
    rclcpp::Time start_time_;
};
}  // namespace trajectory_evaluator

#endif  // AUTOWARE__TRAJECTORY_EVALUATOR__TRAJECTORY_EVALUATOR_NODE_HPP_
