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

#include "autoware/trajectory_evaluator/trajectory_evaluator_node.hpp"
#include "std_msgs/msg/string.hpp" 
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"

namespace trajectory_evaluator
{
TrajectoryEvaluatorNode::TrajectoryEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("trajectory_evaluator", node_options)
{
    // Publishers
    using namespace std::literals::chrono_literals;
    timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TrajectoryEvaluatorNode::onTimer, this));
}

void TrajectoryEvaluatorNode::onTimer()
{
    auto current_time = now();

    const auto ego_state_ptr = kinematic_state_sub_.takeData();
    onKinematicState(ego_state_ptr);

    const auto traj_msg = traj_sub_.takeData();
    onTrajectory(traj_msg);
}


void TrajectoryEvaluatorNode::onTrajectory(Trajectory::ConstSharedPtr traj_msg) {
    if (traj_msg != nullptr) {
        trajectory_history_.push_back({traj_msg, this->now()});
        if (trajectory_history_.size() > 100) {
            trajectory_history_.erase(trajectory_history_.begin());
        }
    }
}

void TrajectoryEvaluatorNode::calculate_time_from_start(
    const TrajectoryWithTimestamp &trajectory_with_timestamp,
    const geometry_msgs::msg::Point &current_ego_point, const float min_velocity, std::vector<TrajectoryPointWithTime> &trajectory_with_time)
{
    auto trajectory_ = trajectory_with_timestamp.trajectory->points;
    const auto nearest_segment_idx = autoware::motion_utils::findNearestSegmentIndex(trajectory_, current_ego_point);
    if (nearest_segment_idx + 1 >= trajectory_.size()) {
        return; 
    }

    // Initialize trajectory_with_time with timestamps
    for (const auto &p : trajectory_) {
        trajectory_with_time.push_back(TrajectoryPointWithTime(p, rclcpp::Duration::from_seconds(0), trajectory_with_timestamp.time_stamp));
    }

    // Calculate the time from start and save timestamps
    for (auto idx = nearest_segment_idx + 1; idx < trajectory_with_time.size(); ++idx) {
        const auto &from = trajectory_with_time[idx - 1].point; 
        const auto velocity = std::max(min_velocity, from.longitudinal_velocity_mps);
        if (velocity != 0.0) {
            auto &to = trajectory_with_time[idx];
            const auto t = autoware::universe_utils::calcDistance2d(from, to.point) / velocity;

            to.time_from_start = rclcpp::Duration::from_seconds(t) + trajectory_with_time[idx - 1].time_from_start;

            to.time_stamp = trajectory_with_time[idx - 1].time_stamp;
        }
    }
}

void TrajectoryEvaluatorNode::onKinematicState(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    const geometry_msgs::msg::Pose& pose = odom_msg->pose.pose;

    std::vector<double> time_errors_;

    for (const auto& trajectory : trajectory_history_) {
        std::vector<TrajectoryPointWithTime> trajectory_with_time;
        calculate_time_from_start(trajectory, pose.position, 1e-3, trajectory_with_time);

        double min_distance = std::numeric_limits<double>::max();
        size_t closest_index = trajectory_with_time.size();

        for (size_t i = 0; i < trajectory_with_time.size(); ++i) {
            const auto& trajectory_point_pose = trajectory_with_time[i].point.pose;
            double distance = std::sqrt(
                std::pow(trajectory_point_pose.position.x - pose.position.x, 2) +
                std::pow(trajectory_point_pose.position.y - pose.position.y, 2)
            );

            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i; 
            }
        }

        if (closest_index < trajectory_with_time.size()) {
            const auto& closest_point = trajectory_with_time[closest_index];
            rclcpp::Time timestamp = closest_point.time_stamp; 
            double actual_time = std::abs(timestamp.seconds() - odom_msg->header.stamp.sec);
            double time_error = std::abs(actual_time - closest_point.time_from_start.seconds()); 
            time_errors_.push_back(time_error);
            RCLCPP_INFO(get_logger(), "Trajectory.time_stamp for closest trajectory point %zu: %f seconds", closest_index, closest_point.time_stamp.seconds());
            RCLCPP_INFO(get_logger(), "Closest point expected for closest trajectory point %zu: %f seconds", closest_index, closest_point.time_from_start.seconds());
            RCLCPP_INFO(get_logger(), "Time error for closest trajectory point %zu: %f seconds", closest_index, time_error);
        }
    }
}

bool TrajectoryEvaluatorNode::is_pose_equal(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2) {
    const double threshold = 1e-3;
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;

    return (dx * dx + dy * dy + dz * dz) < (threshold * threshold);
}

}  // namespace trajectory_evaluator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_evaluator::TrajectoryEvaluatorNode)
