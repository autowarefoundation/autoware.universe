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
    const auto traj_msg = traj_sub_.takeData();
    onTrajectory(traj_msg, ego_state_ptr);
    onKinematicState(ego_state_ptr);

}


void TrajectoryEvaluatorNode::onTrajectory(const Trajectory::ConstSharedPtr traj_msg, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    if (traj_msg != nullptr) {
        TrajectoryWithTimestamp trajectory_with_timestamp;
        trajectory_with_timestamp.time_stamp = this->now();
        const auto current_position = odom_msg->pose.pose.position;
        std::vector<TrajectoryPointWithTime> trajectory_with_time;

        for (const auto& point : traj_msg->points) {
            TrajectoryPointWithTime point_with_time(point, rclcpp::Duration::from_seconds(0), trajectory_with_timestamp.time_stamp);
            trajectory_with_timestamp.trajectory_points.push_back(point_with_time);
        }

        calculate_time_from_start(trajectory_with_timestamp, current_position, 1e-3);
        trajectory_history_.push_back(trajectory_with_timestamp);
        if (trajectory_history_.size() > 10) {
            trajectory_history_.erase(trajectory_history_.begin());
        }
    }
}

void TrajectoryEvaluatorNode::calculate_time_from_start(
    TrajectoryWithTimestamp &trajectory,
    const geometry_msgs::msg::Point &current_ego_point, 
    const float min_velocity)
{
    auto &trajectory_points = trajectory.trajectory_points;
    std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory_points_sequence;
    for (const auto& traj_point_with_time : trajectory_points) {
        trajectory_points_sequence.push_back(traj_point_with_time.point);
    }

    const auto nearest_segment_idx = autoware::motion_utils::findNearestSegmentIndex(trajectory_points_sequence, current_ego_point);
    
    if (nearest_segment_idx + 1 >= trajectory_points.size()) {
        return; 
    }

    for (auto &point_with_time : trajectory_points) {
        point_with_time.time_from_start = rclcpp::Duration::from_seconds(0);
    }

    for (size_t idx = nearest_segment_idx + 1; idx < trajectory_points.size(); ++idx) {
        const auto &from = trajectory_points[idx - 1];
        const auto velocity = std::max(min_velocity, from.point.longitudinal_velocity_mps);
        
        if (velocity != 0.0) {
            auto &to = trajectory_points[idx];
            const double distance = autoware::universe_utils::calcDistance2d(from.point.pose, to.point.pose);
            const double time_delta = distance / velocity;

            to.time_from_start = rclcpp::Duration::from_seconds(time_delta) + trajectory_points[idx - 1].time_from_start;

            to.time_stamp = trajectory_points[idx - 1].time_stamp + rclcpp::Duration::from_seconds(time_delta);
        }
    }
}

void TrajectoryEvaluatorNode::onKinematicState(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    if (odom_msg != nullptr) {
        const geometry_msgs::msg::Pose& pose = odom_msg->pose.pose;
        
        const double velocity = std::sqrt(
            std::pow(odom_msg->twist.twist.linear.x, 2) +
            std::pow(odom_msg->twist.twist.linear.y, 2) +
            std::pow(odom_msg->twist.twist.linear.z, 2)
        );

        // Skip processing if the vehicle is stationary
        if (velocity == 0.0) {
            return;
        }

        std::vector<double> time_errors_;

        rclcpp::Time current_time = this->now();

        for (const auto& trajectory : trajectory_history_) {
            double min_distance = std::numeric_limits<double>::max();
            size_t closest_index = trajectory.trajectory_points.size();

            for (size_t i = 0; i < trajectory.trajectory_points.size(); ++i) {
                const auto& trajectory_point_pose = trajectory.trajectory_points[i].point.pose;
                double distance = std::sqrt(
                    std::pow(trajectory_point_pose.position.x - pose.position.x, 2) +
                    std::pow(trajectory_point_pose.position.y - pose.position.y, 2)
                );

                if (distance < min_distance) {
                    min_distance = distance;
                    closest_index = i; 
                }
            }

            if (closest_index < trajectory.trajectory_points.size()) {
                const auto& closest_point = trajectory.trajectory_points[closest_index];

                double time_at_pose = (current_time - trajectory.time_stamp).seconds(); // t_P
                double time_error = std::abs(time_at_pose - closest_point.time_from_start.seconds());
                time_errors_.push_back(time_error);

                // Log information
                RCLCPP_INFO(get_logger(), "Trajectory published at %f seconds", trajectory.time_stamp.seconds());
                RCLCPP_INFO(get_logger(), "Closest point expected at: %f seconds", closest_point.time_from_start.seconds());
                RCLCPP_INFO(get_logger(), "Time error for closest trajectory point: %f seconds", time_error);
            }
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
