/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pure_pursuit/pure_pursuit_node.h"
#include "pure_pursuit/pure_pursuit_viz.h"
#include "pure_pursuit/util/planning_utils.h"
#include "pure_pursuit/util/tf_utils.h"

namespace
{
autoware_control_msgs::ControlCommand createControlCommand(
  const double kappa, const double velocity, const double acceleration, const double wheel_base)
{
  autoware_control_msgs::ControlCommand cmd;
  cmd.velocity = velocity;
  cmd.acceleration = acceleration;
  cmd.steering_angle = planning_utils::convertCurvatureToSteeringAngle(wheel_base, kappa);
  return cmd;
}

double calcLookaheadDistance(
  const double velocity, const double lookahead_distance_ratio, const double min_lookahead_distance)
{
  const double lookahead_distance = lookahead_distance_ratio * std::abs(velocity);
  return std::max(lookahead_distance, min_lookahead_distance);
}

}  // namespace

PurePursuitNode::PurePursuitNode() : nh_(""), private_nh_("~"), tf_listener_(tf_buffer_)
{
  pure_pursuit_ = std::make_unique<planning_utils::PurePursuit>();

  // Global Parameters
  private_nh_.param<double>("/vehicle_info/wheel_base", param_.wheel_base, 2.7);

  // Node Parameters
  private_nh_.param<double>("control_period", param_.ctrl_period, 0.02);

  // Algorithm Parameters
  private_nh_.param<double>("lookahead_distance_ratio", param_.lookahead_distance_ratio, 2.2);
  private_nh_.param<double>("min_lookahead_distance", param_.min_lookahead_distance, 2.5);
  private_nh_.param<double>(
    "reverse_min_lookahead_distance", param_.reverse_min_lookahead_distance, 7.0);

  // Subscribers
  sub_trajectory_ =
    private_nh_.subscribe("input/reference_trajectory", 1, &PurePursuitNode::onTrajectory, this);
  sub_current_velocity_ =
    private_nh_.subscribe("input/current_velocity", 1, &PurePursuitNode::onCurrentVelocity, this);

  // Publishers
  pub_ctrl_cmd_ =
    private_nh_.advertise<autoware_control_msgs::ControlCommandStamped>("output/control_raw", 1);

  // Debug Publishers
  pub_debug_marker_ = private_nh_.advertise<visualization_msgs::MarkerArray>("debug/marker", 0);

  // Timer
  timer_ = nh_.createTimer(ros::Duration(param_.ctrl_period), &PurePursuitNode::onTimer, this);

  //  Wait for first current pose
  tf_utils::waitForTransform(tf_buffer_, "map", "base_link");
}

bool PurePursuitNode::isDataReady() const
{
  if (!current_velocity_) {
    ROS_WARN_THROTTLE(5.0, "waiting for current_velocity...");
    return false;
  }

  if (!trajectory_) {
    ROS_WARN_THROTTLE(5.0, "waiting for trajectory...");
    return false;
  }

  if (!current_pose_) {
    ROS_WARN_THROTTLE(5.0, "waiting for current_pose...");
    return false;
  }

  return true;
}

void PurePursuitNode::onCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  current_velocity_ = msg;
}

void PurePursuitNode::onTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & msg)
{
  trajectory_ = msg;
}

void PurePursuitNode::onTimer(const ros::TimerEvent & event)
{
  current_pose_ = tf_utils::getCurrentPose(tf_buffer_);

  if (!isDataReady()) {
    return;
  }

  const auto target_values = calcTargetValues();

  if (target_values) {
    publishCommand(*target_values);
    publishDebugMarker();
  } else {
    ROS_ERROR("failed to solve pure_pursuit");
    publishCommand({0.0, 0.0, 0.0});
  }
}

void PurePursuitNode::publishCommand(const TargetValues & targets) const
{
  autoware_control_msgs::ControlCommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.control =
    createControlCommand(targets.kappa, targets.velocity, targets.acceleration, param_.wheel_base);
  pub_ctrl_cmd_.publish(cmd);
}

void PurePursuitNode::publishDebugMarker() const
{
  visualization_msgs::MarkerArray marker_array;

  marker_array.markers.push_back(createNextTargetMarker(debug_data_.next_target));
  marker_array.markers.push_back(
    createTrajectoryCircleMarker(debug_data_.next_target, current_pose_->pose));

  pub_debug_marker_.publish(marker_array);
}

boost::optional<TargetValues> PurePursuitNode::calcTargetValues() const
{
  // Calculate target point for velocity/acceleration
  const auto target_point = calcTargetPoint();
  if (!target_point) {
    return {};
  }

  const double target_vel = target_point->twist.linear.x;
  const double target_acc = target_point->accel.linear.x;

  // Calculate lookahead ditance
  const bool is_reverse = (target_vel < 0);
  const double min_lookahead_distance =
    is_reverse ? param_.reverse_min_lookahead_distance : param_.min_lookahead_distance;
  const double lookahead_distance = calcLookaheadDistance(
    current_velocity_->twist.linear.x, param_.lookahead_distance_ratio, min_lookahead_distance);

  // Set PurePursuit data
  pure_pursuit_->setCurrentPose(current_pose_->pose);
  pure_pursuit_->setWaypoints(planning_utils::extractPoses(*trajectory_));
  pure_pursuit_->setLookaheadDistance(lookahead_distance);

  // Run PurePursuit
  const auto pure_pursuit_result = pure_pursuit_->run();
  if (!pure_pursuit_result.first) {
    return {};
  }

  const auto kappa = pure_pursuit_result.second;

  // Set debug data
  debug_data_.next_target = pure_pursuit_->getLocationOfNextTarget();

  return TargetValues{kappa, target_vel, target_acc};
}

boost::optional<autoware_planning_msgs::TrajectoryPoint> PurePursuitNode::calcTargetPoint() const
{
  const auto closest_idx_result = planning_utils::findClosestIdxWithDistAngThr(
    planning_utils::extractPoses(*trajectory_), current_pose_->pose, 3.0, M_PI_4);

  if (!closest_idx_result.first) {
    ROS_ERROR("cannot find closest waypoint");
    return {};
  }

  return trajectory_->points.at(closest_idx_result.second);
}
