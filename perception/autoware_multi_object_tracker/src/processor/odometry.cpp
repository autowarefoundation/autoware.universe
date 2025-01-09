// Copyright 2025 TIER IV, Inc.
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

#include "odometry.hpp"

#include <tf2_ros/create_timer_ros.h>

#include <memory>
#include <string>

namespace autoware::multi_object_tracker
{

Odometry::Odometry(rclcpp::Node & node, const std::string & world_frame_id)
: node_(node),
  world_frame_id_(world_frame_id),
  tf_buffer_(node_.get_clock()),
  tf_listener_(tf_buffer_)
{
  // Create tf timer
  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
    node_.get_node_base_interface(), node_.get_node_timers_interface());
  tf_buffer_.setCreateTimerInterface(cti);
}

bool Odometry::setOdometryFromTf(const rclcpp::Time & time)
{
  // Get the transform of the self frame
  try {
    // Check if the frames are ready
    std::string errstr;  // This argument prevents error msg from being displayed in the terminal.
    if (!tf_buffer_.canTransform(
          world_frame_id_, ego_frame_id_, tf2::TimePointZero, tf2::Duration::zero(), &errstr)) {
      return false;
    }

    // Lookup the transform
    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer_.lookupTransform(
      /*target*/ world_frame_id_, /*src*/ ego_frame_id_, time, rclcpp::Duration::from_seconds(0.5));

    // set the current transform and continue processing
    current_transform_ = self_transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("multi_object_tracker"), ex.what());
    return false;
  }

  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = time + rclcpp::Duration::from_seconds(0.00001);

    // set the odometry pose
    auto & odom_pose = odometry.pose.pose;
    odom_pose.position.x = current_transform_.translation.x;
    odom_pose.position.y = current_transform_.translation.y;
    odom_pose.position.z = current_transform_.translation.z;
    odom_pose.orientation = current_transform_.rotation;

    // set odometry twist
    auto & odom_twist = odometry.twist.twist;
    odom_twist.linear.x = 10.0;  // m/s
    odom_twist.linear.y = 0.1;   // m/s
    odom_twist.angular.z = 0.1;  // rad/s

    // model the uncertainty
    auto & odom_pose_cov = odometry.pose.covariance;
    odom_pose_cov[0] = 0.1;      // x-x
    odom_pose_cov[7] = 0.1;      // y-y
    odom_pose_cov[35] = 0.0001;  // yaw-yaw

    auto & odom_twist_cov = odometry.twist.covariance;
    odom_twist_cov[0] = 2.0;     // x-x [m^2/s^2]
    odom_twist_cov[7] = 0.2;     // y-y [m^2/s^2]
    odom_twist_cov[35] = 0.001;  // yaw-yaw [rad^2/s^2]

    current_odometry_ = odometry;
  }

  return true;
}

}  // namespace autoware::multi_object_tracker
