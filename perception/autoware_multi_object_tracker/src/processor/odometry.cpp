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

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

std::optional<geometry_msgs::msg::Transform> Odometry::getTransform(const rclcpp::Time & time) const
{
  return getTransform(ego_frame_id_, time);
}

std::optional<geometry_msgs::msg::Transform> Odometry::getTransform(
  const std::string & source_frame_id, const rclcpp::Time & time) const
{
  try {
    // Check if the frames are ready
    std::string errstr;  // This argument prevents error msg from being displayed in the terminal.
    if (!tf_buffer_.canTransform(
          world_frame_id_, source_frame_id, tf2::TimePointZero, tf2::Duration::zero(), &errstr)) {
      return std::nullopt;
    }

    // Lookup the transform
    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer_.lookupTransform(
      world_frame_id_, source_frame_id, time, rclcpp::Duration::from_seconds(0.5));
    return std::optional<geometry_msgs::msg::Transform>(self_transform_stamped.transform);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("multi_object_tracker"), ex.what());
    return std::nullopt;
  }
}

bool Odometry::setOdometryFromTf(const rclcpp::Time & time)
{
  const auto self_transform = getTransform(time);
  if (!self_transform) {
    return false;
  }
  current_transform_ = self_transform.value();

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

std::optional<types::DynamicObjectList> Odometry::transformObjects(
  const types::DynamicObjectList & input_objects) const
{
  types::DynamicObjectList output_objects = input_objects;

  // transform to world coordinate
  if (input_objects.header.frame_id != world_frame_id_) {
    output_objects.header.frame_id = world_frame_id_;
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    {
      const auto ros_target2objects_world =
        getTransform(input_objects.header.frame_id, input_objects.header.stamp);
      if (!ros_target2objects_world) {
        return std::nullopt;
      }
      tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    }
    for (auto & object : output_objects.objects) {
      auto & pose_with_cov = object.kinematics.pose_with_covariance;
      tf2::fromMsg(pose_with_cov.pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      // transform pose, frame difference and object pose
      tf2::toMsg(tf_target2objects, pose_with_cov.pose);
      // transform covariance, only the frame difference
      pose_with_cov.covariance =
        tf2::transformCovariance(pose_with_cov.covariance, tf_target2objects_world);
    }
  }
  return std::optional<types::DynamicObjectList>(output_objects);
}

}  // namespace autoware::multi_object_tracker
