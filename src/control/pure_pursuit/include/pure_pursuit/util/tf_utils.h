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

#pragma once

#include <string>

#include <boost/optional.hpp>  // To be replaced by std::optional in C++17

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace tf_utils
{
inline boost::optional<geometry_msgs::TransformStamped> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & from, const std::string & to,
  const ros::Time & time, const ros::Duration & duration)
{
  try {
    return tf_buffer.lookupTransform(from, to, time, duration);
  } catch (tf2::TransformException & ex) {
    return {};
  }
}

inline geometry_msgs::TransformStamped waitForTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & from, const std::string & to)
{
  while (ros::ok()) {
    try {
      const auto transform =
        tf_buffer.lookupTransform(from, to, ros::Time::now(), ros::Duration(10.0));
      return transform;
    } catch (tf2::TransformException & ex) {
      ROS_INFO("waiting for transform from `%s` to `%s` ...", from.c_str(), to.c_str());
    }
  }
}

inline geometry_msgs::PoseStamped transform2pose(const geometry_msgs::TransformStamped & transform)
{
  geometry_msgs::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

inline boost::optional<geometry_msgs::PoseStamped> getCurrentPose(
  const tf2_ros::Buffer & tf_buffer, const double timeout = 1.0)
{
  const auto tf_current_pose =
    getTransform(tf_buffer, "map", "base_link", ros::Time(0), ros::Duration(0));
  if (!tf_current_pose) {
    return {};
  }

  const auto time_diff = (ros::Time::now() - tf_current_pose->header.stamp).toSec();
  if (std::abs(time_diff) > timeout) {
    return {};
  }

  return transform2pose(*tf_current_pose);
}

}  // namespace tf_utils
