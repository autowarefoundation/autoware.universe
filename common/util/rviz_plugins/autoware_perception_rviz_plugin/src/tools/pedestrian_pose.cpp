/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf/transform_listener.h>

#include "dummy_perception_publisher/Object.h"

#include "rviz/display_context.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/string_property.h"

#include <unique_id/unique_id.h>

#include "pedestrian_pose.hpp"

namespace rviz
{
PedestrianInitialPoseTool::PedestrianInitialPoseTool()
{
  shortcut_key_ = 'l';

  topic_property_ = new StringProperty(
    "Pose Topic", "/simulation/dummy_perceotion/publisher/object_info",
    "The topic on which to publish dummy object info.", getPropertyContainer(), SLOT(updateTopic()),
    this);
  std_dev_x_ = new FloatProperty(
    "X std deviation", 0.03, "X standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_y_ = new FloatProperty(
    "Y std deviation", 0.03, "Y standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_z_ = new FloatProperty(
    "Z std deviation", 0.03, "Z standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_theta_ = new FloatProperty(
    "Theta std deviation", 5.0 * M_PI / 180.0, "Theta standard deviation for initial pose [rad]",
    getPropertyContainer());
  position_z_ =
    new FloatProperty("Z position", 0.0, "Z position for initial pose [m]", getPropertyContainer());
  velocity_ = new FloatProperty("Velocity", 0.0, "velocity [m/s]", getPropertyContainer());
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_z_->setMin(0);
  std_dev_theta_->setMin(0);
  position_z_->setMin(0);
}

void PedestrianInitialPoseTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("2D Dummy Pedestrian");
  updateTopic();
}

void PedestrianInitialPoseTool::updateTopic()
{
  dummy_object_info_pub_ =
    nh_.advertise<dummy_perception_publisher::Object>(topic_property_->getStdString(), 1);
}

void PedestrianInitialPoseTool::onPoseSet(double x, double y, double theta)
{
  const ros::Time current_time = ros::Time::now();
  dummy_perception_publisher::Object output_msg;
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  // header
  output_msg.header.frame_id = fixed_frame;
  output_msg.header.stamp = current_time;

  // semantic
  output_msg.semantic.type = autoware_perception_msgs::Semantic::PEDESTRIAN;
  output_msg.semantic.confidence = 1.0;

  // shape
  output_msg.shape.type = autoware_perception_msgs::Shape::CYLINDER;
  const double width = 0.8;
  const double length = 0.8;
  output_msg.shape.dimensions.x = length;
  output_msg.shape.dimensions.y = width;
  output_msg.shape.dimensions.z = 2.0;

  // inital state
  // pose
  output_msg.initial_state.pose_covariance.pose.position.x = x;
  output_msg.initial_state.pose_covariance.pose.position.y = y;
  output_msg.initial_state.pose_covariance.pose.position.z = position_z_->getFloat();
  output_msg.initial_state.pose_covariance.covariance[0] =
    std_dev_x_->getFloat() * std_dev_x_->getFloat();
  output_msg.initial_state.pose_covariance.covariance[7] =
    std_dev_y_->getFloat() * std_dev_y_->getFloat();
  output_msg.initial_state.pose_covariance.covariance[14] =
    std_dev_z_->getFloat() * std_dev_z_->getFloat();
  output_msg.initial_state.pose_covariance.covariance[35] =
    std_dev_theta_->getFloat() * std_dev_theta_->getFloat();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::quaternionTFToMsg(quat, output_msg.initial_state.pose_covariance.pose.orientation);
  ROS_INFO(
    "Setting pose: %.3f %.3f %.3f %.3f [frame=%s]", x, y, position_z_->getFloat(), theta,
    fixed_frame.c_str());
  // twist
  output_msg.initial_state.twist_covariance.twist.linear.x = velocity_->getFloat();
  output_msg.initial_state.twist_covariance.twist.linear.y = 0.0;
  output_msg.initial_state.twist_covariance.twist.linear.z = 0.0;
  ROS_INFO(
    "Setting twist: %.3f %.3f %.3f [frame=%s]", velocity_->getFloat(), 0.0, 0.0,
    fixed_frame.c_str());

  // action
  output_msg.action = dummy_perception_publisher::Object::ADD;

  // id
  output_msg.id = unique_id::toMsg(unique_id::fromRandom());

  dummy_object_info_pub_.publish(output_msg);
}

}  // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::PedestrianInitialPoseTool, rviz::Tool)
