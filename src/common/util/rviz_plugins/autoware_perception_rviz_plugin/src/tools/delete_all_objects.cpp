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

#include "delete_all_objects.hpp"

namespace rviz
{
DeleteAllObjectsTool::DeleteAllObjectsTool()
{
  shortcut_key_ = 'd';

  topic_property_ = new StringProperty(
    "Pose Topic", "/simulation/dummy_perceotion/publisher/object_info",
    "The topic on which to publish dummy object info.", getPropertyContainer(), SLOT(updateTopic()),
    this);
}

void DeleteAllObjectsTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Delete All Objects");
  updateTopic();
}

void DeleteAllObjectsTool::updateTopic()
{
  dummy_object_info_pub_ =
    nh_.advertise<dummy_perception_publisher::Object>(topic_property_->getStdString(), 1);
}

void DeleteAllObjectsTool::onPoseSet(double x, double y, double theta)
{
  const ros::Time current_time = ros::Time::now();
  dummy_perception_publisher::Object output_msg;
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  // header
  output_msg.header.frame_id = fixed_frame;
  output_msg.header.stamp = current_time;

  // action
  output_msg.action = dummy_perception_publisher::Object::DELETEALL;

  dummy_object_info_pub_.publish(output_msg);
}

}  // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::DeleteAllObjectsTool, rviz::Tool)
