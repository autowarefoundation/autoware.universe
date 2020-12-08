// Copyright 2020 Tier IV, Inc.
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

#include "pose_history_display.hpp"

namespace rviz_plugins
{
PoseHistory::PoseHistory()
{
  const char * topic_type = ros::message_traits::datatype<geometry_msgs::PoseStamped>();
  const char * topic_desc = "Name of topic to display";
  property_topic_ =
    new rviz::RosTopicProperty("Topic", "", topic_type, topic_desc, this, SLOT(updateTopic()));
  property_buffer_size_ = new rviz::IntProperty("Buffer Size", 100, "", this);
  property_line_view_ = new rviz::BoolProperty("Line", true, "", this);
  property_line_width_ = new rviz::FloatProperty("Width", 0.1, "", property_line_view_);
  property_line_color_ = new rviz::ColorProperty("Color", Qt::white, "", property_line_view_);

  property_buffer_size_->setMin(0);
  property_buffer_size_->setMax(16000);
  property_line_width_->setMin(0.0);
}

PoseHistory::~PoseHistory()
{
  // Properties are deleted by Qt
}

void PoseHistory::onInitialize()
{
  lines_.reset(new rviz::BillboardLine(scene_manager_, scene_node_));
}

void PoseHistory::onEnable() {subscribe();}

void PoseHistory::onDisable() {unsubscribe();}

void PoseHistory::update(float wall_dt, float ros_dt)
{
  if (!history_.empty()) {
    lines_->clear();
    if (property_line_view_->getBool()) {
      updateLines();
    }
  }
}

void PoseHistory::updateTopic()
{
  unsubscribe();
  subscribe();
}

void PoseHistory::subscribe()
{
  auto topic_name = property_topic_->getTopicStd();
  if (1 < topic_name.length()) {
    sub_ = nh_.subscribe(topic_name, 10, &PoseHistory::onMessage, this);
  }
}

void PoseHistory::unsubscribe()
{
  sub_.shutdown();

  history_.clear();
  lines_->clear();
}

void PoseHistory::onMessage(const geometry_msgs::PoseStamped & message)
{
  if (target_frame_ != message.header.frame_id) {
    history_.clear();
    target_frame_ = message.header.frame_id;
  }
  history_.emplace_back(message);

  updateHistory();
}

void PoseHistory::updateHistory()
{
  int buffer_size = property_buffer_size_->getInt();
  while (buffer_size < history_.size()) {
    history_.pop_front();
  }
}

void PoseHistory::updateLines()
{
  QColor color = property_line_color_->getColor();
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  auto frame_manager = context_->getFrameManager();
  if (!frame_manager->getTransform(target_frame_, ros::Time(0), position, orientation)) {
    std::string error;
    frame_manager->transformHasProblems(target_frame_, ros::Time(0), error);
    setStatusStd(rviz::StatusProperty::Error, "Transform", error);
    return;
  }

  setStatusStd(rviz::StatusProperty::Ok, "Transform", "Transform OK");
  lines_->setMaxPointsPerLine(history_.size());
  lines_->setLineWidth(property_line_width_->getFloat());
  lines_->setPosition(position);
  lines_->setOrientation(orientation);
  lines_->setColor(color.redF(), color.greenF(), color.blueF(), color.alphaF());

  for (const auto & message : history_) {
    Ogre::Vector3 point;
    point.x = message.pose.position.x;
    point.y = message.pose.position.y;
    point.z = message.pose.position.z;
    lines_->addPoint(point);
  }
}

}  // namespace rviz_plugins

#include "pluginlib/class_list_macros.h"
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PoseHistory, rviz::Display)
