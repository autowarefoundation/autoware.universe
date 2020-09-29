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

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz/message_filter_display.h>
// #include <rviz/properties/ros_topic_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/validate_floats.h>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <deque>
#include <memory>

#include "std_msgs/Float32.h"

#include "jsk_overlay_utils.hpp"
#endif

namespace rviz_plugins
{
class MaxVelocityDisplay : public rviz::Display
{
  Q_OBJECT

public:
  MaxVelocityDisplay();
  virtual ~MaxVelocityDisplay();

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;
  void subscribe();
  void unsubscribe();
private Q_SLOTS:
  void updateTopic();
  void updateVisualization();

protected:
  void processMessage(const std_msgs::Float32ConstPtr & msg_ptr);
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz::ColorProperty * property_text_color_;
  rviz::IntProperty * property_left_;
  rviz::IntProperty * property_top_;
  rviz::IntProperty * property_length_;
  rviz::RosTopicProperty * property_topic_name_;
  rviz::FloatProperty * property_value_scale_;

private:
  ros::Subscriber sub_;
  std_msgs::Float32ConstPtr last_msg_ptr_;
};

}  // namespace rviz_plugins
