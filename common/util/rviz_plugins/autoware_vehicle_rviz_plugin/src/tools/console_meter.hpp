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

#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
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

#include "geometry_msgs/TwistStamped.h"

#include "jsk_overlay_utils.hpp"
#endif

namespace rviz_plugins
{
class ConsoleMeterDisplay : public rviz::MessageFilterDisplay<geometry_msgs::TwistStamped>
{
  Q_OBJECT

public:
  ConsoleMeterDisplay();
  virtual ~ConsoleMeterDisplay();

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void processMessage(const geometry_msgs::TwistStampedConstPtr & msg_ptr) override;
  std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(
    const double vel_max, const double cmd_vel);
  std::unique_ptr<Ogre::ColourValue> gradation(
    const QColor & color_min, const QColor & color_max, const double ratio);
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz::ColorProperty * property_text_color_;
  rviz::IntProperty * property_left_;
  rviz::IntProperty * property_top_;
  rviz::IntProperty * property_length_;
  rviz::FloatProperty * property_handle_angle_scale_;
  rviz::IntProperty * property_value_height_offset_;
  // QImage hud_;

private:
  const double meter_min_velocity_;
  const double meter_max_velocity_;
  const double meter_min_angle_;
  const double meter_max_angle_;
  geometry_msgs::TwistStampedConstPtr last_msg_ptr_;
};

}  // namespace rviz_plugins
