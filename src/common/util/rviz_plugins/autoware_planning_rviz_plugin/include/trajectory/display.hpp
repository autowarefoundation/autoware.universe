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

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <ros/ros.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/validate_floats.h>

#include <deque>
#include <memory>

#include "autoware_planning_msgs/Trajectory.h"

namespace rviz_plugins
{
class AutowareTrajectoryDisplay
: public rviz::MessageFilterDisplay<autoware_planning_msgs::Trajectory>
{
  Q_OBJECT

public:
  AutowareTrajectoryDisplay();
  virtual ~AutowareTrajectoryDisplay();

  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void processMessage(const autoware_planning_msgs::TrajectoryConstPtr & msg_ptr) override;
  std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(
    const double vel_max, const double cmd_vel);
  std::unique_ptr<Ogre::ColourValue> gradation(
    const QColor & color_min, const QColor & color_max, const double ratio);
  Ogre::ManualObject * path_manual_object_;
  Ogre::ManualObject * velocity_manual_object_;
  rviz::BoolProperty * property_path_view_;
  rviz::BoolProperty * property_velocity_view_;
  rviz::FloatProperty * property_path_width_;
  rviz::ColorProperty * property_path_color_;
  rviz::ColorProperty * property_velocity_color_;
  rviz::FloatProperty * property_velocity_scale_;
  rviz::FloatProperty * property_path_alpha_;
  rviz::FloatProperty * property_velocity_alpha_;
  rviz::BoolProperty * property_path_color_view_;
  rviz::BoolProperty * property_velocity_color_view_;
  rviz::FloatProperty * property_vel_max_;

private:
  autoware_planning_msgs::TrajectoryConstPtr last_msg_ptr_;
  bool validateFloats(const autoware_planning_msgs::TrajectoryConstPtr & msg_ptr);
};

}  // namespace rviz_plugins
