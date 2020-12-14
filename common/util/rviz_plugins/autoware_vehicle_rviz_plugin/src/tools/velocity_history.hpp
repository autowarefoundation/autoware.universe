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

#include "OgreBillboardSet.h"
#include "OgreManualObject.h"
#include "OgreMaterialManager.h"
#include "OgreSceneManager.h"
#include "OgreSceneNode.h"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"

#include <deque>
#include <memory>
#include <tuple>

#include "geometry_msgs/msg/twist_stamped.hpp"

namespace rviz_plugins
{
class VelocityHistoryDisplay : public rviz_common::MessageFilterDisplay<geometry_msgs::msg::TwistStamped>
{
  Q_OBJECT

public:
  VelocityHistoryDisplay();
  virtual ~VelocityHistoryDisplay();

  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void processMessage(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg_ptr) override;
  std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(
    const double vel_max, const double cmd_vel);
  std::unique_ptr<Ogre::ColourValue> gradation(
    const QColor & color_min, const QColor & color_max, const double ratio);
  Ogre::ManualObject * velocity_manual_object_;
  rviz_common::properties::FloatProperty * property_velocity_timeout_;
  rviz_common::properties::FloatProperty * property_path_alpha_;
  rviz_common::properties::FloatProperty * property_velocity_alpha_;
  rviz_common::properties::FloatProperty * property_velocity_scale_;
  rviz_common::properties::BoolProperty * property_velocity_color_view_;
  rviz_common::properties::ColorProperty * property_velocity_color_;
  rviz_common::properties::FloatProperty * property_vel_max_;

private:
  std::deque<std::tuple<geometry_msgs::msg::TwistStamped::ConstSharedPtr, Ogre::Vector3>> history_;
  bool validateFloats(const geometry_msgs::msg::TwistStamped::ConstSharedPtr & msg_ptr);
};

}  // namespace rviz_plugins
