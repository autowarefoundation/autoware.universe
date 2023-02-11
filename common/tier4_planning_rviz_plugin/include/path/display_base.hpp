// Copyright 2023 TIER IV, Inc.
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

#ifndef PATH__DISPLAY_BASE_HPP_
#define PATH__DISPLAY_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_rendering/objects/movable_text.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <deque>
#include <memory>

namespace rviz_plugins
{
template <typename T>
class AutowarePathBaseDisplay : public rviz_common::MessageFilterDisplay<T>
{
public:
  AutowarePathBaseDisplay();
  ~AutowarePathBaseDisplay() override;

  void onInitialize() override;
  void reset() override;

protected:
  void processMessage(const typename T::ConstSharedPtr msg_ptr) override;

  Ogre::ManualObject * path_manual_object_{nullptr};
  Ogre::ManualObject * velocity_manual_object_{nullptr};
  std::vector<rviz_rendering::MovableText *> velocity_texts_;
  std::vector<Ogre::SceneNode *> velocity_text_nodes_;
  rviz_common::properties::BoolProperty property_path_view_;
  rviz_common::properties::BoolProperty property_velocity_view_;
  rviz_common::properties::FloatProperty property_path_width_;
  rviz_common::properties::ColorProperty property_path_color_;
  rviz_common::properties::ColorProperty property_velocity_color_;
  rviz_common::properties::FloatProperty property_path_alpha_;
  rviz_common::properties::FloatProperty property_velocity_alpha_;
  rviz_common::properties::FloatProperty property_velocity_scale_;
  rviz_common::properties::BoolProperty property_velocity_text_view_;
  rviz_common::properties::FloatProperty property_velocity_text_scale_;
  rviz_common::properties::BoolProperty property_path_color_view_;
  rviz_common::properties::BoolProperty property_velocity_color_view_;
  rviz_common::properties::FloatProperty property_vel_max_;

private:
  autoware_auto_planning_msgs::msg::Path::ConstSharedPtr last_msg_ptr_;
};
}  // namespace rviz_plugins

#endif  // PATH__DISPLAY_BASE_HPP_
