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

#ifndef STRING_OVERLAY_DISPLAY_HPP_
#define STRING_OVERLAY_DISPLAY_HPP_

#include <memory>
#include <mutex>

#ifndef Q_MOC_RUN
#include "jsk_overlay_utils.hpp"

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <std_msgs/msg/string.hpp>
#endif

namespace rviz_plugins
{
class StringOverlayDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  StringOverlayDisplay();
  ~StringOverlayDisplay() override;

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;
  void subscribe();
  void unsubscribe();

private Q_SLOTS:
  void updateVisualization();

protected:
  void update(float wall_dt, float ros_dt) override;
  void processMessage(
      const std_msgs::msg::String::ConstSharedPtr msg_ptr);
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz_common::properties::IntProperty* property_left_;
  rviz_common::properties::IntProperty* property_top_;
  rviz_common::properties::IntProperty* property_width_;
  rviz_common::properties::IntProperty* property_height_;
  rviz_common::properties::StringProperty* property_topic_name_;
  rviz_common::properties::ColorProperty* property_fg_color_;
  rviz_common::properties::ColorProperty* property_bg_color_;
  rviz_common::properties::FloatProperty* property_fg_alpha_;
  rviz_common::properties::FloatProperty* property_bg_alpha_;
  rviz_common::properties::IntProperty* property_font_size_;
  QImage hud_;

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
  std_msgs::msg::String::ConstSharedPtr last_msg_ptr_ = nullptr;
  std::string topic_name_;
};

}  // namespace rviz_plugins

#endif  // STRING_OVERLAY_DISPLAY_HPP_
