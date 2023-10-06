// Copyright 2023 Tier IV, Inc.
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

// Copyright (c) 2014, JSK Lab
// All rights reserved.
//
// Software License Agreement (BSD License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.S SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#ifndef MRM_SUMMARY_OVERLAY_DISPLAY_HPP_
#define MRM_SUMMARY_OVERLAY_DISPLAY_HPP_

#include <memory>
#include <mutex>

#ifndef Q_MOC_RUN
#include "jsk_overlay_utils.hpp"

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <autoware_auto_system_msgs/msg/hazard_status_stamped.hpp>
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>

#endif

namespace rviz_plugins
{
class MrmSummaryOverlayDisplay
: public rviz_common::RosTopicDisplay<autoware_auto_system_msgs::msg::HazardStatusStamped>

{
  using LocalizationInitializationState =
    autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;

  Q_OBJECT

public:
  MrmSummaryOverlayDisplay();
  ~MrmSummaryOverlayDisplay() override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void update(float wall_dt, float ros_dt) override;
  void processMessage(
    const autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg_ptr) override;
  rclcpp::Node::SharedPtr raw_node_;

  rclcpp::Subscription<LocalizationInitializationState>::SharedPtr sub_localization_init_;
  rclcpp::Subscription<RouteState>::SharedPtr sub_route_state_;
  bool flag_localization_initialized;
  bool flag_route_set;

  void onLocalizationInit(const LocalizationInitializationState::ConstSharedPtr msg_ptr)
  {
    flag_localization_initialized = 
      (msg_ptr->state == LocalizationInitializationState::INITIALIZED);
  }
  void onRouteState(const RouteState::ConstSharedPtr msg_ptr)
  {
    flag_route_set = !(msg_ptr->state == RouteState::UNSET);
  }

  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz_common::properties::ColorProperty * property_text_color_;
  rviz_common::properties::IntProperty * property_left_;
  rviz_common::properties::IntProperty * property_top_;
  rviz_common::properties::IntProperty * property_value_height_offset_;
  rviz_common::properties::FloatProperty * property_value_scale_;
  rviz_common::properties::IntProperty * property_font_size_;
  rviz_common::properties::IntProperty * property_max_letter_num_;
  rviz_common::properties::StringProperty * property_localization_state_api_topic_;
  rviz_common::properties::StringProperty * property_routing_state_api_topic_;
  // QImage hud_;

private:
  static constexpr int line_width_ = 2;
  static constexpr int hand_width_ = 4;

  std::mutex mutex_;
  autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr last_msg_ptr_;
};
}  // namespace rviz_plugins

#endif  // MRM_SUMMARY_OVERLAY_DISPLAY_HPP_
