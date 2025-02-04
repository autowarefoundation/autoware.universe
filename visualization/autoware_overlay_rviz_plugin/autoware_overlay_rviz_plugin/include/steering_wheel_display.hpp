// Copyright 2024 The Autoware Contributors
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

#ifndef STEERING_WHEEL_DISPLAY_HPP_
#define STEERING_WHEEL_DISPLAY_HPP_
#include "overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include "autoware_vehicle_msgs/msg/steering_report.hpp"

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace autoware_overlay_rviz_plugin
{

class SteeringWheelDisplay
{
public:
  SteeringWheelDisplay();
  void drawSteeringWheel(
    QPainter & painter, const QRectF & backgroundRect, float handle_angle_scale_);
  void updateSteeringData(const autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr & msg);

private:
  float steering_angle_ = 0.0f;
  QColor gray = QColor(194, 194, 194);

  QImage wheelImage;
  QImage scaledWheelImage;
  QImage coloredImage(const QImage & source, const QColor & color);
  autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr last_msg_ptr_;
};

}  // namespace autoware_overlay_rviz_plugin

#endif  // STEERING_WHEEL_DISPLAY_HPP_
