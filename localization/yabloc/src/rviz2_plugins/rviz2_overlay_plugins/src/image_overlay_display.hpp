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

#ifndef IMAGE_OVERLAY_DISPLAY_HPP_
#define IMAGE_OVERLAY_DISPLAY_HPP_

#include <memory>
#include <mutex>

#ifndef Q_MOC_RUN
#include "jsk_overlay_utils.hpp"

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <image_transport/image_transport.hpp>
#endif

namespace rviz_plugins
{
class ImageOverlayDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  ImageOverlayDisplay();
  ~ImageOverlayDisplay() override;

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;
  void subscribe();
  void unsubscribe();

private Q_SLOTS:
  void updateVisualization();

protected:
  void update(float wall_dt, float ros_dt) override;
  void processMessage(const sensor_msgs::msg::Image::ConstSharedPtr msg_ptr);
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz_common::properties::IntProperty* property_left_;
  rviz_common::properties::IntProperty* property_top_;
  rviz_common::properties::IntProperty* property_width_;
  rviz_common::properties::IntProperty* property_height_;
  rviz_common::properties::StringProperty* property_topic_name_;
  rviz_common::properties::FloatProperty* property_alpha_;
  rviz_common::properties::BoolProperty* property_image_type_;

private:
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<image_transport::Subscriber> sub_;
  sensor_msgs::msg::Image::ConstSharedPtr last_msg_ptr_;
  std::string topic_name_;
  bool update_required_ = true;
};

}  // namespace rviz_plugins

#endif  // IMAGE_OVERLAY_DISPLAY_HPP_
