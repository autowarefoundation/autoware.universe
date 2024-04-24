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

#include "mission_details_display.hpp"

#include <QFontDatabase>
#include <QPainter>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_rendering/render_system.hpp>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>

namespace autoware_mission_details_overlay_rviz_plugin
{

MissionDetailsDisplay::MissionDetailsDisplay()
{
  property_width_ = new rviz_common::properties::IntProperty(
    "Width", 300, "Width of the overlay", this, SLOT(updateOverlaySize()));
  property_height_ = new rviz_common::properties::IntProperty(
    "Height", 100, "Height of the overlay", this, SLOT(updateOverlaySize()));
  property_left_ = new rviz_common::properties::IntProperty(
    "Left", 10, "Left position of the overlay", this, SLOT(updateOverlayPosition()));
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", 10, "Top position of the overlay", this, SLOT(updateOverlayPosition()));

  // Initialize the component displays
  remaining_distance_time_display_ = std::make_unique<RemainingDistanceTimeDisplay>();
}

void MissionDetailsDisplay::onInitialize()
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  rviz_common::Display::onInitialize();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  static int count = 0;
  std::stringstream ss;
  ss << "MissionDetailsDisplay" << count++;
  overlay_.reset(new autoware_mission_details_overlay_rviz_plugin::OverlayObject(ss.str()));
  overlay_->show();
  updateOverlaySize();
  updateOverlayPosition();

  auto rviz_ros_node = context_->getRosNodeAbstraction();

  remaining_distance_time_topic_property_ =
    std::make_unique<rviz_common::properties::RosTopicProperty>(
      "Remaining Distance and Time Topic", "/planning/mission_remaining_distance_time",
      "autoware_internal_msgs/msg/MissionRemainingDistanceTime",
      "Topic for Mission Remaining Distance and Time Data", this,
      SLOT(topic_updated_remaining_distance_time()));
  remaining_distance_time_topic_property_->initialize(rviz_ros_node);
}

void MissionDetailsDisplay::setupRosSubscriptions()
{
  // Don't create a node, just use the one from the parent class
  auto rviz_node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  remaining_distance_time_sub_ =
    rviz_node_->create_subscription<autoware_internal_msgs::msg::MissionRemainingDistanceTime>(
      "/planning/mission_remaining_distance_time",
      rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
      [this](const autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg) {
        updateRemainingDistanceTimeData(msg);
      });
}

MissionDetailsDisplay::~MissionDetailsDisplay()
{
  std::lock_guard<std::mutex> lock(property_mutex_);
  overlay_.reset();

  remaining_distance_time_sub_.reset();

  remaining_distance_time_display_.reset();

  remaining_distance_time_topic_property_.reset();
}

void MissionDetailsDisplay::update(float /* wall_dt */, float /* ros_dt */)
{
  if (!overlay_) {
    return;
  }
  autoware_mission_details_overlay_rviz_plugin::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(Qt::transparent);
  drawWidget(hud);
}

void MissionDetailsDisplay::onEnable()
{
  std::lock_guard<std::mutex> lock(property_mutex_);
  if (overlay_) {
    overlay_->show();
  }
  setupRosSubscriptions();
}

void MissionDetailsDisplay::onDisable()
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  remaining_distance_time_sub_.reset();

  if (overlay_) {
    overlay_->hide();
  }
}

void MissionDetailsDisplay::updateRemainingDistanceTimeData(
  const autoware_internal_msgs::msg::MissionRemainingDistanceTime::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (remaining_distance_time_display_) {
    remaining_distance_time_display_->updateRemainingDistanceTimeData(msg);
    queueRender();
  }
}

void MissionDetailsDisplay::drawWidget(QImage & hud)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (!overlay_->isVisible()) {
    return;
  }

  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);

  QRectF backgroundRect(0, 0, 300, 100);
  drawHorizontalRoundedRectangle(painter, backgroundRect);

  if (remaining_distance_time_display_) {
    remaining_distance_time_display_->drawRemainingDistanceTimeDisplay(painter, backgroundRect);
  }

  painter.end();
}

void MissionDetailsDisplay::drawHorizontalRoundedRectangle(
  QPainter & painter, const QRectF & backgroundRect)
{
  painter.setRenderHint(QPainter::Antialiasing, true);
  QColor colorFromHSV;
  colorFromHSV.setHsv(0, 0, 0);  // Hue, Saturation, Value
  colorFromHSV.setAlphaF(0.65);  // Transparency

  painter.setBrush(colorFromHSV);

  painter.setPen(Qt::NoPen);
  painter.drawRoundedRect(
    backgroundRect, backgroundRect.height() / 2, backgroundRect.height() / 2);  // Circular ends
}
void MissionDetailsDisplay::drawVerticalRoundedRectangle(
  QPainter & painter, const QRectF & backgroundRect)
{
  painter.setRenderHint(QPainter::Antialiasing, true);
  QColor colorFromHSV;
  colorFromHSV.setHsv(0, 0, 0);  // Hue, Saturation, Value
  colorFromHSV.setAlphaF(0.65);  // Transparency

  painter.setBrush(colorFromHSV);

  painter.setPen(Qt::NoPen);
  painter.drawRoundedRect(
    backgroundRect, backgroundRect.width() / 2, backgroundRect.width() / 2);  // Circular ends
}

void MissionDetailsDisplay::reset()
{
  rviz_common::Display::reset();
  overlay_->hide();
}

void MissionDetailsDisplay::updateOverlaySize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  queueRender();
}

void MissionDetailsDisplay::updateOverlayPosition()
{
  std::lock_guard<std::mutex> lock(mutex_);
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  queueRender();
}

void MissionDetailsDisplay::updateOverlayColor()
{
  std::lock_guard<std::mutex> lock(mutex_);
  queueRender();
}

void MissionDetailsDisplay::topic_updated_remaining_distance_time()
{
  // resubscribe to the topic
  remaining_distance_time_sub_.reset();
  auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
  remaining_distance_time_sub_ =
    rviz_ros_node->get_raw_node()
      ->create_subscription<autoware_internal_msgs::msg::MissionRemainingDistanceTime>(
        remaining_distance_time_topic_property_->getTopicStd(),
        rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
        [this](const autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg) {
          updateRemainingDistanceTimeData(msg);
        });
}

}  // namespace autoware_mission_details_overlay_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware_mission_details_overlay_rviz_plugin::MissionDetailsDisplay, rviz_common::Display)
