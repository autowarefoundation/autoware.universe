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

#include "turn_signal.hpp"
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <ros/package.h>
#include <rviz/display_context.h>
#include <rviz/uniform_string_stream.h>
#include <QPainter>

namespace rviz_plugins
{
TurnSignalDisplay::TurnSignalDisplay()
{
  property_left_ = new rviz::IntProperty(
    "Left", 128, "Left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz::IntProperty(
    "Top", 128, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_width_ = new rviz::IntProperty(
    "Width", 256, "Width of the plotter window", this, SLOT(updateVisualization()), this);
  property_width_->setMin(10);
  property_height_ = new rviz::IntProperty(
    "Height", 256, "Width of the plotter window", this, SLOT(updateVisualization()), this);
  property_height_->setMin(10);
}

TurnSignalDisplay::~TurnSignalDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void TurnSignalDisplay::onInitialize()
{
  MFDClass::onInitialize();
  static int count = 0;
  rviz::UniformStringStream ss;
  ss << "TurnSignalDisplayObject" << count++;
  overlay_.reset(new jsk_rviz_plugins::OverlayObject(ss.str()));

  overlay_->show();

  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
}

void TurnSignalDisplay::onEnable()
{
  subscribe();
  overlay_->show();
}

void TurnSignalDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();
}

void TurnSignalDisplay::processMessage(const autoware_vehicle_msgs::TurnSignalConstPtr & msg_ptr)
{
  if (!isEnabled()) {
    return;
  }
  if (!overlay_->isVisible()) {
    return;
  }

  QColor background_color;
  background_color.setAlpha(0);
  jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(background_color);

  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);

  int w = overlay_->getTextureWidth();
  int h = overlay_->getTextureHeight();

  // turn signal color
  QColor white_color(Qt::white);
  white_color.setAlpha(255);
  if (msg_ptr->data == autoware_vehicle_msgs::TurnSignal::RIGHT) {
    QPointF * line = new QPointF[7];
    painter.setPen(QPen(white_color, int(2), Qt::DotLine));
    line[0].setX((double)w * 0.0 / 5.0);
    line[0].setY((double)h * 1.0 / 2.0);
    line[1].setX((double)w * 1.0 / 5.0);
    line[1].setY((double)h * 1.0 / 5.0);
    line[2].setX((double)w * 1.0 / 5.0);
    line[2].setY((double)h * 2.0 / 5.0);
    line[3].setX((double)w * 2.0 / 5.0);
    line[3].setY((double)h * 2.0 / 5.0);
    line[4].setX((double)w * 2.0 / 5.0);
    line[4].setY((double)h * 3.0 / 5.0);
    line[5].setX((double)w * 1.0 / 5.0);
    line[5].setY((double)h * 3.0 / 5.0);
    line[6].setX((double)w * 1.0 / 5.0);
    line[6].setY((double)h * 4.0 / 5.0);
    painter.drawPolygon(line, 7);
    painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));
    painter.setPen(QPen(white_color, int(2), Qt::SolidLine));
    line[0].setX((double)w * 5.0 / 5.0);
    line[0].setY((double)h * 1.0 / 2.0);
    line[1].setX((double)w * 4.0 / 5.0);
    line[1].setY((double)h * 1.0 / 5.0);
    line[2].setX((double)w * 4.0 / 5.0);
    line[2].setY((double)h * 2.0 / 5.0);
    line[3].setX((double)w * 3.0 / 5.0);
    line[3].setY((double)h * 2.0 / 5.0);
    line[4].setX((double)w * 3.0 / 5.0);
    line[4].setY((double)h * 3.0 / 5.0);
    line[5].setX((double)w * 4.0 / 5.0);
    line[5].setY((double)h * 3.0 / 5.0);
    line[6].setX((double)w * 4.0 / 5.0);
    line[6].setY((double)h * 4.0 / 5.0);
    painter.drawPolygon(line, 7);
  } else if (msg_ptr->data == autoware_vehicle_msgs::TurnSignal::LEFT) {
    QPointF * line = new QPointF[7];
    painter.setPen(QPen(white_color, int(2), Qt::DotLine));
    line[0].setX((double)w * 5.0 / 5.0);
    line[0].setY((double)h * 1.0 / 2.0);
    line[1].setX((double)w * 4.0 / 5.0);
    line[1].setY((double)h * 1.0 / 5.0);
    line[2].setX((double)w * 4.0 / 5.0);
    line[2].setY((double)h * 2.0 / 5.0);
    line[3].setX((double)w * 3.0 / 5.0);
    line[3].setY((double)h * 2.0 / 5.0);
    line[4].setX((double)w * 3.0 / 5.0);
    line[4].setY((double)h * 3.0 / 5.0);
    line[5].setX((double)w * 4.0 / 5.0);
    line[5].setY((double)h * 3.0 / 5.0);
    line[6].setX((double)w * 4.0 / 5.0);
    line[6].setY((double)h * 4.0 / 5.0);
    painter.drawPolygon(line, 7);
    painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));
    painter.setPen(QPen(white_color, int(2), Qt::SolidLine));
    line[0].setX((double)w * 0.0 / 5.0);
    line[0].setY((double)h * 1.0 / 2.0);
    line[1].setX((double)w * 1.0 / 5.0);
    line[1].setY((double)h * 1.0 / 5.0);
    line[2].setX((double)w * 1.0 / 5.0);
    line[2].setY((double)h * 2.0 / 5.0);
    line[3].setX((double)w * 2.0 / 5.0);
    line[3].setY((double)h * 2.0 / 5.0);
    line[4].setX((double)w * 2.0 / 5.0);
    line[4].setY((double)h * 3.0 / 5.0);
    line[5].setX((double)w * 1.0 / 5.0);
    line[5].setY((double)h * 3.0 / 5.0);
    line[6].setX((double)w * 1.0 / 5.0);
    line[6].setY((double)h * 4.0 / 5.0);
    painter.drawPolygon(line, 7);
  } else if (msg_ptr->data == autoware_vehicle_msgs::TurnSignal::HAZARD) {
    painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));
    painter.setPen(QPen(white_color, int(2), Qt::SolidLine));
    QPointF * line = new QPointF[7];
    line[0].setX((double)w * 5.0 / 5.0);
    line[0].setY((double)h * 1.0 / 2.0);
    line[1].setX((double)w * 4.0 / 5.0);
    line[1].setY((double)h * 1.0 / 5.0);
    line[2].setX((double)w * 4.0 / 5.0);
    line[2].setY((double)h * 2.0 / 5.0);
    line[3].setX((double)w * 3.0 / 5.0);
    line[3].setY((double)h * 2.0 / 5.0);
    line[4].setX((double)w * 3.0 / 5.0);
    line[4].setY((double)h * 3.0 / 5.0);
    line[5].setX((double)w * 4.0 / 5.0);
    line[5].setY((double)h * 3.0 / 5.0);
    line[6].setX((double)w * 4.0 / 5.0);
    line[6].setY((double)h * 4.0 / 5.0);
    painter.drawPolygon(line, 7);
    line[0].setX((double)w * 0.0 / 5.0);
    line[0].setY((double)h * 1.0 / 2.0);
    line[1].setX((double)w * 1.0 / 5.0);
    line[1].setY((double)h * 1.0 / 5.0);
    line[2].setX((double)w * 1.0 / 5.0);
    line[2].setY((double)h * 2.0 / 5.0);
    line[3].setX((double)w * 2.0 / 5.0);
    line[3].setY((double)h * 2.0 / 5.0);
    line[4].setX((double)w * 2.0 / 5.0);
    line[4].setY((double)h * 3.0 / 5.0);
    line[5].setX((double)w * 1.0 / 5.0);
    line[5].setY((double)h * 3.0 / 5.0);
    line[6].setX((double)w * 1.0 / 5.0);
    line[6].setY((double)h * 4.0 / 5.0);
    painter.drawPolygon(line, 7);
  } else {
    painter.setPen(QPen(white_color, int(2), Qt::DotLine));
    QPointF * line = new QPointF[7];
    line[0].setX((double)w * 5.0 / 5.0);
    line[0].setY((double)h * 1.0 / 2.0);
    line[1].setX((double)w * 4.0 / 5.0);
    line[1].setY((double)h * 1.0 / 5.0);
    line[2].setX((double)w * 4.0 / 5.0);
    line[2].setY((double)h * 2.0 / 5.0);
    line[3].setX((double)w * 3.0 / 5.0);
    line[3].setY((double)h * 2.0 / 5.0);
    line[4].setX((double)w * 3.0 / 5.0);
    line[4].setY((double)h * 3.0 / 5.0);
    line[5].setX((double)w * 4.0 / 5.0);
    line[5].setY((double)h * 3.0 / 5.0);
    line[6].setX((double)w * 4.0 / 5.0);
    line[6].setY((double)h * 4.0 / 5.0);
    painter.drawPolygon(line, 7);
    line[0].setX((double)w * 0.0 / 5.0);
    line[0].setY((double)h * 1.0 / 2.0);
    line[1].setX((double)w * 1.0 / 5.0);
    line[1].setY((double)h * 1.0 / 5.0);
    line[2].setX((double)w * 1.0 / 5.0);
    line[2].setY((double)h * 2.0 / 5.0);
    line[3].setX((double)w * 2.0 / 5.0);
    line[3].setY((double)h * 2.0 / 5.0);
    line[4].setX((double)w * 2.0 / 5.0);
    line[4].setY((double)h * 3.0 / 5.0);
    line[5].setX((double)w * 1.0 / 5.0);
    line[5].setY((double)h * 3.0 / 5.0);
    line[6].setX((double)w * 1.0 / 5.0);
    line[6].setY((double)h * 4.0 / 5.0);
    painter.drawPolygon(line, 7);
  }
  painter.end();
  last_msg_ptr_ = msg_ptr;
}

void TurnSignalDisplay::updateVisualization()
{
  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());

  if (last_msg_ptr_ != nullptr) processMessage(last_msg_ptr_);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::TurnSignalDisplay, rviz::Display)
