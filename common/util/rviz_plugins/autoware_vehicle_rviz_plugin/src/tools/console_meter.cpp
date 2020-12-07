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

#include "console_meter.hpp"
#include "OGRE/OgreHardwarePixelBuffer.h"
#include "ros/package.h"
#include "rviz/display_context.h"
#include "rviz/uniform_string_stream.h"
#include "QPainter"

namespace rviz_plugins
{
std::unique_ptr<Ogre::ColourValue> ConsoleMeterDisplay::gradation(
  const QColor & color_min, const QColor & color_max, const double ratio)
{
  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue);
  color_ptr->g = color_max.greenF() * ratio + color_min.greenF() * (1.0 - ratio);
  color_ptr->r = color_max.redF() * ratio + color_min.redF() * (1.0 - ratio);
  color_ptr->b = color_max.blueF() * ratio + color_min.blueF() * (1.0 - ratio);

  return color_ptr;
}

std::unique_ptr<Ogre::ColourValue> ConsoleMeterDisplay::setColorDependsOnVelocity(
  const double vel_max, const double cmd_vel)
{
  const double cmd_vel_abs = std::fabs(cmd_vel);
  const double vel_min = 0.0;

  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue());
  if (vel_min < cmd_vel_abs && cmd_vel_abs <= (vel_max / 2.0)) {
    double ratio = (cmd_vel_abs - vel_min) / (vel_max / 2.0 - vel_min);
    color_ptr = gradation(Qt::red, Qt::yellow, ratio);
  } else if ((vel_max / 2.0) < cmd_vel_abs && cmd_vel_abs <= vel_max) {
    double ratio = (cmd_vel_abs - vel_max / 2.0) / (vel_max - vel_max / 2.0);
    color_ptr = gradation(Qt::yellow, Qt::green, ratio);
  } else if (vel_max < cmd_vel_abs) {
    *color_ptr = Ogre::ColourValue::Green;
  } else {
    *color_ptr = Ogre::ColourValue::Red;
  }

  return color_ptr;
}

ConsoleMeterDisplay::ConsoleMeterDisplay()
: meter_min_velocity_(0.0 / 3.6 /* 0.0kmph */),
  meter_max_velocity_(60.0 / 3.6 /* 60.0kmph */),
  meter_min_angle_(0.698132 /* 40deg */),
  meter_max_angle_(5.58505 /* 320deg */)
{
  property_text_color_ = new rviz::ColorProperty(
    "Text Color", QColor(25, 255, 240), "text color", this, SLOT(updateVisualization()), this);
  property_left_ = new rviz::IntProperty(
    "Left", 128, "Left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz::IntProperty(
    "Top", 128, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_length_ = new rviz::IntProperty(
    "Length", 256, "Length of the plotter window", this, SLOT(updateVisualization()), this);
  property_length_->setMin(10);
  property_value_height_offset_ = new rviz::IntProperty(
    "Value height offset", 0, "Height offset of the plotter window", this,
    SLOT(updateVisualization()));
  property_handle_angle_scale_ = new rviz::FloatProperty(
    "Scale", 3.0, "Scale is steering andle to handle angle ", this, SLOT(updateVisualization()),
    this);
  property_handle_angle_scale_->setMin(0.1);
}

ConsoleMeterDisplay::~ConsoleMeterDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void ConsoleMeterDisplay::onInitialize()
{
  MFDClass::onInitialize();
  static int count = 0;
  rviz::UniformStringStream ss;
  ss << "ConsoleMeterDisplayObject" << count++;
  overlay_.reset(new jsk_rviz_plugins::OverlayObject(ss.str()));

  overlay_->show();

  overlay_->updateTextureSize(property_length_->getInt(), property_length_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());

  // QColor background_color;
  // background_color.setAlpha(0);
  // jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  // hud_ = buffer.getQImage(*overlay_);
  // for (int i = 0; i < overlay_->getTextureWidth(); i++)
  // {
  //   for (int j = 0; j < overlay_->getTextureHeight(); j++)
  //   {
  //     hud_.setPixel(i, j, background_color.rgba());
  //   }
  // }
}

void ConsoleMeterDisplay::onEnable()
{
  subscribe();
  overlay_->show();
}

void ConsoleMeterDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();
}

void ConsoleMeterDisplay::processMessage(const geometry_msgs::TwistStampedConstPtr & msg_ptr)
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

  // meter
  QColor white_color(Qt::white);
  white_color.setAlpha(255);
  const double velocity_ratio = std::min(
    std::max(std::fabs(msg_ptr->twist.linear.x) - meter_min_velocity_, 0.0) /
      (meter_max_velocity_ - meter_min_velocity_),
    1.0);
  const double theta =
    (velocity_ratio * (meter_max_angle_ - meter_min_angle_)) + meter_min_angle_ + M_PI_2;
  const double min_range_theta = meter_max_angle_ + M_PI_2;
  const double max_range_theta = meter_min_angle_ + M_PI_2;
  painter.setPen(QPen(white_color, int(4), Qt::SolidLine));
  painter.drawLine(
    w / 2, h / 2, (w / 2) + ((double)w / 2.0) * std::cos(theta),
    (h / 2) + ((double)h / 2.0) * std::sin(theta));
  painter.setPen(QPen(white_color, int(2), Qt::SolidLine));
  painter.drawLine(
    (w / 2) + ((double)w / 8.0) * std::cos(min_range_theta),
    (h / 2) + ((double)h / 8.0) * std::sin(min_range_theta),
    (w / 2) + ((double)w / 2.0) * std::cos(min_range_theta),
    (h / 2) + ((double)h / 2.0) * std::sin(min_range_theta));
  painter.setPen(QPen(white_color, int(2), Qt::SolidLine));
  painter.drawLine(
    (w / 2) + ((double)w / 8.0) * std::cos(max_range_theta),
    (h / 2) + ((double)h / 8.0) * std::sin(max_range_theta),
    (w / 2) + ((double)w / 2.0) * std::cos(max_range_theta),
    (h / 2) + ((double)h / 2.0) * std::sin(max_range_theta));
  painter.drawArc(
    0, 0, w, h, 16 * ((min_range_theta - M_PI) * 180.0 / M_PI),
    16 * ((max_range_theta - min_range_theta) * 180.0 / M_PI));
  painter.drawArc(
    w * 3 / 8, h * 3 / 8, w / 4, h / 4, 16 * ((min_range_theta - M_PI) * 180.0 / M_PI),
    16 * ((max_range_theta - min_range_theta) * 180.0 / M_PI));

  // text
  QColor text_color(property_text_color_->getColor());
  text_color.setAlpha(255);
  painter.setPen(QPen(text_color, int(2), Qt::SolidLine));
  QFont font = painter.font();
  font.setPointSize(std::max(int(double(w) / 15.0), 1));
  font.setBold(true);
  painter.setFont(font);
  std::ostringstream velocity_ss;
  velocity_ss << std::fixed << std::setprecision(2) << msg_ptr->twist.linear.x * 3.6 << "km/h";
  painter.drawText(
    0, std::min(property_value_height_offset_->getInt(), h - 1), w,
    std::max(h - property_value_height_offset_->getInt(), 1), Qt::AlignCenter | Qt::AlignVCenter,
    velocity_ss.str().c_str());
  painter.end();
  last_msg_ptr_ = msg_ptr;
}

void ConsoleMeterDisplay::updateVisualization()
{
  overlay_->updateTextureSize(property_length_->getInt(), property_length_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());

  // QColor background_color;
  // background_color.setAlpha(0);
  // jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  // hud_ = buffer.getQImage(*overlay_);
  // for (int i = 0; i < overlay_->getTextureWidth(); i++)
  // {
  //   for (int j = 0; j < overlay_->getTextureHeight(); j++)
  //   {
  //     hud_.setPixel(i, j, background_color.rgba());
  //   }
  // }

  if (last_msg_ptr_ != nullptr) processMessage(last_msg_ptr_);
}

}  // namespace rviz_plugins

#include "pluginlib/class_list_macros.h"
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ConsoleMeterDisplay, rviz::Display)
