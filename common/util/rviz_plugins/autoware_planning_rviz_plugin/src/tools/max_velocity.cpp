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

#include "max_velocity.hpp"
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <ros/package.h>
#include <rviz/display_context.h>
#include <rviz/uniform_string_stream.h>
#include <QPainter>

namespace rviz_plugins
{
MaxVelocityDisplay::MaxVelocityDisplay()
{
  property_topic_name_ = new rviz::RosTopicProperty(
    "Topic", "/planning/scenario_planning/current_max_velocity", ros::message_traits::datatype<std_msgs::Float32>(), "std_msgs::Float32 topic",
    this, SLOT(updateTopic()));
  property_text_color_ = new rviz::ColorProperty(
    "Text Color", QColor(255, 255, 255), "text color", this, SLOT(updateVisualization()), this);
  property_left_ = new rviz::IntProperty(
    "Left", 128, "Left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz::IntProperty(
    "Top", 128, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_length_ = new rviz::IntProperty(
    "Length", 96, "Length of the plotter window", this, SLOT(updateVisualization()), this);
  property_length_->setMin(10);
  property_value_scale_ = new rviz::FloatProperty(
    "Value Scale", 1.0 / 10.0, "Value scale", this, SLOT(updateVisualization()), this);
  property_value_scale_->setMin(0.01);
}

MaxVelocityDisplay::~MaxVelocityDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void MaxVelocityDisplay::onInitialize()
{
  static int count = 0;
  rviz::UniformStringStream ss;
  ss << "MaxVelocityDisplayObject" << count++;
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

void MaxVelocityDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
}
void MaxVelocityDisplay::subscribe()
{
  std::string topic_name = property_topic_name_->getTopicStd();
  if (topic_name.length() > 0 && topic_name != "/") {
    ros::NodeHandle nh;
    sub_ = nh.subscribe(topic_name, 1, &MaxVelocityDisplay::processMessage, this);
  }
}

void MaxVelocityDisplay::unsubscribe() { sub_.shutdown(); }

void MaxVelocityDisplay::onEnable()
{
  subscribe();
  overlay_->show();
}

void MaxVelocityDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();
}

void MaxVelocityDisplay::processMessage(const std_msgs::Float32ConstPtr & msg_ptr)
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

  const int line_width = property_length_->getInt() / 8;

  int w = overlay_->getTextureWidth() - line_width;
  int h = overlay_->getTextureHeight() - line_width;

  // draw sign
  // QColor white_color(Qt::red);
  // white_color.setAlpha(255);
  // const double min_range_theta = 2.0 * M_PI + M_PI_2;
  // const double max_range_theta = 0.0 + M_PI_2;
  // painter.setPen(QPen(white_color, line_width, Qt::SolidLine));
  // painter.drawLine(
  //   (w / 2) + (line_width * 0.5) + ((double)w / 2.0 - (line_width * 0.5)) * std::cos(M_PI_4),
  //   (h / 2) + (line_width * 0.5) - ((double)h / 2.0 - (line_width * 0.5)) * std::sin(M_PI_4),
  //   (w / 2) + (line_width * 0.5) - ((double)w / 2.0 - (line_width * 0.5)) * std::cos(M_PI_4),
  //   (h / 2) + (line_width * 0.5) + ((double)h / 2.0 - (line_width * 0.5)) * std::sin(M_PI_4));
  // painter.drawArc(
  //   line_width * 0.5, line_width * 0.5, w, h, 16 * ((min_range_theta - M_PI) * 180.0 / M_PI),
  //   16 * ((max_range_theta - min_range_theta) * 180.0 / M_PI));

  // text
  QColor text_color(property_text_color_->getColor());
  text_color.setAlpha(255);
  painter.setPen(QPen(text_color, int(2), Qt::SolidLine));
  QFont font = painter.font();
  font.setPointSize(std::max(int(double(w) * property_value_scale_->getFloat()), 1));
  font.setBold(true);
  painter.setFont(font);
  std::ostringstream velocity_ss;
  velocity_ss << std::fixed << std::setprecision(0) << "limited" << std::endl
              << msg_ptr->data * 3.6 << "km/h";
  painter.drawText(
    (int)(line_width * 0.5), std::min((int)(line_width * 0.5), h - 1), w, std::max(h, 1),
    Qt::AlignCenter | Qt::AlignVCenter, velocity_ss.str().c_str());
  painter.end();
  last_msg_ptr_ = msg_ptr;
}

void MaxVelocityDisplay::updateVisualization()
{
  overlay_->updateTextureSize(property_length_->getInt(), property_length_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());

  if (last_msg_ptr_ != nullptr) processMessage(last_msg_ptr_);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::MaxVelocityDisplay, rviz::Display)
