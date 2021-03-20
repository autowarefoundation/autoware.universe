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

#include <memory>
#include <string>
#include <algorithm>

#include "steering_angle.hpp"
#include "QPainter"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rviz_common/uniform_string_stream.hpp"

namespace rviz_plugins
{
std::unique_ptr<Ogre::ColourValue> SteeringAngleDisplay::gradation(
  const QColor & color_min, const QColor & color_max, const double ratio)
{
  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue);
  color_ptr->g = color_max.greenF() * ratio + color_min.greenF() * (1.0 - ratio);
  color_ptr->r = color_max.redF() * ratio + color_min.redF() * (1.0 - ratio);
  color_ptr->b = color_max.blueF() * ratio + color_min.blueF() * (1.0 - ratio);

  return color_ptr;
}

std::unique_ptr<Ogre::ColourValue> SteeringAngleDisplay::setColorDependsOnVelocity(
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

SteeringAngleDisplay::SteeringAngleDisplay()
: handle_image_(
    std::string(ament_index_cpp::get_package_share_directory("autoware_vehicle_rviz_plugin") +
    "/images/handle.png").c_str())
{
  property_text_color_ = new rviz_common::properties::ColorProperty(
    "Text Color", QColor(25, 255, 240), "text color", this, SLOT(updateVisualization()), this);
  property_left_ = new rviz_common::properties::IntProperty(
    "Left", 128, "Left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", 128, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_length_ = new rviz_common::properties::IntProperty(
    "Length", 256, "Length of the plotter window", this, SLOT(updateVisualization()), this);
  property_length_->setMin(10);
  property_value_height_offset_ = new rviz_common::properties::IntProperty(
    "Value height offset", 0, "Height offset of the plotter window", this,
    SLOT(updateVisualization()));
  property_value_scale_ = new rviz_common::properties::FloatProperty(
    "Value Scale", 1.0 / 6.667, "Value scale", this, SLOT(updateVisualization()), this);
  property_value_scale_->setMin(0.01);
  property_handle_angle_scale_ = new rviz_common::properties::FloatProperty(
    "Scale", 3.0, "Scale is steering angle to handle angle ", this, SLOT(updateVisualization()),
    this);
  property_handle_angle_scale_->setMin(0.1);
}

SteeringAngleDisplay::~SteeringAngleDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void SteeringAngleDisplay::onInitialize()
{
  RTDClass::onInitialize();
  static int count = 0;
  rviz_common::UniformStringStream ss;
  ss << "SteeringAngleDisplayObject" << count++;
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

void SteeringAngleDisplay::onEnable()
{
  subscribe();
  overlay_->show();
}

void SteeringAngleDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();
}

void SteeringAngleDisplay::processMessage(
  const autoware_vehicle_msgs::msg::Steering::ConstSharedPtr msg_ptr)
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
  QColor text_color(property_text_color_->getColor());
  text_color.setAlpha(255);
  painter.setPen(QPen(text_color, static_cast<int>(2), Qt::SolidLine));

  const int w = overlay_->getTextureWidth();
  const int h = overlay_->getTextureHeight();

  QMatrix rotation_matrix;
  rotation_matrix.rotate(
    std::round(property_handle_angle_scale_->getFloat() * (msg_ptr->data / M_PI) * -180.0));
  // else
  // rotation_matrix.rotate
  // ((property_handle_angle_scale_->getFloat() * (msg_ptr->data / M_PI) * -180.0));
  int handle_image_width = handle_image_.width(), handle_image_height = handle_image_.height();
  QPixmap rotate_handle_image;
  rotate_handle_image = handle_image_.transformed(rotation_matrix);
  rotate_handle_image = rotate_handle_image.copy(
    (rotate_handle_image.width() - handle_image_width) / 2,
    (rotate_handle_image.height() - handle_image_height) / 2, handle_image_width,
    handle_image_height);
  painter.drawPixmap(
    0, 0, property_length_->getInt(), property_length_->getInt(), rotate_handle_image);

  QFont font = painter.font();
  font.setPixelSize(
    std::max(static_cast<int>((static_cast<double>(w)) * property_value_scale_->getFloat()), 1));
  font.setBold(true);
  painter.setFont(font);
  std::ostringstream steering_angle_ss;
  steering_angle_ss << std::fixed << std::setprecision(1) << msg_ptr->data * 180.0 / M_PI << "deg";
  painter.drawText(
    0, std::min(property_value_height_offset_->getInt(), h - 1), w,
    std::max(h - property_value_height_offset_->getInt(), 1), Qt::AlignCenter | Qt::AlignVCenter,
    steering_angle_ss.str().c_str());

  painter.end();
  last_msg_ptr_ = msg_ptr;
  updateVisualization();
}

void SteeringAngleDisplay::updateVisualization()
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
}

}  // namespace rviz_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rviz_plugins::SteeringAngleDisplay, rviz_common::Display)
