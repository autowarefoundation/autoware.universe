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

#include "velocity_history.hpp"
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rviz_plugins
{
std::unique_ptr<Ogre::ColourValue> VelocityHistoryDisplay::gradation(
  const QColor & color_min, const QColor & color_max, const double ratio)
{
  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue);
  color_ptr->g = color_max.greenF() * ratio + color_min.greenF() * (1.0 - ratio);
  color_ptr->r = color_max.redF() * ratio + color_min.redF() * (1.0 - ratio);
  color_ptr->b = color_max.blueF() * ratio + color_min.blueF() * (1.0 - ratio);

  return color_ptr;
}

std::unique_ptr<Ogre::ColourValue> VelocityHistoryDisplay::setColorDependsOnVelocity(
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

VelocityHistoryDisplay::VelocityHistoryDisplay()
{
  property_velocity_timeout_ =
    new rviz::FloatProperty("Timeout", 10.0, "", this, SLOT(updateVisualization()), this);
  property_velocity_timeout_->setMin(0.0);
  property_velocity_timeout_->setMax(100000.0);
  property_velocity_alpha_ =
    new rviz::FloatProperty("Alpha", 1.0, "", this, SLOT(updateVisualization()), this);
  property_velocity_alpha_->setMin(0.0);
  property_velocity_alpha_->setMax(1.0);
  property_velocity_scale_ =
    new rviz::FloatProperty("Scale", 0.3, "", this, SLOT(updateVisualization()), this);
  property_velocity_scale_->setMin(0.1);
  property_velocity_scale_->setMax(10.0);
  property_velocity_color_view_ =
    new rviz::BoolProperty("Constant Color", false, "", this, SLOT(updateVisualization()), this);
  property_velocity_color_ = new rviz::ColorProperty(
    "Color", Qt::black, "", property_velocity_color_view_, SLOT(updateVisualization()), this);
  property_vel_max_ = new rviz::FloatProperty(
    "Color Border Vel Max", 3.0, "[m/s]", this, SLOT(updateVisualization()), this);
  property_vel_max_->setMin(0.0);
}

VelocityHistoryDisplay::~VelocityHistoryDisplay()
{
  if (initialized()) {
    scene_manager_->destroyManualObject(velocity_manual_object_);
  }
}

void VelocityHistoryDisplay::onInitialize()
{
  MFDClass::onInitialize();

  velocity_manual_object_ = scene_manager_->createManualObject();
  velocity_manual_object_->setDynamic(true);
  scene_node_->attachObject(velocity_manual_object_);
}

void VelocityHistoryDisplay::reset()
{
  MFDClass::reset();
  velocity_manual_object_->clear();
}

bool VelocityHistoryDisplay::validateFloats(const geometry_msgs::TwistStampedConstPtr & msg_ptr)
{
  if (!rviz::validateFloats(msg_ptr->twist.linear.x)) return false;

  return true;
}

void VelocityHistoryDisplay::processMessage(const geometry_msgs::TwistStampedConstPtr & msg_ptr)
{
  if (!validateFloats(msg_ptr)) {
    setStatus(
      rviz::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  std_msgs::Header header;
  header = msg_ptr->header;
  header.frame_id = "base_link";
  if (!context_->getFrameManager()->getTransform(header, position, orientation)) {
    ROS_DEBUG(
      "Error transforming from frame '%s' to frame '%s'", header.frame_id.c_str(),
      qPrintable(fixed_frame_));
  }

  history_.push_back(std::make_tuple(msg_ptr, position));
  updateVisualization();
}

void VelocityHistoryDisplay::updateVisualization()
{
  if (history_.empty()) return;
  velocity_manual_object_->clear();

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(
    "BaseWhiteNoLighting", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setDepthWriteEnabled(false);
  ros::Time current_time = ros::Time::now();

  while (!history_.empty()) {
    if (
      property_velocity_timeout_->getFloat() <
      (current_time - std::get<0>(history_.front())->header.stamp).toSec())
      history_.pop_front();
    else
      break;
  }

  // std::cout << __LINE__ << ":" <<std::get<1>(history_.front()) <<std::endl;
  velocity_manual_object_->estimateVertexCount(history_.size());
  velocity_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);

  for (size_t i = 0; i < history_.size(); ++i) {
    Ogre::ColourValue color;
    if (property_velocity_color_view_->getBool()) {
      color = rviz::qtToOgre(property_velocity_color_->getColor());
    } else {
      /* color change depending on velocity */
      std::unique_ptr<Ogre::ColourValue> dynamic_color_ptr = setColorDependsOnVelocity(
        property_vel_max_->getFloat(), std::get<0>(history_.at(i))->twist.linear.x);
      color = *dynamic_color_ptr;
    }
    color.a = 1.0 - (current_time - std::get<0>(history_.at(i))->header.stamp).toSec() /
                      property_velocity_timeout_->getFloat();
    color.a = std::min(std::max(color.a, float(0.0)), float(1.0));
    // std::cout << __LINE__ << ":" <<std::get<1>(history_.front()) <<std::endl;

    // color.a = property_velocity_alpha_->getFloat();
    velocity_manual_object_->position(
      std::get<1>(history_.at(i)).x, std::get<1>(history_.at(i)).y,
      std::get<1>(history_.at(i)).z +
        std::get<0>(history_.at(i))->twist.linear.x * property_velocity_scale_->getFloat());
    velocity_manual_object_->colour(color);
  }
  velocity_manual_object_->end();
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::VelocityHistoryDisplay, rviz::Display)
