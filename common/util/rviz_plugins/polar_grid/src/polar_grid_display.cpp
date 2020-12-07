/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>

#include <math.h>
#include "boost/bind.hpp"

#include "OgreSceneManager.h"
#include "OgreSceneNode.h"

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/parse_color.h"
#include "rviz/properties/property.h"
#include "rviz/selection/selection_manager.h"

#include "polar_grid_display.hpp"

namespace rviz
{
PolarGridDisplay::PolarGridDisplay() : Display(), wave_range_(0.0)
{
  frame_property_ = new TfFrameProperty(
    "Reference Frame", TfFrameProperty::FIXED_FRAME_STRING,
    "The TF frame this grid will use for its origin.", this, 0, true);

  color_property_ = new ColorProperty(
    "Color", Qt::white, "The color of the grid lines.", this, SLOT(updatePlane()));

  d_range_property_ =
    new FloatProperty("Delta Range", 10.0f, "Delta Range[m].", this, SLOT(updatePlane()));
  d_range_property_->setMin(0.1f);
  d_range_property_->setMax(100.0f);

  max_range_property_ =
    new FloatProperty("Max Range", 200.0f, "Max Range[m].", this, SLOT(updatePlane()));
  max_range_property_->setMin(0.0f);
  max_range_property_->setMax(500.0f);
  max_alpha_property_ = new FloatProperty(
    "Max Alpha", 1.0f, "The amount of transparency to apply to the grid lines.", this,
    SLOT(updatePlane()));
  max_alpha_property_->setMin(0.0f);
  max_alpha_property_->setMax(1.0f);
  min_alpha_property_ = new FloatProperty(
    "Min Alpha", 0.2f, "The amount of transparency to apply to the grid lines.", this,
    SLOT(updatePlane()));
  min_alpha_property_->setMin(0.0f);
  min_alpha_property_->setMax(1.0f);
  wave_velocity_property_ =
    new FloatProperty("Wave Velocity", 100.0f, "Wave Velocity [m/s]", this, SLOT(updatePlane()));
  wave_velocity_property_->setMin(0.0f);
  wave_color_property_ = new ColorProperty(
    "Wave Color", Qt::white, "The color of the grid lines.", this, SLOT(updatePlane()));

  max_wave_alpha_property_ = new FloatProperty(
    "Max Wave Alpha", 1.0f, "The amount of transparency to apply to the grid lines.", this,
    SLOT(updatePlane()));
  max_wave_alpha_property_->setMin(0.0f);
  max_wave_alpha_property_->setMax(1.0f);
  min_wave_alpha_property_ = new FloatProperty(
    "Min Wave Alpha", 0.2f, "The amount of transparency to apply to the grid lines.", this,
    SLOT(updatePlane()));
  min_wave_alpha_property_->setMin(0.0f);
  min_wave_alpha_property_->setMax(1.0f);
}

PolarGridDisplay::~PolarGridDisplay()
{
  if (initialized()) {
    scene_manager_->destroyManualObject(rings_manual_object_);
    scene_manager_->destroyManualObject(wave_manual_object_);
  }
}

void PolarGridDisplay::onInitialize()
{
  rings_manual_object_ = scene_manager_->createManualObject();
  rings_manual_object_->setDynamic(true);
  scene_node_->attachObject(rings_manual_object_);
  wave_manual_object_ = scene_manager_->createManualObject();
  wave_manual_object_->setDynamic(true);
  scene_node_->attachObject(wave_manual_object_);

  frame_property_->setFrameManager(context_->getFrameManager());
  updatePlane();
}

void PolarGridDisplay::reset()
{
  rings_manual_object_->clear();
  wave_manual_object_->clear();
}

void PolarGridDisplay::update(float /*dt*/, float ros_dt)
{
  QString qframe = frame_property_->getFrame();
  std::string frame = qframe.toStdString();

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (context_->getFrameManager()->getTransform(frame, ros::Time(), position, orientation)) {
    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");
  } else {
    std::string error;
    if (context_->getFrameManager()->transformHasProblems(frame, ros::Time(), error)) {
      setStatus(StatusProperty::Error, "Transform", QString::fromStdString(error));
    } else {
      setStatus(
        StatusProperty::Error, "Transform",
        "Could not transform from [" + qframe + "] to [" + fixed_frame_ + "]");
    }
  }
  wave_range_ += ros_dt * wave_velocity_property_->getFloat();
  wave_range_ = std::fmod(wave_range_, max_range_property_->getFloat());
  wave_manual_object_->clear();
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(
    "BaseWhiteNoLighting", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setDepthWriteEnabled(false);
  const float d_theta = 3.6 * M_PI / 180.0;
  const float d_alpha = std::min(
    std::max(
      (float)max_wave_alpha_property_->getFloat() - (float)min_wave_alpha_property_->getFloat(),
      float(0.0)),
    float(1.0));
  Ogre::ColourValue color;
  color = rviz::qtToOgre(wave_color_property_->getColor());
  color.a = max_wave_alpha_property_->getFloat();
  wave_manual_object_->estimateVertexCount(int(2 * M_PI / d_theta));
  wave_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

  color.a = d_alpha * (1.0 - (wave_range_ / max_range_property_->getFloat())) +
            min_wave_alpha_property_->getFloat();
  color.a = std::max(color.a, min_wave_alpha_property_->getFloat());
  for (float theta = 0.0; theta < 2.0 * M_PI + d_theta; theta += d_theta) {
    wave_manual_object_->position(
      wave_range_ * (float)std::cos(theta), wave_range_ * (float)std::sin(theta), 0.0);
    wave_manual_object_->colour(color);
  }
  wave_manual_object_->end();

  context_->queueRender();
}

void PolarGridDisplay::updatePlane()
{
  rings_manual_object_->clear();

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(
    "BaseWhiteNoLighting", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setDepthWriteEnabled(false);

  const float d_theta = 3.6 * M_PI / 180.0;
  const float d_range = d_range_property_->getFloat();
  const float d_alpha = std::min(
    std::max(
      (float)max_alpha_property_->getFloat() - (float)min_alpha_property_->getFloat(), float(0.0)),
    float(1.0));
  const float epsilon = 0.001;
  const float max_range = max_range_property_->getFloat() + epsilon;
  Ogre::ColourValue color;
  color = rviz::qtToOgre(color_property_->getColor());
  color.a = max_alpha_property_->getFloat();
  rings_manual_object_->estimateVertexCount(
    int(max_range / d_range) + int(float(2.0) * M_PI / d_theta));
  rings_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
  for (int r = d_range; r <= max_range; r += d_range) {
    for (float theta = 0.0; theta < 2 * M_PI; theta += d_theta) {
      {
        rings_manual_object_->position((float)r * std::cos(theta), (float)r * std::sin(theta), 0.0);
        rings_manual_object_->colour(color);
        rings_manual_object_->position(
          (float)r * std::cos(theta + d_theta), (float)r * std::sin(theta + d_theta), 0.0);
        rings_manual_object_->colour(color);
      }
    }
    color.a -= d_alpha / (max_range / d_range);
    color.a = std::max(color.a, min_alpha_property_->getFloat());
  }

  rings_manual_object_->end();

  context_->queueRender();
}

}  // namespace rviz

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rviz::PolarGridDisplay, rviz::Display)
