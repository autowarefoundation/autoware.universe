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

#pragma once

#include "OgreBillboardSet.h"
#include "OgreManualObject.h"
#include "OgreSceneManager.h"
#include "OgreSceneNode.h"
#include "rviz/display_context.h"
#include "rviz/display.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/properties/vector_property.h"

namespace rviz
{
/**
 * \class PolarGridDisplay
 * \brief Displays a grid in either the XY, YZ, or XZ plane.
 *
 * For more information see Grid
 */
class PolarGridDisplay : public Display
{
  Q_OBJECT
public:
  PolarGridDisplay();
  virtual ~PolarGridDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update(float dt, float ros_dt);
  void reset() override;

private Q_SLOTS:
  void updatePlane();

private:
  Ogre::ManualObject * rings_manual_object_;
  Ogre::ManualObject * wave_manual_object_;
  float wave_range_;

  TfFrameProperty * frame_property_;
  FloatProperty * d_range_property_;
  FloatProperty * max_range_property_;
  FloatProperty * max_alpha_property_;
  FloatProperty * min_alpha_property_;
  ColorProperty * color_property_;
  FloatProperty * wave_velocity_property_;
  ColorProperty * wave_color_property_;
  FloatProperty * max_wave_alpha_property_;
  FloatProperty * min_wave_alpha_property_;
};

}  // namespace rviz
