/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef OPERATION_VIEW_CONTROLLER_HPP_
#define OPERATION_VIEW_CONTROLLER_HPP_

#include "rviz_default_plugins/view_controllers/orbit/orbit_view_controller.hpp"

#include <OgreVector3.h>

namespace Ogre
{
class SceneNode;
}

namespace tier4_adapi_rviz_plugins
{
class TfFrameProperty;

/**
 * \brief Like the orbit view controller, but focal point moves only in the x-y plane.
 */
class OperationViewController : public rviz_default_plugins::view_controllers::OrbitViewController
{
  Q_OBJECT

public:
  void onInitialize() override;

  void handleMouseEvent(rviz_common::ViewportMouseEvent & evt) override;

  void lookAt(const Ogre::Vector3 & point) override;

  void reset() override;

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @a source_view.
   *
   * @a source_view must return a valid @c Ogre::Camera* from getCamera(). */
  void mimic(rviz_common::ViewController * source_view) override;

protected:
  void updateCamera() override;

  void updateTargetSceneNode() override;

  std::pair<bool, Ogre::Vector3> intersectGroundPlane(Ogre::Ray mouse_ray);
};

}  // namespace tier4_adapi_rviz_plugins

#endif  // OPERATION_VIEW_CONTROLLER_HPP_
