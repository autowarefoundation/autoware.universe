/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2014 University of Osnabrück
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES  (INCLUDING,  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE   OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * route_set_tool.h
 *
 *  Author: Henning Deeken <hdeeken@uos.de>
 *
 */

#ifndef OPERATION_VIEW_TOOL_HPP_
#define OPERATION_VIEW_TOOL_HPP_

#include "operation_view_controller.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

#include <QApplication>
#include <QIcon>
#include <QMessageBox>

#include <map>
#include <vector>

/**
 *
 *@class OperationViewTool
 *
 *@brief Implements a rviz tool that allows to navigate in a ego-shooter mode.
 */

namespace tier4_adapi_rviz_plugins
{

class FPSMotionConfigWidget;
class OperationViewTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  OperationViewTool();
  ~OperationViewTool();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();

  virtual int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel);
  virtual int processMouseEvent(rviz_common::ViewportMouseEvent & event);

private Q_SLOTS:
  void setOffset() { m_pos_offset = (double)step_length_property_->getFloat(); }
  void setBoost()
  {
    if (boost_property_->getFloat() < 0.0) {
      m_boost = 0.0;
    } else if (boost_property_->getFloat() > 1.0) {
      m_boost = 1.0;
    } else {
      m_boost = (double)boost_property_->getFloat();
    }
  }

  void setFlyMode() { m_fly_mode = fly_property_->getBool(); }
  void setLeftHandMode() { m_left_hand_mode = left_hand_property_->getBool(); }
  // temporarily disabled
  // void setFallbackTool() { m_fallback_tool = m_tools.at(fallback_tool_property_->getOptionInt());
  // }
  void setFallbackViewController()
  {
    m_fallback_view_controller =
      m_view_controller_classes.at(fallback_view_controller_property_->getOptionInt());
  }

private:
  Ogre::SceneNode * m_sceneNode;

  bool m_fly_mode;
  bool m_left_hand_mode;
  bool m_removed_select;

  double m_pos_offset;
  double m_boost;

  QStringList m_tool_classes;
  // temporarily disabled
  // std::vector<rviz_common::Tool *> m_tools;
  // rviz_common::Tool *m_fallback_tool;

  QStringList m_view_controller_classes;
  QString m_fallback_view_controller;
  std::vector<rviz_common::ViewController *> m_view_controller;

  rviz_common::properties::FloatProperty * step_length_property_;
  rviz_common::properties::FloatProperty * boost_property_;
  rviz_common::properties::BoolProperty * fly_property_;
  rviz_common::properties::BoolProperty * left_hand_property_;
  // temporarily disabled
  // rviz_common::properties::EnumProperty *fallback_tool_property_;
  rviz_common::properties::EnumProperty * fallback_view_controller_property_;

  // temporarily disabled
  // void setFallbackToolProperty();
  void setFallbackViewControllerProperty();
};
}  // namespace tier4_adapi_rviz_plugins
#endif  // OPERATION_VIEW_TOOL_HPP_
