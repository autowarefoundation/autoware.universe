// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#include "mrm_goal.hpp"

namespace rviz_plugins
{

MrmGoalTool::MrmGoalTool()
{
  shortcut_key_ = 'm';
}

void MrmGoalTool::onInitialize()
{
  GoalTool::onInitialize();
  setName("MRM Goal Pose");
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::MrmGoalTool, rviz_common::Tool)
