// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef MRM_GOAL_HPP_
#define MRM_GOAL_HPP_

#include <rviz_default_plugins/tools/goal_pose/goal_tool.hpp>

namespace rviz_plugins
{

class MrmGoalTool : public rviz_default_plugins::tools::GoalTool
{
  Q_OBJECT

public:
  MrmGoalTool();
  void onInitialize() override;
};

}  // namespace rviz_plugins

#endif  // MRM_GOAL_HPP_
