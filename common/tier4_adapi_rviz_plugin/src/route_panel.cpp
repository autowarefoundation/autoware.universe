//
//  Copyright 2020 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#include "route_panel.hpp"

#include <QGridLayout>
#include <QGroupBox>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

namespace tier4_adapi_rviz_plugins
{

RoutePanel::RoutePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  mode_goal_only_ = new QPushButton("goal only");
  mode_waypoints_ = new QPushButton("waypoints");
  waypoints_reset_ = new QPushButton("reset");
  waypoints_apply_ = new QPushButton("apply");
  allow_goal_modification_ = new QCheckBox("allow goal modification");

  mode_goal_only_->setCheckable(true);
  mode_waypoints_->setCheckable(true);

  // layout widgets
  const auto layout = new QGridLayout();
  setLayout(layout);

  // layout main mode buttons
  {
    const auto group = new QGroupBox("mode");
    const auto local = new QVBoxLayout();
    local->addWidget(mode_goal_only_);
    local->addWidget(mode_waypoints_);
    group->setLayout(local);
    layout->addWidget(group, 0, 0);
  }

  // layout waypoint buttons
  {
    const auto group = new QGroupBox("waypoints");
    const auto local = new QVBoxLayout();
    local->addWidget(waypoints_reset_);
    local->addWidget(waypoints_apply_);
    group->setLayout(local);
    layout->addWidget(group, 0, 1);
  }

  // layout options
  {
    const auto group = new QGroupBox("options");
    const auto local = new QVBoxLayout();
    local->addWidget(allow_goal_modification_);
    group->setLayout(local);
    layout->addWidget(group, 1, 0, 1, 2);
  }
}

void RoutePanel::onInitialize()
{
  auto lock = getDisplayContext()->getRosNodeAbstraction().lock();
  auto node = lock->get_raw_node();

  sub_pose_ = node->create_subscription<PoseStamped>(
    "/rviz/routing/pose", rclcpp::QoS(1),
    std::bind(&RoutePanel::onPose, this, std::placeholders::_1));
}

void RoutePanel::setRoute(const PoseStamped & pose)
{
  (void)pose;
}

void RoutePanel::onPose(const PoseStamped::ConstSharedPtr msg)
{
  if (mode_goal_only_->isChecked()) {
    setRoute(*msg);
  }
  if (mode_waypoints_->isChecked()) {
    waypoints_.push_back(*msg);
  }
}

}  // namespace tier4_adapi_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tier4_adapi_rviz_plugins::RoutePanel, rviz_common::Panel)
