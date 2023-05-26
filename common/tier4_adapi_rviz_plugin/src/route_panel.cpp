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

#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>

namespace tier4_adapi_rviz_plugins
{

RoutePanel::RoutePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  waypoints_mode_ = new QPushButton("mode");
  waypoints_reset_ = new QPushButton("reset");
  waypoints_apply_ = new QPushButton("apply");
  allow_goal_modification_ = new QCheckBox("allow goal modification");

  waypoints_mode_->setCheckable(true);
  waypoints_reset_->setDisabled(true);
  waypoints_apply_->setDisabled(true);

  // layout widgets
  const auto layout = new QVBoxLayout();
  setLayout(layout);

  // layout waypoints mode
  {
    const auto group = new QGroupBox("waypoints");
    const auto local = new QHBoxLayout();
    local->addWidget(waypoints_mode_);
    local->addWidget(waypoints_reset_);
    local->addWidget(waypoints_apply_);
    group->setLayout(local);
    layout->addWidget(group);
  }

  // layout options
  {
    const auto group = new QGroupBox("options");
    const auto local = new QHBoxLayout();
    local->addWidget(allow_goal_modification_);
    group->setLayout(local);
    layout->addWidget(group);
  }
}

void RoutePanel::onInitialize()
{
  auto lock = getDisplayContext()->getRosNodeAbstraction().lock();
  auto node = lock->get_raw_node();

  sub_pose_ = node->create_subscription<PoseStamped>(
    "/rviz/routing/pose", rclcpp::QoS(1),
    std::bind(&RoutePanel::onPose, this, std::placeholders::_1));

  const auto adaptor = component_interface_utils::NodeAdaptor(node.get());
  adaptor.init_cli(cli_clear_);
  adaptor.init_cli(cli_route_);
}

void RoutePanel::setRoute(const PoseStamped & pose)
{
  const auto req = std::make_shared<ClearRoute::Service::Request>();
  cli_clear_->async_send_request(req, [this, pose](auto) {
    const auto req = std::make_shared<SetRoutePoints::Service::Request>();
    req->header = pose.header;
    req->goal = pose.pose;
    req->option.allow_goal_modification = allow_goal_modification_->isChecked();
    cli_route_->async_send_request(req);
  });
}

void RoutePanel::onPose(const PoseStamped::ConstSharedPtr msg)
{
  if (!waypoints_mode_->isChecked()) {
    setRoute(*msg);
  } else {
    waypoints_.push_back(*msg);
  }
}

}  // namespace tier4_adapi_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tier4_adapi_rviz_plugins::RoutePanel, rviz_common::Panel)
