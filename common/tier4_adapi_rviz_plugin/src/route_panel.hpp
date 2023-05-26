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

#ifndef ROUTE_PANEL_HPP_
#define ROUTE_PANEL_HPP_

#include <QCheckBox>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>

namespace tier4_adapi_rviz_plugins
{

class RoutePanel : public rviz_common::Panel
{
  Q_OBJECT
  using PoseStamped = geometry_msgs::msg::PoseStamped;

public:
  explicit RoutePanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  QPushButton * mode_goal_only_;
  QPushButton * mode_waypoints_;
  QPushButton * waypoints_reset_;
  QPushButton * waypoints_apply_;
  QCheckBox * allow_goal_modification_;

  void setRoute(const PoseStamped & pose);

  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
  std::vector<PoseStamped> waypoints_;
  void onPose(const PoseStamped::ConstSharedPtr msg);
};

}  // namespace tier4_adapi_rviz_plugins

#endif  // ROUTE_PANEL_HPP_
