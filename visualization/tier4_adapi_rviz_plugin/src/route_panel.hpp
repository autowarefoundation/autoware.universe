//  Copyright 2023 The Autoware Contributors
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

#ifndef ROUTE_PANEL_HPP_
#define ROUTE_PANEL_HPP_

#include <QButtonGroup>
#include <QCheckBox>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <autoware/adapi_specs/routing.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>

namespace tier4_adapi_rviz_plugins
{

class RoutePanel : public rviz_common::Panel
{
  Q_OBJECT
  using ClearRoute = autoware::adapi_specs::routing::ClearRoute;
  using SetRoutePoints = autoware::adapi_specs::routing::SetRoutePoints;
  using ChangeRoutePoints = autoware::adapi_specs::routing::ChangeRoutePoints;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

public:
  explicit RoutePanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  QPushButton * waypoints_mode_;
  QPushButton * waypoints_reset_;
  QPushButton * waypoints_apply_;
  QPushButton * adapi_clear_;
  QPushButton * adapi_set_;
  QPushButton * adapi_change_;
  QLabel * adapi_response_;
  QCheckBox * adapi_auto_clear_;
  QGroupBox * waypoints_group_;
  QCheckBox * allow_goal_modification_;

  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
  std::vector<PoseStamped> waypoints_;
  void onPose(const PoseStamped::ConstSharedPtr msg);

  enum AdapiMode { Set, Change };
  AdapiMode adapi_mode_;

  autoware::component_interface_utils::Client<ClearRoute>::SharedPtr cli_clear_;
  autoware::component_interface_utils::Client<SetRoutePoints>::SharedPtr cli_set_;
  autoware::component_interface_utils::Client<ChangeRoutePoints>::SharedPtr cli_change_;
  void requestRoute(const PoseStamped & pose);
  void asyncSendRequest(SetRoutePoints::Service::Request::SharedPtr req);

private slots:
  void clearRoute();
  void onWaypointsMode(bool clicked);
  void onWaypointsReset();
  void onWaypointsApply();
};

}  // namespace tier4_adapi_rviz_plugins

#endif  // ROUTE_PANEL_HPP_
