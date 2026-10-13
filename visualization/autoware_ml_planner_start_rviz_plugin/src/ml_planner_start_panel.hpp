// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ML_PLANNER_START_PANEL_HPP_
#define ML_PLANNER_START_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace autoware::ml_planner_start_rviz_plugin
{

class MLPlannerStartPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MLPlannerStartPanel(QWidget * parent = nullptr);
  ~MLPlannerStartPanel() override;

  void onInitialize() override;

private:
  enum class Phase { Idle, Starting, Working, Stopping, Done, Error };
  enum class StopReason { None, Departure, Manual };

  void onStartClicked();
  void onForceStopClicked();
  void onKinematic(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void refreshUi();
  void sendSetBool(bool data);
  void onServiceDone(bool data, bool success, const std::string & message);
  /// Move Working -> Stopping. Returns false if another stop is already in progress.
  bool claimStop(StopReason reason);
  void tryDepartureStop();

  QPushButton * start_button_{nullptr};
  QPushButton * force_stop_button_{nullptr};
  QLabel * status_label_{nullptr};
  QLabel * speed_label_{nullptr};

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  std::optional<rclcpp::Client<std_srvs::srv::SetBool>::SharedFutureAndRequestId> pending_request_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_;

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;

  std::mutex phase_mutex_;
  Phase phase_{Phase::Idle};
  StopReason stop_reason_{StopReason::None};
  std::string status_message_;

  std::atomic<double> speed_mps_{0.0};
  std::atomic<bool> speed_received_{false};
  std::shared_ptr<std::atomic<bool>> alive_;
};

}  // namespace autoware::ml_planner_start_rviz_plugin

#endif  // ML_PLANNER_START_PANEL_HPP_
