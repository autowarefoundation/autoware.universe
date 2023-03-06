//
//  Copyright 2020 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE_STATE_PANEL_HPP_
#define AUTOWARE_STATE_PANEL_HPP_

#include <QFileDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QListWidget>
#include <QMessageBox>
#include <QPixmap>
#include <QPushButton>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>

namespace rviz_plugins
{
enum class AutomaticGoalPanelState {
  INITIALIZING,
  EDITING,
  PLANNING,
  PLANNED,
  STARTING,
  STARTED,
  ARRIVED,
  AUTONEXT,
  STOPPING,
  STOPPED,
  CLEARING,
  CLEARED,
  ERROR,
};

class AutowareAutomaticGoalPanel : public rviz_common::Panel
{
  using PanelState = AutomaticGoalPanelState;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  using SetRoutePoints = autoware_adapi_v1_msgs::srv::SetRoutePoints;
  using ClearRoute = autoware_adapi_v1_msgs::srv::ClearRoute;
  Q_OBJECT

public:
  explicit AutowareAutomaticGoalPanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:  // NOLINT for Qt
  void onToggleLoopList(bool checked);
  void onToggleAutoMode(bool checked);
  void onClickPlan();
  void onClickStart();
  void onClickStop();
  void onClickClearRoute();
  void onClickRemove();
  void onClickLoadListFromFile();
  void onClickSaveListToFile();

protected:
  void onRoute(const RouteState::ConstSharedPtr msg);
  void onOperationMode(const OperationModeState::ConstSharedPtr msg);
  void onAppendGoal(const PoseStamped::ConstSharedPtr pose);
  void setGoalColor(const unsigned goal_index, QColor color);
  void updateAutoExecutionTimerTick();
  void updateGoalList();
  void updateGUI();
  void publishMarkers();
  void loadGoalList(const std::string & file);
  void saveGoalList(const std::string & file);

  rclcpp::Node::SharedPtr raw_node_;
  PanelState state{PanelState::INITIALIZING};
  std::vector<PoseStamped::ConstSharedPtr> goal_list_;
  unsigned current_goal{0};
  bool is_automatic_mode_on{false};
  bool is_loop_list_on{false};

  // Sub/Client
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_append_goal_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_change_to_autonomous_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_change_to_stop_;
  rclcpp::Subscription<RouteState>::SharedPtr sub_route_;
  rclcpp::Client<ClearRoute>::SharedPtr cli_clear_route_;
  rclcpp::Client<SetRoutePoints>::SharedPtr cli_set_route_;

  // QT Objects
  QTimer * timer;
  QGroupBox * makeGoalListGroup();
  QGroupBox * makeRoutingGroup();
  QGroupBox * makeEngagementGroup();
  void showMessageBox(const QString & string);
  QListWidget * goal_list_widget_ptr_{nullptr};
  QLabel * routing_label_ptr_{nullptr};
  QLabel * operation_mode_label_ptr_{nullptr};
  QLabel * engagement_label_ptr_{nullptr};
  QPushButton * loop_list_btn_ptr_{nullptr};
  QPushButton * load_file_btn_ptr_{nullptr};
  QPushButton * save_file_btn_ptr_{nullptr};
  QPushButton * automatic_mode_btn_ptr_{nullptr};
  QPushButton * remove_selected_goal_btn_ptr_{nullptr};
  QPushButton * clear_route_btn_ptr_{nullptr};
  QPushButton * plan_btn_ptr_{nullptr};
  QPushButton * start_btn_ptr_{nullptr};
  QPushButton * stop_btn_ptr_{nullptr};

  // Common
  void disableAutomaticMode() { automatic_mode_btn_ptr_->setChecked(false); }

  bool callPlanToGoalIndex(
    const rclcpp::Client<SetRoutePoints>::SharedPtr client, const int goal_index)
  {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(raw_node_->get_logger(), "SetRoutePoints client is unavailable");
      return false;
    }

    auto req = std::make_shared<SetRoutePoints::Request>();
    req->header = goal_list_.at(goal_index)->header;
    req->goal = goal_list_.at(goal_index)->pose;
    client->async_send_request(
      req, [this](typename rclcpp::Client<SetRoutePoints>::SharedFuture result) {
        if (result.get()->status.code != 0) state = PanelState::ERROR;
        updateGUI();
        printCallResult<SetRoutePoints>(result);
      });
    return true;
  }

  template <typename T>
  bool callServiceWithoutResponse(const typename rclcpp::Client<T>::SharedPtr client)
  {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(raw_node_->get_logger(), "Client is unavailable");
      return false;
    }

    auto req = std::make_shared<typename T::Request>();
    client->async_send_request(req, [this](typename rclcpp::Client<T>::SharedFuture result) {
      if (result.get()->status.code != 0) state = PanelState::ERROR;
      updateGUI();
      printCallResult<T>(result);
    });
    return true;
  }

  template <typename T>
  void printCallResult(typename rclcpp::Client<T>::SharedFuture result)
  {
    if (result.get()->status.code != 0) {
      RCLCPP_ERROR(
        raw_node_->get_logger(), "%s status: %d, %s", typeid(T).name(), result.get()->status.code,
        result.get()->status.message.c_str());
    } else {
      RCLCPP_INFO(
        raw_node_->get_logger(), "%s status: %d, %s", typeid(T).name(), result.get()->status.code,
        result.get()->status.message.c_str());
    }
  }

  static void activateButton(QAbstractButton * btn) { btn->setEnabled(true); }
  static void deactivateButton(QAbstractButton * btn) { btn->setEnabled(false); }

  static void updateLabel(QLabel * label, QString text, QString style_sheet)
  {
    label->setText(text);
    label->setStyleSheet(style_sheet);
  }
};
}  // namespace rviz_plugins

#endif  // AUTOWARE_STATE_PANEL_HPP_
