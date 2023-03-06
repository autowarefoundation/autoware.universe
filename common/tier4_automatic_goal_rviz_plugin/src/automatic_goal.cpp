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

#include "automatic_goal.hpp"

namespace rviz_plugins
{
AutowareAutomaticGoalPanel::AutowareAutomaticGoalPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &AutowareAutomaticGoalPanel::updateAutoExecutionTimerTick);

  auto * h_layout = new QHBoxLayout(this);
  auto * v_layout = new QVBoxLayout(this);
  h_layout->addWidget(makeGoalListGroup());
  v_layout->addWidget(makeEngagementGroup());
  v_layout->addWidget(makeRoutingGroup());
  h_layout->addLayout(v_layout);
  setLayout(h_layout);
}

// Layouts
QGroupBox * AutowareAutomaticGoalPanel::makeGoalListGroup()
{
  auto * group = new QGroupBox("GoalList", this);
  auto * grid = new QGridLayout(group);

  load_file_btn_ptr_ = new QPushButton("Load from file", group);
  connect(load_file_btn_ptr_, SIGNAL(clicked()), SLOT(onClickLoadListFromFile()));
  grid->addWidget(load_file_btn_ptr_, 0, 0);

  save_file_btn_ptr_ = new QPushButton("Save to file", group);
  connect(save_file_btn_ptr_, SIGNAL(clicked()), SLOT(onClickSaveListToFile()));
  grid->addWidget(save_file_btn_ptr_, 1, 0);

  goal_list_widget_ptr_ = new QListWidget(group);
  goal_list_widget_ptr_->setStyleSheet("border:1px solid black;");
  grid->addWidget(goal_list_widget_ptr_, 2, 0);

  remove_selected_goal_btn_ptr_ = new QPushButton("Remove selected", group);
  connect(remove_selected_goal_btn_ptr_, SIGNAL(clicked()), SLOT(onClickRemove()));
  grid->addWidget(remove_selected_goal_btn_ptr_, 3, 0);

  loop_list_btn_ptr_ = new QPushButton("Loop list", group);
  loop_list_btn_ptr_->setCheckable(true);

  connect(loop_list_btn_ptr_, SIGNAL(toggled(bool)), SLOT(onToggleLoopList(bool)));
  grid->addWidget(loop_list_btn_ptr_, 4, 0);

  group->setLayout(grid);
  return group;
}

QGroupBox * AutowareAutomaticGoalPanel::makeRoutingGroup()
{
  auto * group = new QGroupBox("Routing", this);
  auto * grid = new QGridLayout(group);

  routing_label_ptr_ = new QLabel("INIT", group);
  routing_label_ptr_->setMinimumSize(100, 25);
  routing_label_ptr_->setAlignment(Qt::AlignCenter);
  routing_label_ptr_->setStyleSheet("border:1px solid black;");
  grid->addWidget(routing_label_ptr_, 0, 0);

  clear_route_btn_ptr_ = new QPushButton("Clear planned route", group);
  connect(clear_route_btn_ptr_, &QPushButton::clicked, [this]() { onClickClearRoute(); });
  grid->addWidget(clear_route_btn_ptr_, 1, 0);
  group->setLayout(grid);

  group->setLayout(grid);
  return group;
}

QGroupBox * AutowareAutomaticGoalPanel::makeEngagementGroup()
{
  auto * group = new QGroupBox("Engagement", this);
  auto * grid = new QGridLayout(group);

  engagement_label_ptr_ = new QLabel("INITIALIZING", group);
  engagement_label_ptr_->setMinimumSize(100, 25);
  engagement_label_ptr_->setAlignment(Qt::AlignCenter);
  engagement_label_ptr_->setStyleSheet("border:1px solid black;");
  grid->addWidget(engagement_label_ptr_, 0, 0);

  automatic_mode_btn_ptr_ = new QPushButton("Send goals automatically", group);
  automatic_mode_btn_ptr_->setCheckable(true);

  connect(automatic_mode_btn_ptr_, SIGNAL(toggled(bool)), SLOT(onToggleAutoMode(bool)));
  grid->addWidget(automatic_mode_btn_ptr_, 1, 0);

  plan_btn_ptr_ = new QPushButton("Plan to selected goal", group);
  connect(plan_btn_ptr_, &QPushButton::clicked, [this] { onClickPlan(); });
  grid->addWidget(plan_btn_ptr_, 2, 0);

  start_btn_ptr_ = new QPushButton("Start current plan", group);
  connect(start_btn_ptr_, &QPushButton::clicked, [this] { onClickStart(); });
  grid->addWidget(start_btn_ptr_, 3, 0);

  stop_btn_ptr_ = new QPushButton("Stop current plan", group);
  connect(stop_btn_ptr_, SIGNAL(clicked()), SLOT(onClickStop()));
  grid->addWidget(stop_btn_ptr_, 4, 0);
  group->setLayout(grid);

  group->setLayout(grid);
  return group;
}

void AutowareAutomaticGoalPanel::showMessageBox(const QString & string)
{
  QMessageBox msgBox(this);
  msgBox.setText(string);
  msgBox.exec();
  return;
}

// Slots
void AutowareAutomaticGoalPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  pub_marker_ = raw_node_->create_publisher<MarkerArray>("~/automatic_goal/markers", 0);
  // Executing
  sub_operation_mode_ = raw_node_->create_subscription<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareAutomaticGoalPanel::onOperationMode, this, std::placeholders::_1));

  cli_change_to_autonomous_ = raw_node_->create_client<ChangeOperationMode>(
    "/api/operation_mode/change_to_autonomous", rmw_qos_profile_services_default);

  cli_change_to_stop_ = raw_node_->create_client<ChangeOperationMode>(
    "/api/operation_mode/change_to_stop", rmw_qos_profile_services_default);

  // Planning
  sub_route_ = raw_node_->create_subscription<RouteState>(
    "/api/routing/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareAutomaticGoalPanel::onRoute, this, std::placeholders::_1));

  cli_clear_route_ = raw_node_->create_client<ClearRoute>(
    "/api/routing/clear_route", rmw_qos_profile_services_default);

  cli_set_route_ = raw_node_->create_client<SetRoutePoints>(
    "/api/routing/set_route_points", rmw_qos_profile_services_default);

  sub_append_goal_ = raw_node_->create_subscription<PoseStamped>(
    "~/automatic_goal/goal", 5,
    std::bind(&AutowareAutomaticGoalPanel::onAppendGoal, this, std::placeholders::_1));
}

void AutowareAutomaticGoalPanel::onToggleLoopList(bool checked)
{
  is_loop_list_on = checked;
  updateGUI();
}

void AutowareAutomaticGoalPanel::onToggleAutoMode(bool checked)
{
  if (checked && goal_list_widget_ptr_->selectedItems().count() != 1) {
    showMessageBox("Select the first goal in GoalList");
    automatic_mode_btn_ptr_->setChecked(false);
    return;
  } else if (checked)
    current_goal = goal_list_widget_ptr_->currentRow();

  is_automatic_mode_on = checked;
  is_automatic_mode_on ? timer->start(1000) : timer->stop();
  updateGoalList();     // reset icons in the list
  onClickClearRoute();  // here will be set PanelState::AUTONEXT or PanelState::EDITING;
}

void AutowareAutomaticGoalPanel::onClickPlan()
{
  if (goal_list_widget_ptr_->selectedItems().count() != 1) {
    showMessageBox("Select a goal in GoalList");
    return;
  }

  if (callPlanToGoalIndex(cli_set_route_, goal_list_widget_ptr_->currentRow())) {
    state = PanelState::PLANNING;
    updateGUI();
  }
}

void AutowareAutomaticGoalPanel::onClickStart()
{
  if (callServiceWithoutResponse<ChangeOperationMode>(cli_change_to_autonomous_)) {
    state = PanelState::STARTING;
    updateGUI();
  }
}

void AutowareAutomaticGoalPanel::onClickStop()
{
  // if ERROR is set then the ego is already stopped
  if (state == PanelState::ERROR) {
    state = PanelState::STOPPED;
    updateGUI();
  } else if (callServiceWithoutResponse<ChangeOperationMode>(cli_change_to_stop_)) {
    state = PanelState::STOPPING;
    updateGUI();
  }
}

void AutowareAutomaticGoalPanel::onClickClearRoute()
{
  if (callServiceWithoutResponse<ClearRoute>(cli_clear_route_)) {
    state = PanelState::CLEARING;
    updateGUI();
  }
}

void AutowareAutomaticGoalPanel::onClickRemove()
{
  if (goal_list_widget_ptr_->currentRow() < goal_list_.size())
    goal_list_.erase(goal_list_.begin() + goal_list_widget_ptr_->currentRow());
  updateGUI();
  updateGoalList();
}

void AutowareAutomaticGoalPanel::onClickLoadListFromFile()
{
  if (goal_list_.size() > 0) {
    QString fileName = QFileDialog::getOpenFileName(
      this, tr("Open File with GoalList"), "/home", tr("Goal lists (*.yaml)"));
    if (fileName.count() > 0) loadGoalList(fileName.toStdString());
  }
}

void AutowareAutomaticGoalPanel::onClickSaveListToFile()
{
  if (goal_list_.size() > 0) {
    QString fileName = QFileDialog::getSaveFileName(
      this, tr("Save File with  GoalList"), "/home/goal_list.yaml", tr("Goal lists (*.yaml)"));
    if (fileName.count() > 0) saveGoalList(fileName.toStdString());
  }
}

// Inputs
void AutowareAutomaticGoalPanel::onRoute(const RouteState::ConstSharedPtr msg)
{
  std::pair<QString, QString> style;
  switch (msg->state) {
    case RouteState::UNSET:
      style = std::make_pair("UNSET", "background-color: #FFFF00;");  // yellow
      if (state == PanelState::CLEARING) state = PanelState::CLEARED;
      break;
    case RouteState::SET:
      style = std::make_pair("SET", "background-color: #00FF00;");  // green
      if (state == PanelState::PLANNING) state = PanelState::PLANNED;
      break;
    case RouteState::ARRIVED:
      style = std::make_pair("ARRIVED", "background-color: #FFA500;");  // orange
      if (state == PanelState::STARTED) state = PanelState::ARRIVED;
      break;
    case RouteState::CHANGING:
      style = std::make_pair("CHANGING", "background-color: #FFFF00;");  // yellow
      break;
    default:
      style = std::make_pair("UNKNOWN", "background-color: #FF0000;");  // red
      break;
  }
  updateLabel(routing_label_ptr_, style.first, style.second);
  updateGUI();
}

void AutowareAutomaticGoalPanel::onOperationMode(const OperationModeState::ConstSharedPtr msg)
{
  if (state == PanelState::INITIALIZING && msg->mode == OperationModeState::STOP)
    state = PanelState::EDITING;

  if (msg->mode == OperationModeState::AUTONOMOUS && state == PanelState::STARTING)
    state = PanelState::STARTED;
  else if (msg->mode == OperationModeState::STOP && state == PanelState::STOPPING)
    state = PanelState::STOPPED;
  updateGUI();
}

void AutowareAutomaticGoalPanel::onAppendGoal(const PoseStamped::ConstSharedPtr pose)
{
  if (state == PanelState::EDITING) {
    goal_list_.push_back(pose);
    updateGoalList();
    updateGUI();
  }
}

// Updates
void AutowareAutomaticGoalPanel::setGoalColor(const unsigned goal_index, QColor color)
{
  QPixmap pixmap(10, 10);
  pixmap.fill(color);
  QIcon icon(pixmap);
  goal_list_widget_ptr_->item(goal_index)->setIcon(icon);
}

void AutowareAutomaticGoalPanel::updateAutoExecutionTimerTick()
{
  if (is_automatic_mode_on) {
    if (state == PanelState::AUTONEXT) {
      // end if loop is off
      if (current_goal >= goal_list_.size() && !is_loop_list_on) {
        disableAutomaticMode();
        return;
      }
      // plan to next goal
      current_goal = current_goal % goal_list_.size();
      if (callPlanToGoalIndex(cli_set_route_, current_goal)) {
        state = PanelState::PLANNING;
        updateGUI();
      }
    } else if (state == PanelState::PLANNED) {
      setGoalColor(current_goal, QColor("yellow"));
      onClickStart();
    } else if (state == PanelState::ARRIVED) {
      setGoalColor(current_goal++, QColor("green"));
      onClickClearRoute();  // will be set AUTONEXT as next state
    } else if (state == PanelState::STOPPED || state == PanelState::ERROR) {
      disableAutomaticMode();
    }
  }
}

void AutowareAutomaticGoalPanel::updateGUI()
{
  deactivateButton(automatic_mode_btn_ptr_);
  deactivateButton(remove_selected_goal_btn_ptr_);
  deactivateButton(clear_route_btn_ptr_);
  deactivateButton(plan_btn_ptr_);
  deactivateButton(start_btn_ptr_);
  deactivateButton(stop_btn_ptr_);
  deactivateButton(load_file_btn_ptr_);
  deactivateButton(save_file_btn_ptr_);
  deactivateButton(loop_list_btn_ptr_);

  std::pair<QString, QString> style;
  switch (state) {
    case PanelState::EDITING:
      activateButton(load_file_btn_ptr_);
      if (goal_list_.size() > 0) {
        activateButton(plan_btn_ptr_);
        activateButton(remove_selected_goal_btn_ptr_);
        activateButton(automatic_mode_btn_ptr_);
        activateButton(save_file_btn_ptr_);
        activateButton(loop_list_btn_ptr_);
      }
      style = std::make_pair("EDITING", "background-color: #FFFF00;");
      break;
    case PanelState::PLANNED:
      activateButton(start_btn_ptr_);
      activateButton(clear_route_btn_ptr_);
      activateButton(save_file_btn_ptr_);
      style = std::make_pair("PLANNED", "background-color: #00FF00;");
      break;
    case PanelState::STARTED:
      activateButton(stop_btn_ptr_);
      activateButton(save_file_btn_ptr_);
      style = std::make_pair("STARTED", "background-color: #00FF00;");
      break;
    case PanelState::STOPPED:
      activateButton(start_btn_ptr_);
      activateButton(automatic_mode_btn_ptr_);
      activateButton(clear_route_btn_ptr_);
      activateButton(save_file_btn_ptr_);
      style = std::make_pair("STOPPED", "background-color: #00FF00;");
      break;
    case PanelState::ARRIVED:
      if (!is_automatic_mode_on) onClickClearRoute();  // will be set EDITING as next state
      break;
    case PanelState::CLEARED:
      is_automatic_mode_on ? state = PanelState::AUTONEXT : state = PanelState::EDITING;
      updateGUI();
      break;
    case PanelState::ERROR:
      activateButton(stop_btn_ptr_);
      if (goal_list_.size() > 0) activateButton(save_file_btn_ptr_);
      style = std::make_pair("ERROR", "background-color: #FF0000;");
      break;
    case PanelState::PLANNING:
      activateButton(clear_route_btn_ptr_);
      style = std::make_pair("PLANNING", "background-color: #FFA500;");
      break;
    case PanelState::STARTING:
      style = std::make_pair("STARTING", "background-color: #FFA500;");
      break;
    case PanelState::STOPPING:
      style = std::make_pair("STOPPING", "background-color: #FFA500;");
      break;
    case PanelState::CLEARING:
      style = std::make_pair("CLEARING", "background-color: #FFA500;");
      break;
  }

  automatic_mode_btn_ptr_->setStyleSheet("");
  loop_list_btn_ptr_->setStyleSheet("");
  if (is_automatic_mode_on) automatic_mode_btn_ptr_->setStyleSheet("background-color: green");
  if (is_loop_list_on) loop_list_btn_ptr_->setStyleSheet("background-color: green");

  updateLabel(engagement_label_ptr_, style.first, style.second);
}

void AutowareAutomaticGoalPanel::updateGoalList()
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2);
  goal_list_widget_ptr_->clear();

  int i = 0;
  for (const auto & p : goal_list_) {
    tf2::Quaternion tf2_quat;
    tf2::convert(p->pose.orientation, tf2_quat);
    ss << "G" << i++ << " (" << p->pose.position.x << ", ";
    ss << p->pose.position.y << ", " << tf2::getYaw(tf2_quat) << ")";
    QString str = QString::fromStdString(ss.str());
    QListWidgetItem * item = new QListWidgetItem(str, goal_list_widget_ptr_);
    goal_list_widget_ptr_->addItem(item);
    setGoalColor(goal_list_widget_ptr_->count() - 1, QColor("white"));
    ss.clear();
    ss.str("");
  }
  publishMarkers();
}

void AutowareAutomaticGoalPanel::publishMarkers()
{
  MarkerArray text_array;
  MarkerArray arrow_array;
  // Clear existing
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = rclcpp::Time();
  marker.ns = "names";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  text_array.markers.push_back(marker);
  marker.ns = "poses";
  arrow_array.markers.push_back(marker);
  pub_marker_->publish(text_array);
  pub_marker_->publish(arrow_array);
  text_array.markers.clear();
  arrow_array.markers.clear();
  // Publish current
  for (int i = 0; i < goal_list_.size(); i++) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Time();
    marker.ns = "names";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = goal_list_[i]->pose;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.scale.x = 1.6;
    marker.scale.y = 1.6;
    marker.scale.z = 1.6;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.frame_locked = false;
    marker.text = "G" + std::to_string(i);
    text_array.markers.push_back(marker);
    marker.ns = "poses";
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_array.markers.push_back(marker);
  }
  pub_marker_->publish(text_array);
  pub_marker_->publish(arrow_array);
}

// File
void AutowareAutomaticGoalPanel::loadGoalList(const std::string & file_path)
{
  YAML::Node node = YAML::LoadFile(file_path);
  goal_list_.clear();
  for (unsigned i = 0; i < node.size(); i++) {
    std::shared_ptr<PoseStamped> pose = std::make_shared<PoseStamped>();
    pose->header.frame_id = "map";
    pose->header.stamp = rclcpp::Time();
    pose->pose.position.x = node[i]["position_x"].as<double>();
    pose->pose.position.y = node[i]["position_y"].as<double>();
    pose->pose.position.z = node[i]["position_z"].as<double>();
    pose->pose.orientation.x = node[i]["orientation_x"].as<double>();
    pose->pose.orientation.y = node[i]["orientation_y"].as<double>();
    pose->pose.orientation.z = node[i]["orientation_z"].as<double>();
    pose->pose.orientation.w = node[i]["orientation_w"].as<double>();
    goal_list_.push_back(pose);
  }
  updateGoalList();
}

void AutowareAutomaticGoalPanel::saveGoalList(const std::string & file_path)
{
  YAML::Node node;
  for (unsigned i = 0; i < goal_list_.size(); ++i) {
    node[i]["position_x"] = goal_list_[i]->pose.position.x;
    node[i]["position_y"] = goal_list_[i]->pose.position.y;
    node[i]["position_z"] = goal_list_[i]->pose.position.z;
    node[i]["orientation_x"] = goal_list_[i]->pose.orientation.x;
    node[i]["orientation_y"] = goal_list_[i]->pose.orientation.y;
    node[i]["orientation_z"] = goal_list_[i]->pose.orientation.z;
    node[i]["orientation_w"] = goal_list_[i]->pose.orientation.w;
  }
  std::ofstream fout(file_path);
  fout << node;
  fout.close();
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowareAutomaticGoalPanel, rviz_common::Panel)
