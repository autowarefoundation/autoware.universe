// Copyright 2023 TIER IV, Inc.
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

#include <QLabel>
#include <rviz_common/display_context.hpp>
#include <tier4_logging_level_configure_rviz_plugin/logging_level_configure.hpp>

#include <cstdlib>

namespace rviz_plugin
{

LoggingLevelConfigureRvizPlugin::LoggingLevelConfigureRvizPlugin(QWidget * parent)
: rviz_common::Panel(parent)
{
}

// Calculate the maximum width among all target_module_name.
int LoggingLevelConfigureRvizPlugin::getMaxModuleNameWidth(QLabel * label)
{
  int max_width = 0;
  QFontMetrics metrics(label->font());
  for (const auto & item : node_logger_map_) {
    const auto & target_module_name = item.first;
    int width = metrics.horizontalAdvance(target_module_name);
    if (width > max_width) {
      max_width = width;
    }
  }
  return max_width;
}

// create node list in node_logger_map_ without
QStringList LoggingLevelConfigureRvizPlugin::getNodeList()
{
  QStringList nodes;
  for (const auto & item : node_logger_map_) {
    const auto & node_logger_vec = item.second;
    for (const auto & node_logger_pair : node_logger_vec) {
      if (!nodes.contains(node_logger_pair.first)) {
        nodes.append(node_logger_pair.first);
      }
    }
  }
  return nodes;
}

void LoggingLevelConfigureRvizPlugin::onInitialize()
{
  setLoggerNodeMap();

  QVBoxLayout * layout = new QVBoxLayout;

  QStringList levels = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};

  constexpr int height = 20;
  for (const auto & item : node_logger_map_) {
    const auto & target_node_name = item.first;

    QHBoxLayout * hLayout = new QHBoxLayout;

    // Create a QLabel to display the node name.
    QLabel * label = new QLabel(target_node_name);
    label->setFixedHeight(height);  // Set fixed height for the button
    label->setFixedWidth(getMaxModuleNameWidth(label));

    hLayout->addWidget(label);  // Add the QLabel to the hLayout.

    QButtonGroup * group = new QButtonGroup(this);
    for (const QString & level : levels) {
      QPushButton * btn = new QPushButton(level);
      btn->setFixedHeight(height);  // Set fixed height for the button
      hLayout->addWidget(btn);      // Add each QPushButton to the hLayout.
      group->addButton(btn);
      button_map_[target_node_name][level] = btn;
      connect(btn, &QPushButton::clicked, this, [this, btn, target_node_name, level]() {
        this->onButtonClick(btn, target_node_name, level);
      });
    }
    // Set the "INFO" button as checked by default and change its color.
    updateButtonColors(target_node_name, button_map_[target_node_name]["INFO"]);

    buttonGroups_[target_node_name] = group;
    layout->addLayout(hLayout);
  }

  setLayout(layout);

  // set up service clients
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  const auto & nodes = getNodeList();
  for (const QString & node : nodes) {
    const auto client = raw_node_->create_client<logging_demo::srv::ConfigLogger>(
      node.toStdString() + "/config_logger");
    client_map_[node] = client;
  }
}

// Modify the signature of the onButtonClick function:
void LoggingLevelConfigureRvizPlugin::onButtonClick(
  QPushButton * button, const QString & target_module_name, const QString & level)
{
  if (button) {
    const auto callback =
      [&](rclcpp::Client<logging_demo::srv::ConfigLogger>::SharedFuture future) {
        std::cerr << "change logging level: "
                  << std::string(future.get()->success ? "success!" : "failed...") << std::endl;
      };

    for (const auto & node_logger_map : node_logger_map_[target_module_name]) {
      const auto node_name = node_logger_map.first;
      const auto logger_name = node_logger_map.second;
      const auto req = std::make_shared<logging_demo::srv::ConfigLogger::Request>();

      req->logger_name = logger_name.toStdString();
      req->level = level.toStdString();
      std::cerr << "logger level of " << req->logger_name << " is set to " << req->level
                << std::endl;
      client_map_[node_name]->async_send_request(req, callback);
    }

    updateButtonColors(
      target_module_name, button);  // Modify updateButtonColors to accept QPushButton only.
  }
}

void LoggingLevelConfigureRvizPlugin::updateButtonColors(
  const QString & target_module_name, QPushButton * active_button)
{
  const QString LIGHT_GREEN = "rgb(181, 255, 20)";
  const QString LIGHT_GRAY = "rgb(211, 211, 211)";
  const QString LIGHT_GRAY_TEXT = "rgb(180, 180, 180)";

  for (const auto & button : button_map_[target_module_name]) {
    if (button.second == active_button) {
      button.second->setStyleSheet("background-color: " + LIGHT_GREEN + "; color: black");
    } else {
      button.second->setStyleSheet(
        "background-color: " + LIGHT_GRAY + "; color: " + LIGHT_GRAY_TEXT);
    }
  }
}

void LoggingLevelConfigureRvizPlugin::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void LoggingLevelConfigureRvizPlugin::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

void LoggingLevelConfigureRvizPlugin::setLoggerNodeMap()
{
  // ===============================================================================================
  // ====================================== Planning ===============================================
  // ===============================================================================================

  QString behavior_path_planner =
    "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner";
  QString behavior_velocity_planner =
    "/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner";

  // behavior_path_planner (all)
  node_logger_map_["behavior_path_planner"] = {
    {behavior_path_planner,
     "planning.scenario_planning.lane_driving.behavior_planning.behavior_path_planner"},
    {behavior_path_planner, "tier4_autoware_utils"}};

  // behavior_path_planner: avoidance
  node_logger_map_["behavior_path_planner: avoidance"] = {
    {behavior_path_planner,
     "planning.scenario_planning.lane_driving.behavior_planning.behavior_path_planner."
     "avoidance"}};

  // behavior_velocity_planner (all)
  node_logger_map_["behavior_velocity_planner"] = {
    {behavior_velocity_planner,
     "planning.scenario_planning.lane_driving.behavior_planning.behavior_velocity_planner"},
    {behavior_velocity_planner, "tier4_autoware_utils"}};

  // behavior_velocity_planner: intersection
  node_logger_map_["behavior_velocity_planner: intersection"] = {
    {behavior_velocity_planner,
     "planning.scenario_planning.lane_driving.behavior_planning.behavior_velocity_planner."
     "intersection"}};

  // obstacle_avoidance_planner
  QString motion_avoidance =
    "/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner";
  node_logger_map_["motion: obstacle_avoidance"] = {
    {motion_avoidance,
     "planning.scenario_planning.lane_driving.motion_planning.obstacle_avoidance_planner"},
    {motion_avoidance, "tier4_autoware_utils"}};

  // motion_velocity_smoother
  QString velocity_smoother = "/planning/scenario_planning/motion_velocity_smoother";
  node_logger_map_["motion: velocity_smoother"] = {
    {velocity_smoother, "planning.scenario_planning.motion_velocity_smoother"},
    {velocity_smoother, "tier4_autoware_utils"}};

  // ===============================================================================================
  // ======================================= Control ===============================================
  // ===============================================================================================

  QString trajectory_follower = "/control/trajectory_follower/controller_node_exe";

  // lateral_controller
  node_logger_map_["lateral_controller"] = {
    {trajectory_follower, "control.trajectory_follower.controller_node_exe.lateral_controller"},
    {trajectory_follower, "tier4_autoware_utils"},
  };

  // longitudinal_controller
  node_logger_map_["longitudinal_controller"] = {
    {trajectory_follower,
     "control.trajectory_follower.controller_node_exe.longitudinal_controller"},
    {trajectory_follower, "tier4_autoware_utils"},
  };

  // vehicle_cmd_gate
  QString vehicle_cmd_gate = "/control/vehicle_cmd_gate";
  node_logger_map_["vehicle_cmd_gate"] = {
    {vehicle_cmd_gate, "control.vehicle_cmd_gate"},
    {vehicle_cmd_gate, "tier4_autoware_utils"},
  };
}

}  // namespace rviz_plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugin::LoggingLevelConfigureRvizPlugin, rviz_common::Panel)
