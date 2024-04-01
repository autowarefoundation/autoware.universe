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

#include "autoware_state_panel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QString>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

inline std::string Bool2String(const bool var)
{
  return var ? "True" : "False";
}

namespace rviz_plugins
{
AutowareStatePanel::AutowareStatePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // Panel Configuration
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  this->setMinimumWidth(550);
  this->setMaximumWidth(550);

  // Layout
  auto * main_v_layout = new QVBoxLayout;
  main_v_layout->setAlignment(Qt::AlignTop);

  // Operation Mode
  auto * operation_mode_group = makeOperationModeGroup();

  main_v_layout->addWidget(operation_mode_group);

  // Diagnostic
  auto * diagnostic_group_box = new QGroupBox("Diagnostic");
  auto * diagnostic_v_layout = new QVBoxLayout;

  auto * localization_group = makeLocalizationGroup();
  auto * motion_group = makeMotionGroup();
  auto * fail_safe_group = makeFailSafeGroup();
  auto * routing_group = makeRoutingGroup();
  auto * diagnostic_group = makeDiagnosticGroup();

  diagnostic_v_layout->addLayout(routing_group);
  // add space between routing and localization
  diagnostic_v_layout->addSpacing(5);
  diagnostic_v_layout->addLayout(localization_group);
  diagnostic_v_layout->addSpacing(5);
  diagnostic_v_layout->addLayout(motion_group);
  diagnostic_v_layout->addSpacing(5);
  diagnostic_v_layout->addLayout(fail_safe_group);
  diagnostic_v_layout->addSpacing(5);
  diagnostic_v_layout->addLayout(diagnostic_group);
  diagnostic_group_box->setLayout(diagnostic_v_layout);
  main_v_layout->addWidget(diagnostic_group_box);

  // Velocity Limit
  auto * velocity_limit_group = makeVelocityLimitGroup();

  main_v_layout->addWidget(velocity_limit_group);
  main_v_layout->addStretch();

  // Settting the layout
  setLayout(main_v_layout);
}

QGroupBox * AutowareStatePanel::makeOperationModeGroup()
{
  control_mode_switch_ptr_ = new CustomToggleSwitch(this);
  connect(
    control_mode_switch_ptr_, &QCheckBox::stateChanged, this,
    &AutowareStatePanel::onSwitchStateChanged);
  auto_button_ptr_ = new QPushButton("AUTO");
  auto_button_ptr_->setCheckable(true);
  auto_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(auto_button_ptr_, SIGNAL(clicked()), SLOT(onClickAutonomous()));
  stop_button_ptr_ = new QPushButton("STOP");
  stop_button_ptr_->setCheckable(true);
  stop_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(stop_button_ptr_, SIGNAL(clicked()), SLOT(onClickStop()));
  local_button_ptr_ = new QPushButton("LOCAL");
  local_button_ptr_->setCheckable(true);
  local_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(local_button_ptr_, SIGNAL(clicked()), SLOT(onClickLocal()));
  remote_button_ptr_ = new QPushButton("REMOTE");
  remote_button_ptr_->setCheckable(true);
  remote_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(remote_button_ptr_, SIGNAL(clicked()), SLOT(onClickRemote()));

  control_mode_label_ptr_ = new QLabel("Autoware Control Mode:  ");
  // set its width to fit the text
  control_mode_label_ptr_->setFixedWidth(
    control_mode_label_ptr_->fontMetrics().horizontalAdvance("Autoware Control Mode:  "));

  auto * group = new QGroupBox("OperationMode");

  auto * hLayoutTop = new QHBoxLayout;
  auto * vLayout = new QVBoxLayout;
  hLayoutTop->addWidget(control_mode_label_ptr_);
  hLayoutTop->addWidget(control_mode_switch_ptr_);
  hLayoutTop->addStretch();

  vLayout->addLayout(hLayoutTop);

  // create a small space between the hlayouttop and the hlayoutbottom
  vLayout->addSpacing(5);

  // auto * hLayoutMid = new QHBoxLayout;
  // hLayoutMid->addWidget(auto_button_ptr_);
  // hLayoutMid->addWidget(stop_button_ptr_);
  // hLayoutMid->addStretch();

  // vLayout->addLayout(hLayoutMid);

  auto * hLayoutBottom = new QHBoxLayout;
  hLayoutBottom->addWidget(auto_button_ptr_);
  hLayoutBottom->addWidget(local_button_ptr_);
  hLayoutBottom->addWidget(remote_button_ptr_);
  hLayoutBottom->addWidget(stop_button_ptr_);

  vLayout->addLayout(hLayoutBottom);

  group->setLayout(vLayout);
  return group;
}

QVBoxLayout * AutowareStatePanel::makeRoutingGroup()
{
  auto * group = new QVBoxLayout;
  auto * h_layout = new QHBoxLayout;

  routing_label_ptr_ = new QLabel;
  routing_label_ptr_->setFixedSize(40, 40);
  routing_label_ptr_->setStyleSheet(
    "border-radius: 20;border: 2px solid #00e678;background-color: #00e678");

  clear_route_button_ptr_ = new QPushButton("Clear Route");
  clear_route_button_ptr_->setFixedWidth(
    clear_route_button_ptr_->fontMetrics().horizontalAdvance("CLEAR EMERGENCY") + 20);
  clear_route_button_ptr_->setCheckable(true);
  clear_route_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(clear_route_button_ptr_, SIGNAL(clicked()), SLOT(onClickClearRoute()));

  auto * routing_label_title_ptr_ = new QLabel("Routing State: ");

  h_layout->addWidget(routing_label_title_ptr_);
  h_layout->addStretch();
  h_layout->addWidget(clear_route_button_ptr_);
  h_layout->addWidget(routing_label_ptr_);

  h_layout->setAlignment(Qt::AlignTop);

  group->addLayout(h_layout);
  return group;
}

QVBoxLayout * AutowareStatePanel::makeLocalizationGroup()
{
  auto * group = new QVBoxLayout;
  auto * h_layout = new QHBoxLayout;

  init_by_gnss_button_ptr_ = new QPushButton("Init by GNSS");
  init_by_gnss_button_ptr_->setFixedWidth(
    init_by_gnss_button_ptr_->fontMetrics().horizontalAdvance("CLEAR EMERGENCY") + 20);
  init_by_gnss_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(init_by_gnss_button_ptr_, SIGNAL(clicked()), SLOT(onClickInitByGnss()));

  localization_label_ptr_ = new QLabel;
  localization_label_ptr_->setFixedSize(40, 40);
  localization_label_ptr_->setStyleSheet(
    "border-radius: 20;border: 2px solid #00e678;background-color: #00e678");

  auto * localization_label_title_ptr_ = new QLabel("Localization State: ");

  h_layout->addWidget(localization_label_title_ptr_);
  h_layout->addStretch();
  h_layout->addWidget(init_by_gnss_button_ptr_);
  h_layout->addWidget(localization_label_ptr_);

  group->addLayout(h_layout);
  return group;
}

QVBoxLayout * AutowareStatePanel::makeMotionGroup()
{
  auto * group = new QVBoxLayout;
  auto * h_layout = new QHBoxLayout;

  accept_start_button_ptr_ = new QPushButton("Accept Start");
  accept_start_button_ptr_->setFixedWidth(
    accept_start_button_ptr_->fontMetrics().horizontalAdvance("CLEAR EMERGENCY") + 20);
  accept_start_button_ptr_->setCheckable(true);
  accept_start_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(accept_start_button_ptr_, SIGNAL(clicked()), SLOT(onClickAcceptStart()));

  motion_label_ptr_ = new QLabel;
  motion_label_ptr_->setFixedSize(40, 40);
  motion_label_ptr_->setStyleSheet(
    "border-radius: 20;border: 2px solid #00e678;background-color: #00e678");

  auto * motion_label_title_ptr_ = new QLabel("Motion State: ");

  h_layout->addWidget(motion_label_title_ptr_);
  h_layout->addStretch();
  h_layout->addWidget(accept_start_button_ptr_);
  h_layout->addWidget(motion_label_ptr_);

  group->addLayout(h_layout);
  return group;
}

QVBoxLayout * AutowareStatePanel::makeFailSafeGroup()
{
  auto * group = new QVBoxLayout;
  auto * v_layout = new QVBoxLayout;
  auto * mrm_state_layout = new QHBoxLayout;
  auto * mrm_behavior_layout = new QHBoxLayout;

  mrm_state_label_ptr_ = new QPushButton("INIT");
  mrm_state_label_ptr_->setDisabled(true);
  mrm_state_label_ptr_->setFixedWidth(
    mrm_state_label_ptr_->fontMetrics().horizontalAdvance("CLEAR EMERGENCY") + 65);

  mrm_behavior_label_ptr_ = new QPushButton("INIT");
  mrm_behavior_label_ptr_->setDisabled(true);
  mrm_behavior_label_ptr_->setFixedWidth(
    mrm_behavior_label_ptr_->fontMetrics().horizontalAdvance("CLEAR EMERGENCY") + 65);

  auto * mrm_state_label_title_ptr_ = new QLabel("MRM State: ");
  auto * mrm_behavior_label_title_ptr_ = new QLabel("MRM Behavior: ");

  mrm_state_layout->addWidget(mrm_state_label_title_ptr_);
  mrm_state_layout->addStretch();
  mrm_state_layout->addWidget(mrm_state_label_ptr_);

  mrm_behavior_layout->addWidget(mrm_behavior_label_title_ptr_);
  mrm_behavior_layout->addStretch();
  mrm_behavior_layout->addWidget(mrm_behavior_label_ptr_);

  v_layout->addLayout(mrm_state_layout);
  v_layout->addSpacing(5);
  v_layout->addLayout(mrm_behavior_layout);

  group->addLayout(v_layout);
  return group;
}

QVBoxLayout * AutowareStatePanel::makeDiagnosticGroup()
{
  auto * group = new QVBoxLayout;

  // Create the scroll area
  QScrollArea * scrollArea = new QScrollArea;
  scrollArea->setFixedHeight(100);  // Adjust the height as needed
  scrollArea->setWidgetResizable(true);
  scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  // Create a widget to contain the layout
  QWidget * scrollAreaWidgetContents = new QWidget;
  // use layout to contain the diagnostic label and the diagnostic level
  diagnostic_layout_ = new QVBoxLayout();
  diagnostic_layout_->setSpacing(5);                   // Set space between items
  diagnostic_layout_->setContentsMargins(5, 5, 5, 5);  // Set margins within the layout

  // Add a QLabel to display the title of what this is
  auto * tsm_label_title_ptr_ = new QLabel("Topic State Monitor: ");
  // Set the layout on the widget
  scrollAreaWidgetContents->setLayout(diagnostic_layout_);

  // Set the widget on the scroll area
  scrollArea->setWidget(scrollAreaWidgetContents);

  group->addWidget(tsm_label_title_ptr_);
  group->addWidget(scrollArea);

  return group;
}

QGroupBox * AutowareStatePanel::makeVelocityLimitGroup()
{
  // Velocity Limit
  velocity_limit_setter_ptr_ = new QLabel("Set Velocity Limit");
  // set its width to fit the text
  velocity_limit_setter_ptr_->setFixedWidth(
    velocity_limit_setter_ptr_->fontMetrics().horizontalAdvance("Set Velocity Limit"));

  velocity_limit_value_label_ = new QLabel("0");
  // max width is set to fit the text
  velocity_limit_value_label_->setMaximumWidth(
    velocity_limit_value_label_->fontMetrics().horizontalAdvance("999"));

  CustomSlider * pub_velocity_limit_slider_ = new CustomSlider(Qt::Horizontal);
  pub_velocity_limit_slider_->setRange(0, 100);
  pub_velocity_limit_slider_->setValue(0);
  pub_velocity_limit_slider_->setMaximumWidth(100);

  connect(pub_velocity_limit_slider_, &QSlider::sliderPressed, this, [this]() {
    sliderIsDragging = true;  // User starts dragging the handle
  });

  connect(pub_velocity_limit_slider_, &QSlider::sliderReleased, this, [this]() {
    sliderIsDragging = false;  // User finished dragging
    onClickVelocityLimit();    // Call when handle is released after dragging
  });

  connect(pub_velocity_limit_slider_, &QSlider::valueChanged, this, [this](int value) {
    this->velocity_limit_value_label_->setText(QString::number(value));
    if (!sliderIsDragging) {   // If the value changed without dragging, it's a click on the track
      onClickVelocityLimit();  // Call the function immediately since it's not a drag operation
    }
  });

  QLabel * velocity_limit_label = new QLabel("km/h");

  // Emergency Button
  emergency_button_ptr_ = new QPushButton("Set Emergency");
  emergency_button_ptr_->setCursor(Qt::PointingHandCursor);
  // set fixed width to fit the text
  emergency_button_ptr_->setFixedWidth(
    emergency_button_ptr_->fontMetrics().horizontalAdvance("CLEAR EMERGENCY") + 40);
  connect(emergency_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickEmergencyButton()));
  auto * utility_layout = new QVBoxLayout;
  auto * velocity_limit_layout = new QHBoxLayout;
  auto * emergency_layout = new QHBoxLayout;

  // Velocity Limit layout
  velocity_limit_layout->addWidget(velocity_limit_setter_ptr_);
  velocity_limit_layout->addWidget(pub_velocity_limit_slider_);
  velocity_limit_layout->addWidget(velocity_limit_value_label_);
  velocity_limit_layout->addWidget(velocity_limit_label);
  velocity_limit_layout->addStretch();

  // Emergency layout
  emergency_layout->addStretch();
  emergency_layout->addWidget(emergency_button_ptr_);

  utility_layout->addLayout(velocity_limit_layout);
  utility_layout->addLayout(emergency_layout);

  auto * velocity_limit_group = new QGroupBox("Utility");
  velocity_limit_group->setLayout(utility_layout);

  return velocity_limit_group;
}

void AutowareStatePanel::onInitialize()
{
  using std::placeholders::_1;

  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Operation Mode
  sub_operation_mode_ = raw_node_->create_subscription<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareStatePanel::onOperationMode, this, _1));

  client_change_to_autonomous_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_autonomous");

  client_change_to_stop_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_stop");

  client_change_to_local_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_local");

  client_change_to_remote_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_remote");

  client_enable_autoware_control_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/enable_autoware_control");

  client_enable_direct_control_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/disable_autoware_control");

  // Routing
  sub_route_ = raw_node_->create_subscription<RouteState>(
    "/api/routing/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareStatePanel::onRoute, this, _1));

  client_clear_route_ = raw_node_->create_client<ClearRoute>("/api/routing/clear_route");

  // Localization
  sub_localization_ = raw_node_->create_subscription<LocalizationInitializationState>(
    "/api/localization/initialization_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareStatePanel::onLocalization, this, _1));

  client_init_by_gnss_ =
    raw_node_->create_client<InitializeLocalization>("/api/localization/initialize");

  // Motion
  sub_motion_ = raw_node_->create_subscription<MotionState>(
    "/api/motion/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareStatePanel::onMotion, this, _1));

  client_accept_start_ = raw_node_->create_client<AcceptStart>("/api/motion/accept_start");

  // FailSafe
  sub_mrm_ = raw_node_->create_subscription<MRMState>(
    "/api/fail_safe/mrm_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareStatePanel::onMRMState, this, _1));

  // Diagnostics
  sub_diag_ = raw_node_->create_subscription<DiagnosticArray>(
    "/diagnostics", 10, std::bind(&AutowareStatePanel::onDiagnostics, this, _1));

  sub_emergency_ = raw_node_->create_subscription<tier4_external_api_msgs::msg::Emergency>(
    "/api/autoware/get/emergency", 10, std::bind(&AutowareStatePanel::onEmergencyStatus, this, _1));

  client_emergency_stop_ = raw_node_->create_client<tier4_external_api_msgs::srv::SetEmergency>(
    "/api/autoware/set/emergency");

  pub_velocity_limit_ = raw_node_->create_publisher<tier4_planning_msgs::msg::VelocityLimit>(
    "/planning/scenario_planning/max_velocity_default", rclcpp::QoS{1}.transient_local());
}

void AutowareStatePanel::onOperationMode(const OperationModeState::ConstSharedPtr msg)
{
  auto changeButtonState = [this](
                             QPushButton * button, const bool is_desired_mode_available,
                             const uint8_t current_mode = OperationModeState::UNKNOWN,
                             const uint8_t desired_mode = OperationModeState::STOP) {
    if (is_desired_mode_available && current_mode != desired_mode) {
      activateButton(button);
      button->setStyleSheet(
        "QPushButton {"
        "background-color: #303030;color: #ffffff;"
        "border: 2px solid #00E678;"
        "font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "background-color: rgba(0, 230, 120, 0.2);"
        "color: #ffffff;"
        "border: 2px solid #00E678;"
        "}");
    } else {
      button->setStyleSheet(
        "QPushButton {"
        "background-color: #00E678;color: black;"
        "border: 2px solid #303030;"
        "font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "background-color: rgba(0, 230, 120, 0.2);"
        "color: #ffffff;"
        "border: 2px solid #303030;"
        "}");
      deactivateButton(button);
    }
  };

  auto changeToggleSwitchState = [this](CustomToggleSwitch * toggle_switch, const bool is_enabled) {
    bool oldState = toggle_switch->blockSignals(true);  // Block signals
    toggle_switch->setChecked(!is_enabled);
    toggle_switch->blockSignals(oldState);  // Restore original signal blocking state
  };

  // Button
  changeButtonState(
    auto_button_ptr_, msg->is_autonomous_mode_available, msg->mode, OperationModeState::AUTONOMOUS);
  changeButtonState(
    stop_button_ptr_, msg->is_stop_mode_available, msg->mode, OperationModeState::STOP);
  changeButtonState(
    local_button_ptr_, msg->is_local_mode_available, msg->mode, OperationModeState::LOCAL);
  changeButtonState(
    remote_button_ptr_, msg->is_remote_mode_available, msg->mode, OperationModeState::REMOTE);

  // toggle switch for control mode
  changeToggleSwitchState(control_mode_switch_ptr_, !msg->is_autoware_control_enabled);
}

void AutowareStatePanel::onRoute(const RouteState::ConstSharedPtr msg)
{
  QString text = "";
  QString style_sheet = "";
  switch (msg->state) {
    case RouteState::UNSET:
      text = "UNSET";
      style_sheet =
        "border-radius: 20;border: 2px solid #f0e130;background-color: #f0e130";  // yellow
      break;

    case RouteState::SET:
      text = "SET";
      style_sheet =
        "border-radius: 20;border: 2px solid #00e678;background-color: #00e678";  // green
      break;

    case RouteState::ARRIVED:
      text = "ARRIVED";
      style_sheet =
        "border-radius: 20;border: 2px solid #ffae42;background-color: #ffae42";  // orange
      break;

    case RouteState::CHANGING:
      text = "CHANGING";
      style_sheet = "border-radius: 20;border: 2px solid #f0e130;background-color: #f0e130";
      break;

    default:
      text = "UNKNOWN";
      style_sheet = "border-radius: 20;border: 2px solid #dc3545;background-color: #dc3545";  // red
      break;
  }

  updateLabel(routing_label_ptr_, "", style_sheet);

  if (msg->state == RouteState::SET) {
    activateButton(clear_route_button_ptr_);
  } else {
    deactivateButton(clear_route_button_ptr_);
  }
}

void AutowareStatePanel::onLocalization(const LocalizationInitializationState::ConstSharedPtr msg)
{
  QString text = "";
  QString style_sheet = "";
  switch (msg->state) {
    case LocalizationInitializationState::UNINITIALIZED:
      text = "UNINITIALIZED";
      style_sheet =
        "border-radius: 20;border: 2px solid #f0e130;background-color: #f0e130";  // yellow
      break;

    case LocalizationInitializationState::INITIALIZING:
      text = "INITIALIZING";
      style_sheet =
        "border-radius: 20;border: 2px solid #ffae42;background-color: #ffae42";  // orange
      break;

    case LocalizationInitializationState::INITIALIZED:
      text = "INITIALIZED";
      style_sheet =
        "border-radius: 20;border: 2px solid #00e678;background-color: #00e678";  // green
      break;

    default:
      text = "UNKNOWN";
      style_sheet = "border-radius: 20;border: 2px solid #dc3545;background-color: #dc3545";  // red
      break;
  }

  updateLabel(localization_label_ptr_, "", style_sheet);
}

void AutowareStatePanel::onMotion(const MotionState::ConstSharedPtr msg)
{
  QString text = "";
  QString style_sheet = "";
  switch (msg->state) {
    case MotionState::STARTING:
      text = "STARTING";
      style_sheet =
        "border-radius: 20;border: 2px solid #f0e130;background-color: #f0e130";  // yellow
      break;

    case MotionState::STOPPED:
      text = "STOPPED";
      style_sheet =
        "border-radius: 20;border: 2px solid #dc3545;background-color: #dc3545";  // red color
      break;

    case MotionState::MOVING:
      text = "MOVING";
      style_sheet =
        "border-radius: 20;border: 2px solid #00e678;background-color: #00e678";  // green
      break;

    default:
      text = "UNKNOWN";
      style_sheet =
        "border-radius: 20;border: 2px solid #ffae42;background-color: #ffae42";  // orange
      break;
  }

  updateLabel(motion_label_ptr_, "", style_sheet);

  if (msg->state == MotionState::STARTING) {
    activateButton(accept_start_button_ptr_);
  } else {
    deactivateButton(accept_start_button_ptr_);
  }
}

void AutowareStatePanel::onMRMState(const MRMState::ConstSharedPtr msg)
{
  // state
  {
    QString text = "";
    QString style_sheet = "";
    switch (msg->state) {
      case MRMState::NONE:
        text = "NONE";
        style_sheet =
          "background-color: #00E678;color: black;font-weight: bold;font-size: 12px;";  // green
        break;

      case MRMState::MRM_OPERATING:
        text = "MRM_OPERATING";
        style_sheet =
          "background-color: #FFAE42;color: black;font-weight: bold;font-size: 12px;";  // orange
        break;

      case MRMState::MRM_SUCCEEDED:
        text = "MRM_SUCCEEDED";
        style_sheet =
          "background-color: #F0E130;color: black;font-weight: bold;font-size: 12px;";  // yellow
        break;

      case MRMState::MRM_FAILED:
        text = "MRM_FAILED";
        style_sheet =
          "background-color: #dc3545;color: black;font-weight: bold;font-size: 12px;";  // red
        break;

      default:
        text = "UNKNOWN";
        style_sheet =
          "background-color: #dc3545;color: black;font-weight: bold;font-size: 12px;";  // red
        break;
    }

    updateButton(mrm_state_label_ptr_, text, style_sheet);
  }

  // behavior
  {
    QString text = "";
    QString style_sheet = "";
    switch (msg->behavior) {
      case MRMState::NONE:
        text = "NONE";
        style_sheet =
          "background-color: #00E678;color: black;font-weight: bold;font-size: 12px;";  // green
        break;

      case MRMState::PULL_OVER:
        text = "PULL_OVER";
        style_sheet =
          "background-color: #F0E130;color: black;font-weight: bold;font-size: 12px;";  // yellow
        break;

      case MRMState::COMFORTABLE_STOP:
        text = "COMFORTABLE_STOP";
        style_sheet =
          "background-color: #F0E130;color: black;font-weight: bold;font-size: 12px;";  // yellow
        break;

      case MRMState::EMERGENCY_STOP:
        text = "EMERGENCY_STOP";
        style_sheet =
          "background-color: #FFAE42;color: black;font-weight: bold;font-size: 12px;";  // orange
        break;

      default:
        text = "UNKNOWN";
        style_sheet =
          "background-color: #dc3545;color: black;font-weight: bold;font-size: 12px;";  // red
        break;
    }

    updateButton(mrm_behavior_label_ptr_, text, style_sheet);
  }
}

void AutowareStatePanel::onDiagnostics(const DiagnosticArray::ConstSharedPtr msg)
{
  for (const auto & status : msg->status) {
    std::string statusName = status.name;  // Assuming name is a std::string
    int level = status.level;              // Assuming level is an int

    // Check if this status name already has an associated QLabel
    auto it = statusLabels.find(statusName);
    if (it != statusLabels.end()) {
      // Status exists, update its display (QLabel) with the new level
      updateStatusLabel(statusName, level);
    } else {
      // New status, add a QLabel for it to the map and the layout
      addStatusLabel(statusName, level);
    }
  }
}

void AutowareStatePanel::addStatusLabel(const std::string & name, int level)
{
  QString baseStyle =
    "QLabel {"
    "  border-radius: 4px;"
    "  padding: 4px;"
    "  margin: 2px;"
    "  font-weight: bold;"
    "  color: #000;";

  QString okStyle = baseStyle + "background-color: #00e678;}";
  QString warnStyle = baseStyle + "background-color: #FFCC00;}";
  QString errorStyle = baseStyle + "background-color: #dc3545;}";
  QString staleStyle = baseStyle + "background-color: #6c757d;}";

  QString labelText = QString::fromStdString(name);
  // + ": " +
  //                     (level == diagnostic_msgs::msg::DiagnosticStatus::OK      ? "OK"
  //                      : level == diagnostic_msgs::msg::DiagnosticStatus::WARN  ? "WARN"
  //                      : level == diagnostic_msgs::msg::DiagnosticStatus::ERROR ? "ERROR"
  //                                                                               : "STALE");

  auto * label = new QLabel(labelText);
  label->setMinimumHeight(30);  // for example, set a minimum height
  label->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);

  // Adjust style based on level
  QString styleSheet;
  switch (level) {
    case diagnostic_msgs::msg::DiagnosticStatus::OK:
      styleSheet = okStyle;
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::WARN:
      styleSheet = warnStyle;
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
      styleSheet = errorStyle;
      break;
    default:
      styleSheet = staleStyle;
      break;
  }

  label->setStyleSheet(styleSheet);
  diagnostic_layout_->addWidget(label);
  statusLabels[name] = label;
}

void AutowareStatePanel::updateStatusLabel(const std::string & name, int level)
{
  QString baseStyle =
    "QLabel {"
    "  border-radius: 4px;"
    "  padding: 4px;"
    "  margin: 2px;"
    "  font-weight: bold;"
    "  color: #000;";

  QString okStyle = baseStyle + "background-color: #00e678;}";
  QString warnStyle = baseStyle + "background-color: #FFCC00;}";
  QString errorStyle = baseStyle + "background-color: #dc3545;}";
  QString staleStyle = baseStyle + "background-color: #6c757d;}";

  // Find the QLabel* associated with this status name
  auto it = statusLabels.find(name);
  if (it != statusLabels.end()) {
    QLabel * label = it->second;

    // Update label's text
    QString labelText = QString::fromStdString(name);
    // +": " + (level == diagnostic_msgs::msg::DiagnosticStatus::OK      ? "OK"
    //          : level == diagnostic_msgs::msg::DiagnosticStatus::WARN  ? "WARN"
    //          : level == diagnostic_msgs::msg::DiagnosticStatus::ERROR ? "ERROR"
    //                                                                   : "STALE");
    label->setText(labelText);

    // Update style based on level, similar to addStatusLabel
    QString styleSheet;
    switch (level) {
      case diagnostic_msgs::msg::DiagnosticStatus::OK:
        styleSheet = okStyle;
        break;
      case diagnostic_msgs::msg::DiagnosticStatus::WARN:
        styleSheet = warnStyle;
        break;
      case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
        styleSheet = errorStyle;
        break;
      default:
        styleSheet = staleStyle;
        break;
    }
    label->setStyleSheet(styleSheet);
    label->adjustSize();  // Adjust the size of the label to fit the content if needed
    label->update();      // Ensure the label is updated immediately
  }
}

void AutowareStatePanel::onEmergencyStatus(
  const tier4_external_api_msgs::msg::Emergency::ConstSharedPtr msg)
{
  current_emergency_ = msg->emergency;
  if (msg->emergency) {
    emergency_button_ptr_->setText(QString::fromStdString("Clear Emergency"));
    emergency_button_ptr_->setStyleSheet(
      "background-color: #dc3545;color: black;font-weight: bold;border: 2px solid #dc3545");
  } else {
    emergency_button_ptr_->setText(QString::fromStdString("Set Emergency"));
    emergency_button_ptr_->setStyleSheet(
      "background-color: #00E678;color: black;font-weight: bold");
  }
}

void AutowareStatePanel::onSwitchStateChanged(int state)
{
  if (state == 0) {
    // call the control mode function
    onClickDirectControl();
  } else if (state == 2) {
    onClickAutowareControl();
  }
}

void AutowareStatePanel::onClickVelocityLimit()
{
  auto velocity_limit = std::make_shared<tier4_planning_msgs::msg::VelocityLimit>();
  velocity_limit->max_velocity = velocity_limit_value_label_->text().toDouble() / 3.6;
  pub_velocity_limit_->publish(*velocity_limit);
}

void AutowareStatePanel::onClickAutonomous()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_change_to_autonomous_);
}
void AutowareStatePanel::onClickStop()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_change_to_stop_);
}
void AutowareStatePanel::onClickLocal()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_change_to_local_);
}
void AutowareStatePanel::onClickRemote()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_change_to_remote_);
}
void AutowareStatePanel::onClickAutowareControl()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_enable_autoware_control_);
}
void AutowareStatePanel::onClickDirectControl()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_enable_direct_control_);
}

void AutowareStatePanel::onClickClearRoute()
{
  callServiceWithoutResponse<ClearRoute>(client_clear_route_);
}

void AutowareStatePanel::onClickInitByGnss()
{
  callServiceWithoutResponse<InitializeLocalization>(client_init_by_gnss_);
}

void AutowareStatePanel::onClickAcceptStart()
{
  callServiceWithoutResponse<AcceptStart>(client_accept_start_);
}

void AutowareStatePanel::onClickEmergencyButton()
{
  using tier4_external_api_msgs::msg::ResponseStatus;
  using tier4_external_api_msgs::srv::SetEmergency;

  auto request = std::make_shared<SetEmergency::Request>();
  request->emergency = !current_emergency_;

  RCLCPP_INFO(raw_node_->get_logger(), request->emergency ? "Set Emergency" : "Clear Emergency");

  client_emergency_stop_->async_send_request(
    request, [this](rclcpp::Client<SetEmergency>::SharedFuture result) {
      const auto & response = result.get();
      if (response->status.code == ResponseStatus::SUCCESS) {
        RCLCPP_INFO(raw_node_->get_logger(), "service succeeded");
      } else {
        RCLCPP_WARN(
          raw_node_->get_logger(), "service failed: %s", response->status.message.c_str());
      }
    });
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowareStatePanel, rviz_common::Panel)
