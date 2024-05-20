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

#include "include/autoware_state_panel.hpp"

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
  this->setMinimumWidth(420);
  this->setMaximumWidth(420);

  // Layout

  // Create a new container widget
  QWidget * containerWidget = new QWidget(this);
  containerWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  containerWidget->setStyleSheet("QWidget { background-color: #0F1417; color: #d0e6f2; }");

  auto * containerLayout = new QVBoxLayout(containerWidget);
  // Set the alignment of the layout
  containerLayout->setAlignment(Qt::AlignTop);
  containerLayout->setSpacing(1);

  auto * operation_mode_group = makeOperationModeGroup();
  auto * diagnostic_v_layout = new QVBoxLayout;
  auto * localization_group = makeLocalizationGroup();
  auto * motion_group = makeMotionGroup();
  auto * fail_safe_group = makeFailSafeGroup();
  auto * routing_group = makeRoutingGroup();
  auto * velocity_limit_group = makeVelocityLimitGroup();
  // auto * diagnostic_group = makeDiagnosticGroup();

  diagnostic_v_layout->addLayout(routing_group);
  // diagnostic_v_layout->addSpacing(5);
  diagnostic_v_layout->addLayout(localization_group);
  // diagnostic_v_layout->addSpacing(5);
  diagnostic_v_layout->addLayout(motion_group);
  // diagnostic_v_layout->addSpacing(5);
  diagnostic_v_layout->addLayout(fail_safe_group);

  // containerLayout->addLayout(diagnostic_group);

  containerLayout->addLayout(operation_mode_group);
  // containerLayout->addSpacing(5);
  containerLayout->addLayout(diagnostic_v_layout);
  // main_v_layout->addSpacing(5);
  containerLayout->addLayout(velocity_limit_group);

  // Main layout for AutowareStatePanel
  QVBoxLayout * mainLayout = new QVBoxLayout(this);
  mainLayout->addWidget(containerWidget);
  setLayout(mainLayout);
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

  // // Diagnostics
  // sub_diag_ = raw_node_->create_subscription<DiagnosticArray>(
  //   "/diagnostics", 10, std::bind(&AutowareStatePanel::onDiagnostics, this, _1));

  sub_emergency_ = raw_node_->create_subscription<tier4_external_api_msgs::msg::Emergency>(
    "/api/autoware/get/emergency", 10, std::bind(&AutowareStatePanel::onEmergencyStatus, this, _1));

  client_emergency_stop_ = raw_node_->create_client<tier4_external_api_msgs::srv::SetEmergency>(
    "/api/autoware/set/emergency");

  pub_velocity_limit_ = raw_node_->create_publisher<tier4_planning_msgs::msg::VelocityLimit>(
    "/planning/scenario_planning/max_velocity_default", rclcpp::QoS{1}.transient_local());

  QObject::connect(segmented_button, &CustomSegmentedButton::buttonClicked, this, [this](int id) {
    const QList<QAbstractButton *> buttons = segmented_button->getButtonGroup()->buttons();

    // Check if the button ID is within valid range
    if (id < 0 || id >= buttons.size()) {
      return;
    }

    // Ensure the button is not null
    QAbstractButton * abstractButton = segmented_button->getButtonGroup()->button(id);
    if (!abstractButton) {
      return;
    }

    QPushButton * button = qobject_cast<QPushButton *>(abstractButton);
    if (button) {
      // Call the corresponding function for each button
      if (button == auto_button_ptr_) {
        onClickAutonomous();
      } else if (button == local_button_ptr_) {
        onClickLocal();
      } else if (button == remote_button_ptr_) {
        onClickRemote();
      } else if (button == stop_button_ptr_) {
        onClickStop();
      }
    } else {
      // qDebug() << "Button not found with ID:" << id;
    }
  });
}

QVBoxLayout * AutowareStatePanel::makeOperationModeGroup()
{
  control_mode_switch_ptr_ = new CustomToggleSwitch(this);
  connect(
    control_mode_switch_ptr_, &QCheckBox::stateChanged, this,
    &AutowareStatePanel::onSwitchStateChanged);

  control_mode_label_ptr_ = new QLabel("Autoware Control");
  control_mode_label_ptr_->setStyleSheet("color: #d0e6f2; font-weight: bold;");

  CustomContainer * group1 = new CustomContainer(this);

  auto * horizontal_layout = new QHBoxLayout;
  horizontal_layout->setSpacing(10);
  horizontal_layout->setContentsMargins(0, 0, 0, 0);

  horizontal_layout->addWidget(control_mode_switch_ptr_);
  horizontal_layout->addWidget(control_mode_label_ptr_);

  // add switch and label to the container
  group1->setContentsMargins(0, 0, 0, 10);
  group1->getLayout()->addLayout(horizontal_layout, 0, 0, 1, 1, Qt::AlignLeft);

  // Create the CustomSegmentedButton
  segmented_button = new CustomSegmentedButton(this);
  auto_button_ptr_ = segmented_button->addButton("Auto");
  local_button_ptr_ = segmented_button->addButton("Local");
  remote_button_ptr_ = segmented_button->addButton("Remote");
  stop_button_ptr_ = segmented_button->addButton("Stop");

  QVBoxLayout * groupLayout = new QVBoxLayout;
  // set these widgets to show up at the left and not stretch more than needed
  groupLayout->setAlignment(Qt::AlignCenter);
  groupLayout->setContentsMargins(10, 0, 0, 0);
  groupLayout->addWidget(group1);
  // groupLayout->addSpacing(5);
  groupLayout->addWidget(segmented_button, 0, Qt::AlignCenter);
  return groupLayout;
}

QVBoxLayout * AutowareStatePanel::makeRoutingGroup()
{
  auto * group = new QVBoxLayout;

  auto * custom_container = new CustomContainer(this);

  routing_icon = new CustomIconLabel(QColor("#84c2e6"));

  clear_route_button_ptr_ = new CustomElevatedButton("Clear Route");
  clear_route_button_ptr_->setCheckable(true);
  clear_route_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(clear_route_button_ptr_, SIGNAL(clicked()), SLOT(onClickClearRoute()));

  QLabel * routing_label = new QLabel("Routing");
  routing_label->setStyleSheet("color: #d0e6f2; font-weight: bold;");

  auto * horizontal_layout = new QHBoxLayout;
  horizontal_layout->setSpacing(10);
  horizontal_layout->setContentsMargins(0, 0, 0, 0);

  horizontal_layout->addWidget(routing_icon);
  horizontal_layout->addWidget(routing_label);

  custom_container->getLayout()->addLayout(horizontal_layout, 0, 0, 1, 1, Qt::AlignLeft);
  custom_container->getLayout()->addWidget(clear_route_button_ptr_, 0, 2, 1, 4, Qt::AlignRight);

  custom_container->setContentsMargins(10, 0, 0, 0);

  group->addWidget(custom_container);

  return group;
}

QVBoxLayout * AutowareStatePanel::makeLocalizationGroup()
{
  auto * group = new QVBoxLayout;
  auto * custom_container = new CustomContainer(this);

  init_by_gnss_button_ptr_ = new CustomElevatedButton("Initialize with GNSS");
  init_by_gnss_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(init_by_gnss_button_ptr_, SIGNAL(clicked()), SLOT(onClickInitByGnss()));

  localization_icon = new CustomIconLabel(QColor("#84c2e6"));
  QLabel * localization_label = new QLabel("Localization");
  localization_label->setStyleSheet("color: #d0e6f2; font-weight: bold;");

  auto * horizontal_layout = new QHBoxLayout;
  horizontal_layout->setSpacing(10);
  horizontal_layout->setContentsMargins(0, 0, 0, 0);

  horizontal_layout->addWidget(localization_icon);
  horizontal_layout->addWidget(localization_label);

  custom_container->getLayout()->addLayout(horizontal_layout, 0, 0, 1, 1, Qt::AlignLeft);
  custom_container->getLayout()->addWidget(init_by_gnss_button_ptr_, 0, 2, 1, 4, Qt::AlignRight);

  custom_container->setContentsMargins(10, 0, 0, 0);

  group->addWidget(custom_container);
  return group;
}

QVBoxLayout * AutowareStatePanel::makeMotionGroup()
{
  auto * group = new QVBoxLayout;
  auto * custom_container = new CustomContainer(this);

  accept_start_button_ptr_ = new CustomElevatedButton("Accept Start");
  accept_start_button_ptr_->setCheckable(true);
  accept_start_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(accept_start_button_ptr_, SIGNAL(clicked()), SLOT(onClickAcceptStart()));

  motion_icon = new CustomIconLabel(QColor("#84c2e6"));
  QLabel * motion_label = new QLabel("Motion");
  motion_label->setStyleSheet("color: #d0e6f2; font-weight: bold;");

  auto * horizontal_layout = new QHBoxLayout;
  horizontal_layout->setSpacing(10);
  horizontal_layout->setContentsMargins(0, 0, 0, 0);
  horizontal_layout->setAlignment(Qt::AlignLeft);

  horizontal_layout->addWidget(motion_icon);
  horizontal_layout->addWidget(motion_label);

  custom_container->getLayout()->addLayout(horizontal_layout, 0, 0, 1, 1, Qt::AlignLeft);
  custom_container->getLayout()->addWidget(accept_start_button_ptr_, 0, 2, 1, 4, Qt::AlignRight);

  custom_container->setContentsMargins(10, 0, 0, 0);

  group->addWidget(custom_container);

  return group;
}

QVBoxLayout * AutowareStatePanel::makeFailSafeGroup()
{
  auto * group = new QVBoxLayout;
  auto * v_layout = new QVBoxLayout;
  auto * custom_container1 = new CustomContainer(this);
  auto * custom_container2 = new CustomContainer(this);

  mrm_state_icon = new CustomIconLabel(QColor("#84c2e6"));
  mrm_behavior_icon = new CustomIconLabel(QColor("#84c2e6"));

  mrm_state_label_ptr_ = new QLabel("MRM State | Unknown");
  mrm_behavior_label_ptr_ = new QLabel("MRM Behavior | Unknown");

  // change text color
  mrm_state_label_ptr_->setStyleSheet("color: #d0e6f2; font-weight: bold;");
  mrm_behavior_label_ptr_->setStyleSheet("color: #d0e6f2; font-weight: bold;");

  auto * horizontal_layout = new QHBoxLayout;
  horizontal_layout->setSpacing(10);
  horizontal_layout->setContentsMargins(0, 0, 0, 0);

  horizontal_layout->addWidget(mrm_state_icon);
  horizontal_layout->addWidget(mrm_state_label_ptr_);

  custom_container1->getLayout()->addLayout(horizontal_layout, 0, 0, 1, 1, Qt::AlignLeft);

  auto * horizontal_layout2 = new QHBoxLayout;
  horizontal_layout2->setSpacing(10);
  horizontal_layout2->setContentsMargins(0, 0, 0, 0);

  horizontal_layout2->addWidget(mrm_behavior_icon);
  horizontal_layout2->addWidget(mrm_behavior_label_ptr_);

  custom_container2->getLayout()->addLayout(horizontal_layout2, 0, 0, 1, 1, Qt::AlignLeft);

  v_layout->addWidget(custom_container1);
  // v_layout->addSpacing(5);
  v_layout->addWidget(custom_container2);

  group->setContentsMargins(10, 0, 0, 0);

  group->addLayout(v_layout);
  return group;
}

/* QVBoxLayout * AutowareStatePanel::makeDiagnosticGroup()
{
  auto * group = new QVBoxLayout;

  // Create the scroll area
  QScrollArea * scrollArea = new QScrollArea;
  scrollArea->setFixedHeight(66);  // Adjust the height as needed
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
} */

QVBoxLayout * AutowareStatePanel::makeVelocityLimitGroup()
{
  // Velocity Limit
  velocity_limit_setter_ptr_ = new QLabel("Set Velocity Limit");
  // set its width to fit the text
  velocity_limit_setter_ptr_->setFixedWidth(
    velocity_limit_setter_ptr_->fontMetrics().horizontalAdvance("Set Velocity Limit"));

  velocity_limit_value_label_ = new QLabel("0");
  velocity_limit_value_label_->setMaximumWidth(
    velocity_limit_value_label_->fontMetrics().horizontalAdvance("0"));

  CustomSlider * pub_velocity_limit_slider_ = new CustomSlider(Qt::Horizontal);
  pub_velocity_limit_slider_->setRange(0, 100);
  pub_velocity_limit_slider_->setValue(0);
  pub_velocity_limit_slider_->setMaximumWidth(300);

  connect(pub_velocity_limit_slider_, &QSlider::sliderPressed, this, [this]() {
    sliderIsDragging = true;  // User starts dragging the handle
  });

  connect(pub_velocity_limit_slider_, &QSlider::sliderReleased, this, [this]() {
    sliderIsDragging = false;  // User finished dragging
    onClickVelocityLimit();    // Call when handle is released after dragging
  });

  connect(pub_velocity_limit_slider_, &QSlider::valueChanged, this, [this](int value) {
    this->velocity_limit_value_label_->setText(QString::number(value));
    velocity_limit_value_label_->setMaximumWidth(
      velocity_limit_value_label_->fontMetrics().horizontalAdvance(QString::number(value)));
    if (!sliderIsDragging) {   // If the value changed without dragging, it's a click on the track
      onClickVelocityLimit();  // Call the function immediately since it's not a drag operation
    }
  });

  // Emergency Button
  emergency_button_ptr_ = new CustomElevatedButton("Set Emergency");

  emergency_button_ptr_->setCursor(Qt::PointingHandCursor);
  // set fixed width to fit the text
  connect(emergency_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickEmergencyButton()));
  auto * utility_layout = new QVBoxLayout;
  auto * velocity_limit_layout = new QHBoxLayout;
  QLabel * velocity_limit_label = new QLabel("km/h");

  velocity_limit_layout->addWidget(pub_velocity_limit_slider_);
  velocity_limit_layout->addSpacing(5);
  velocity_limit_layout->addWidget(velocity_limit_value_label_);
  velocity_limit_layout->addWidget(velocity_limit_label);

  // Velocity Limit layout
  utility_layout->addSpacing(15);
  utility_layout->addWidget(velocity_limit_setter_ptr_);
  utility_layout->addSpacing(10);
  utility_layout->addLayout(velocity_limit_layout);
  utility_layout->addSpacing(25);
  utility_layout->addWidget(emergency_button_ptr_);

  utility_layout->setContentsMargins(15, 0, 15, 0);

  return utility_layout;
}

void AutowareStatePanel::onOperationMode(const OperationModeState::ConstSharedPtr msg)
{
  auto changeButtonState = [this](
                             CustomSegmentedButtonItem * button,
                             const bool is_desired_mode_available,
                             const uint8_t current_mode = OperationModeState::UNKNOWN,
                             const uint8_t desired_mode = OperationModeState::STOP) {
    button->setHovered(false);  // Reset hover state
    if (is_desired_mode_available) {
      if (current_mode == desired_mode) {
        // Enabled and Checked
        button->setChecked(true);
        button->setActivated(true);
        button->setCheckableButton(false);
        button->setDisabledButton(false);
      } else {
        // Enabled and Checkable
        button->setChecked(false);
        button->setActivated(false);
        button->setCheckableButton(true);
        button->setDisabledButton(false);
      }
    } else {
      // Disabled and Unchecked
      button->setChecked(false);
      button->setActivated(false);
      button->setCheckableButton(false);
      button->setDisabledButton(true);
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

  // routing
  if (msg->is_in_transition) {
    routing_icon->updateStyle(Pending, QColor("#eef08b"));
  }
}

void AutowareStatePanel::onRoute(const RouteState::ConstSharedPtr msg)
{
  IconState state;
  QColor bgColor;

  switch (msg->state) {
    case RouteState::UNSET:
      state = Pending;
      bgColor = QColor("#eef08b");
      break;

    case RouteState::SET:
      state = Active;
      bgColor = QColor("#00e678");
      break;

    case RouteState::ARRIVED:
      state = Danger;
      bgColor = QColor("#f08b8b");
      break;

    case RouteState::CHANGING:
      bgColor = QColor("#eef08b");
      state = Pending;
      break;

    default:
      state = None;
      bgColor = QColor("#84c2e6");
      break;
  }

  routing_icon->updateStyle(state, bgColor);

  if (msg->state == RouteState::SET) {
    activateButton(clear_route_button_ptr_);
  } else {
    clear_route_button_ptr_->setStyleSheet(
      "QPushButton {"
      "background-color: #292d30;color: #6e7276;"
      "border: 2px solid #292d30;"
      "font-weight: bold;"
      "}");
    deactivateButton(clear_route_button_ptr_);
  }
}

void AutowareStatePanel::onLocalization(const LocalizationInitializationState::ConstSharedPtr msg)
{
  IconState state;
  QColor bgColor;

  switch (msg->state) {
    case LocalizationInitializationState::UNINITIALIZED:
      state = None;
      bgColor = QColor("#84c2e6");
      break;

    case LocalizationInitializationState::INITIALIZED:
      state = Active;
      bgColor = QColor("#00e678");
      break;

    case LocalizationInitializationState::INITIALIZING:
      state = Pending;
      bgColor = QColor("#eef08b");
      break;

    default:
      state = None;
      bgColor = QColor("#84c2e6");
      break;
  }

  localization_icon->updateStyle(state, bgColor);
}

void AutowareStatePanel::onMotion(const MotionState::ConstSharedPtr msg)
{
  IconState state;
  QColor bgColor;

  switch (msg->state) {
    case MotionState::STARTING:
      state = Pending;
      bgColor = QColor("#eef08b");
      break;

    case MotionState::MOVING:
      state = Active;
      bgColor = QColor("#00e678");
      break;

    case MotionState::STOPPED:
      state = Danger;
      bgColor = QColor("#f08b8b");
      break;

    default:
      state = None;
      bgColor = QColor("#84c2e6");
      break;
  }

  motion_icon->updateStyle(state, bgColor);

  if (msg->state == MotionState::STARTING) {
    activateButton(accept_start_button_ptr_);
  } else {
    deactivateButton(accept_start_button_ptr_);
  }
}

void AutowareStatePanel::onMRMState(const MRMState::ConstSharedPtr msg)
{
  IconState state;
  QColor bgColor;
  QString mrm_state = "MRM State | Unknown";

  switch (msg->state) {
    case MRMState::NONE:
      state = None;
      bgColor = QColor("#84c2e6");
      mrm_state = "MRM State | Inactive";
      break;

    case MRMState::MRM_OPERATING:
      state = Active;
      bgColor = QColor("#84c2e6");
      mrm_state = "MRM State | Active";
      break;

    case MRMState::MRM_SUCCEEDED:
      state = Active;
      bgColor = QColor("#00e678");
      mrm_state = "MRM State | Successful";
      break;

    case MRMState::MRM_FAILED:
      state = Danger;
      bgColor = QColor("#f08b8b");
      mrm_state = "MRM State | Failed";
      break;

    default:
      state = None;
      bgColor = QColor("#84c2e6");
      mrm_state = "MRM State | Unknown";
      break;
  }

  mrm_state_icon->updateStyle(state, bgColor);
  mrm_state_label_ptr_->setText(mrm_state);

  // behavior
  {
    IconState state;
    QColor bgColor;
    QString mrm_behavior = "MRM Behavior | Unknown";

    switch (msg->behavior) {
      case MRMState::NONE:
        state = Crash;
        bgColor = QColor("#84c2e6");
        mrm_behavior = "MRM Behavior | Inactive";
        break;

      case MRMState::PULL_OVER:
        state = Crash;
        bgColor = QColor("#00e678");
        mrm_behavior = "MRM Behavior | Pull Over";
        break;

      case MRMState::COMFORTABLE_STOP:
        state = Crash;
        bgColor = QColor("#eef08b");
        mrm_behavior = "MRM Behavior | Comfortable Stop";
        break;

      case MRMState::EMERGENCY_STOP:
        state = Crash;
        bgColor = QColor("#f08b8b");
        mrm_behavior = "MRM Behavior | Emergency Stop";
        break;

      default:
        state = Crash;
        bgColor = QColor("#84c2e6");
        mrm_behavior = "MRM Behavior | Unknown";
        break;
    }

    mrm_behavior_icon->updateStyle(state, bgColor);
    mrm_behavior_label_ptr_->setText(mrm_behavior);
  }
}

/* void AutowareStatePanel::onDiagnostics(const DiagnosticArray::ConstSharedPtr msg)
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
} */

/* void AutowareStatePanel::addStatusLabel(const std::string & name, int level)
{
  QString baseStyle =
    "QLabel {"
    "  border-radius: 4px;"
    "  padding: 4px;"
    "  margin: 2px;"
    "  font-weight: bold;"
    "  color: #003546;";

  QString okStyle = baseStyle + "background-color: #84c2e6;}";
  QString warnStyle = baseStyle + "background-color: #FFCC00;}";
  QString errorStyle = baseStyle + "background-color: #f08b8b;}";
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
} */

/* void AutowareStatePanel::updateStatusLabel(const std::string & name, int level)
{
  QString baseStyle =
    "QLabel {"
    "  border-radius: 4px;"
    "  padding: 4px;"
    "  margin: 2px;"
    "  font-weight: bold;"
    "  color: #003546;";

  QString okStyle = baseStyle + "background-color: #84c2e6;}";
  QString warnStyle = baseStyle + "background-color: #FFCC00;}";
  QString errorStyle = baseStyle + "background-color: #f08b8b;}";
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
} */

void AutowareStatePanel::onEmergencyStatus(
  const tier4_external_api_msgs::msg::Emergency::ConstSharedPtr msg)
{
  current_emergency_ = msg->emergency;
  if (msg->emergency) {
    emergency_button_ptr_->updateStyle(
      "Clear Emergency", QColor("#f08b8b"), QColor("#FFDAD6"), QColor("#ed7474"));
  } else {
    emergency_button_ptr_->updateStyle(
      "Set Emergency", QColor("#84c2e6"), QColor("#003546"), QColor("#8bd0f0"));
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
