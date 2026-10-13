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

#include "ml_planner_start_panel.hpp"

#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace autoware::ml_planner_start_rviz_plugin
{
namespace
{
constexpr double kDepartureSpeedMps = 0.2;
constexpr char kServiceName[] =
  "/planning/trajectory_generator/neural_network_based_planner/ml_planner_node/service/start";
constexpr char kKinematicTopic[] = "/localization/kinematic_state";

const char * kButtonStyleIdle =
  "QPushButton { background-color: #2E7D32; color: white; border-radius: 6px; }";
const char * kButtonStyleBusy =
  "QPushButton { background-color: #546E7A; color: white; border-radius: 6px; }";
const char * kButtonStyleWorking =
  "QPushButton { background-color: #F9A825; color: black; border-radius: 6px; }";
const char * kButtonStyleDone =
  "QPushButton { background-color: #1565C0; color: white; border-radius: 6px; }";
const char * kButtonStyleError =
  "QPushButton { background-color: #C62828; color: white; border-radius: 6px; }";
const char * kForceStopStyle =
  "QPushButton { background-color: #C62828; color: white; border-radius: 6px; }"
  "QPushButton:disabled { background-color: #B0BEC5; color: #37474F; }";
}  // namespace

MLPlannerStartPanel::MLPlannerStartPanel(QWidget * parent)
: rviz_common::Panel(parent), alive_(std::make_shared<std::atomic<bool>>(true))
{
  auto * layout = new QVBoxLayout(this);

  start_button_ = new QPushButton("Start ML planner");
  start_button_->setMinimumHeight(64);
  start_button_->setFont(QFont("Sans", 11, QFont::Bold));
  connect(start_button_, &QPushButton::clicked, this, &MLPlannerStartPanel::onStartClicked);
  layout->addWidget(start_button_);

  force_stop_button_ = new QPushButton("Force stop");
  force_stop_button_->setMinimumHeight(40);
  force_stop_button_->setFont(QFont("Sans", 10, QFont::Bold));
  force_stop_button_->setToolTip(
    "Turn the ego-velocity override off immediately, even if the vehicle has not moved.");
  connect(
    force_stop_button_, &QPushButton::clicked, this, &MLPlannerStartPanel::onForceStopClicked);
  layout->addWidget(force_stop_button_);

  status_label_ = new QLabel("Ready. Press Start to enable the standstill velocity override.");
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);

  speed_label_ = new QLabel("Speed: —");
  layout->addWidget(speed_label_);

  setLayout(layout);
  refreshUi();

  auto * speed_timer = new QTimer(this);
  connect(speed_timer, &QTimer::timeout, this, [this]() {
    if (!speed_received_.load(std::memory_order_relaxed)) {
      speed_label_->setText("Speed: —");
      return;
    }
    speed_label_->setText(
      QString("Speed: %1 m/s").arg(speed_mps_.load(std::memory_order_relaxed), 0, 'f', 2));
  });
  speed_timer->start(200);
}

MLPlannerStartPanel::~MLPlannerStartPanel()
{
  alive_->store(false);
}

void MLPlannerStartPanel::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
  const auto node_abstraction = rviz_ros_node_.lock();
  if (!node_abstraction) {
    {
      std::lock_guard<std::mutex> lock(phase_mutex_);
      phase_ = Phase::Error;
      status_message_ = "RViz ROS node is not available";
    }
    refreshUi();
    return;
  }

  raw_node_ = node_abstraction->get_raw_node();
  client_ = raw_node_->create_client<std_srvs::srv::SetBool>(kServiceName);
  sub_kinematic_ = raw_node_->create_subscription<nav_msgs::msg::Odometry>(
    kKinematicTopic, rclcpp::QoS(10),
    std::bind(&MLPlannerStartPanel::onKinematic, this, std::placeholders::_1));
}

void MLPlannerStartPanel::onStartClicked()
{
  bool send = false;
  {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    if (phase_ == Phase::Starting || phase_ == Phase::Working || phase_ == Phase::Stopping) {
      return;
    }
    if (!client_ || !client_->service_is_ready()) {
      phase_ = Phase::Error;
      status_message_ = "Start service is not available";
    } else {
      phase_ = Phase::Starting;
      stop_reason_ = StopReason::None;
      status_message_ = "Calling start (data: true)…";
      send = true;
    }
  }
  refreshUi();
  if (send) {
    sendSetBool(true);
  }
}

void MLPlannerStartPanel::onForceStopClicked()
{
  if (!client_ || !client_->service_is_ready()) {
    {
      std::lock_guard<std::mutex> lock(phase_mutex_);
      status_message_ = "Start service is not available";
    }
    refreshUi();
    return;
  }
  if (!claimStop(StopReason::Manual)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    status_message_ = "Force stop: calling data false…";
  }
  refreshUi();
  sendSetBool(false);
}

void MLPlannerStartPanel::onKinematic(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const double speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  speed_mps_.store(speed, std::memory_order_relaxed);
  speed_received_.store(true, std::memory_order_relaxed);
  if (speed > kDepartureSpeedMps) {
    tryDepartureStop();
  }
}

bool MLPlannerStartPanel::claimStop(StopReason reason)
{
  std::lock_guard<std::mutex> lock(phase_mutex_);
  if (phase_ != Phase::Working) {
    return false;
  }
  phase_ = Phase::Stopping;
  stop_reason_ = reason;
  return true;
}

void MLPlannerStartPanel::tryDepartureStop()
{
  if (!client_ || !client_->service_is_ready()) {
    return;
  }
  if (!claimStop(StopReason::Departure)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    status_message_ = "Speed exceeded 0.2 m/s. Calling data false…";
  }
  const auto alive = alive_;
  QMetaObject::invokeMethod(
    this,
    [this, alive]() {
      if (!alive->load()) {
        return;
      }
      refreshUi();
    },
    Qt::QueuedConnection);
  sendSetBool(false);
}

void MLPlannerStartPanel::sendSetBool(bool data)
{
  if (!client_ || !raw_node_) {
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = data;
  RCLCPP_INFO(
    raw_node_->get_logger(), "ML planner start panel: calling %s data=%s", kServiceName,
    data ? "true" : "false");

  const auto alive = alive_;
  pending_request_.emplace(client_->async_send_request(
    request, [this, alive, data](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      if (!alive->load()) {
        return;
      }
      bool success = false;
      std::string message;
      try {
        const auto response = future.get();
        success = static_cast<bool>(response->success);
        message = response->message;
      } catch (const std::exception & exception) {
        message = exception.what();
      }
      QMetaObject::invokeMethod(
        this,
        [this, alive, data, success, message]() {
          if (!alive->load()) {
            return;
          }
          onServiceDone(data, success, message);
        },
        Qt::QueuedConnection);
    }));
}

void MLPlannerStartPanel::onServiceDone(bool data, bool success, const std::string & message)
{
  bool departure_already_true = false;
  {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    if (data) {
      if (phase_ != Phase::Starting) {
        return;
      }
      if (success) {
        phase_ = Phase::Working;
        status_message_ =
          message.empty() ? "Start succeeded. Ego velocity override is on." : message;
        departure_already_true = speed_received_.load(std::memory_order_relaxed) &&
                                 speed_mps_.load(std::memory_order_relaxed) > kDepartureSpeedMps;
      } else {
        phase_ = Phase::Error;
        status_message_ = message.empty() ? "Start service returned failure" : message;
      }
    } else {
      if (phase_ != Phase::Stopping) {
        return;
      }
      if (success) {
        phase_ = Phase::Done;
        status_message_ = stop_reason_ == StopReason::Manual
                            ? "Force stop succeeded. Ego velocity override is off."
                            : "Vehicle moved above 0.2 m/s. Ego velocity override is off.";
        if (!message.empty()) {
          status_message_ += " (" + message + ")";
        }
      } else {
        // The override is still on. Stay in Working so Force stop and the speed trigger can retry.
        phase_ = Phase::Working;
        status_message_ =
          message.empty() ? "Stop service returned failure. Override is still on." : message;
      }
    }
  }

  if (raw_node_) {
    RCLCPP_INFO(
      raw_node_->get_logger(), "ML planner start panel: data=%s success=%s message='%s'",
      data ? "true" : "false", success ? "true" : "false", message.c_str());
  }
  refreshUi();
  if (departure_already_true) {
    tryDepartureStop();
  }
}

void MLPlannerStartPanel::refreshUi()
{
  Phase phase = Phase::Idle;
  StopReason stop_reason = StopReason::None;
  std::string message;
  {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    phase = phase_;
    stop_reason = stop_reason_;
    message = status_message_;
  }

  force_stop_button_->setEnabled(phase == Phase::Working);
  force_stop_button_->setStyleSheet(kForceStopStyle);

  switch (phase) {
    case Phase::Idle:
      start_button_->setEnabled(true);
      start_button_->setText("Start ML planner");
      start_button_->setStyleSheet(kButtonStyleIdle);
      if (message.empty()) {
        status_label_->setText("Ready. Press Start to enable the standstill velocity override.");
      } else {
        status_label_->setText(QString::fromStdString(message));
      }
      break;
    case Phase::Starting:
      start_button_->setEnabled(false);
      start_button_->setText("Calling start…");
      start_button_->setStyleSheet(kButtonStyleBusy);
      status_label_->setText(QString::fromStdString(message));
      break;
    case Phase::Working:
      start_button_->setEnabled(false);
      start_button_->setText("WORKING");
      start_button_->setStyleSheet(kButtonStyleWorking);
      status_label_->setText(
        QString("WORKING — %1\nWaiting for speed > 0.2 m/s, or press Force stop.")
          .arg(QString::fromStdString(message)));
      break;
    case Phase::Stopping:
      start_button_->setEnabled(false);
      start_button_->setText(stop_reason == StopReason::Manual ? "Force stopping…" : "Stopping…");
      start_button_->setStyleSheet(kButtonStyleBusy);
      status_label_->setText(QString::fromStdString(message));
      break;
    case Phase::Done:
      start_button_->setEnabled(true);
      start_button_->setText("Start again");
      start_button_->setStyleSheet(kButtonStyleDone);
      status_label_->setText(QString("DONE — %1").arg(QString::fromStdString(message)));
      break;
    case Phase::Error:
      start_button_->setEnabled(true);
      start_button_->setText("Retry start");
      start_button_->setStyleSheet(kButtonStyleError);
      status_label_->setText(QString("ERROR — %1").arg(QString::fromStdString(message)));
      break;
  }
}

}  // namespace autoware::ml_planner_start_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::ml_planner_start_rviz_plugin::MLPlannerStartPanel, rviz_common::Panel)
