// Copyright 2021 Tier IV, Inc.
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

#include "screen_capture_panel.hpp"

#include <QHBoxLayout>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>

#include <ctime>
#include <iostream>

void setFormatTime(QLineEdit * line, double time)
{
  char buffer[128];
  time_t seconds = static_cast<time_t>(time);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H:%M:%S", localtime(&seconds));
  line->setText(QString(buffer) + QString::number((time - seconds), 'f', 2).rightRef(3));
}

AutowareScreenCapturePanel::AutowareScreenCapturePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  ros_time_label_ = new QLineEdit;
  ros_time_label_->setReadOnly(true);

  QHBoxLayout * layout = new QHBoxLayout(this);
  screen_capture_button_ptr_ = new QPushButton("Capture Screen Shot");

  connect(screen_capture_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickScreenCapture()));
  layout->addWidget(new QLabel("ROS Time:"));
  layout->addWidget(ros_time_label_);
  layout->addWidget(screen_capture_button_ptr_);
  setLayout(layout);

  QTimer * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &AutowareScreenCapturePanel::update);
  timer->start(60);
}

void AutowareScreenCapturePanel::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
}

void AutowareScreenCapturePanel::onClickScreenCapture()
{
  const std::string time_text = ros_time_label_->text().toStdString();
  getDisplayContext()->getViewManager()->getRenderPanel()->getRenderWindow()->captureScreenShot(
    time_text + ".png");
  return;
}

void AutowareScreenCapturePanel::update()
{
  setFormatTime(
    ros_time_label_, rviz_ros_node_.lock()->get_raw_node()->get_clock()->now().seconds());
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(AutowareScreenCapturePanel, rviz_common::Panel)
