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

#include <rclcpp/rclcpp.hpp>

#include <ctime>
#include <filesystem>
#include <iostream>

void setFormatDate(QLabel * line, double time)
{
  char buffer[128];
  time_t seconds = static_cast<time_t>(time);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H-%M-%S", localtime(&seconds));
  line->setText(QString(buffer));
}

AutowareScreenCapturePanel::AutowareScreenCapturePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * v_layout = new QVBoxLayout;

  // screen capture
  auto * cap_layout = new QHBoxLayout;
  {
    ros_time_label_ = new QLabel;
    screen_capture_button_ptr_ = new QPushButton("Capture Screen Shot");
    connect(screen_capture_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickScreenCapture()));
    cap_layout->addWidget(screen_capture_button_ptr_);
    cap_layout->addWidget(ros_time_label_);
    cap_layout->addWidget(new QLabel(" [Key] "));
  }

  // video capture
  auto * video_cap_layout = new QHBoxLayout;
  {
    capture_to_mp4_button_ptr_ = new QPushButton("Capture To Video");
    connect(capture_to_mp4_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickVideoCapture()));
    capture_hz_ = new QSpinBox();
    capture_hz_->setRange(1, 2);
    capture_hz_->setValue(1);
    capture_hz_->setSingleStep(1);
    connect(capture_hz_, SIGNAL(valueChanged(int)), this, SLOT(onRateChanged(int)));
    // video cap layout
    video_cap_layout->addWidget(capture_to_mp4_button_ptr_);
    video_cap_layout->addWidget(capture_hz_);
    video_cap_layout->addWidget(new QLabel(" [Hz]"));
  }

  // consider layout
  {
    v_layout->addLayout(cap_layout);
    v_layout->addLayout(video_cap_layout);
    setLayout(v_layout);
  }
  QTimer * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &AutowareScreenCapturePanel::update);
  timer->start(1000);
  capture_timer_ = new QTimer(this);
  connect(capture_timer_, &QTimer::timeout, this, &AutowareScreenCapturePanel::onTimer);
  state_ = State::WAITING_FOR_CAPTURE;
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

void convertToVideo() {}

void AutowareScreenCapturePanel::onClickVideoCapture()
{
  const int clock = static_cast<int>(1e3 / capture_hz_->value());
  switch (state_) {
    case State::WAITING_FOR_CAPTURE:
      // initialize setting
      {
        counter_ = 0;
        root_folder_ = ros_time_label_->text().toStdString();
        std::filesystem::create_directory(root_folder_);
      }
      capture_to_mp4_button_ptr_->setText("capturing rviz screen");
      capture_to_mp4_button_ptr_->setStyleSheet("background-color: #FF0000;");
      capture_timer_->start(clock);
      state_ = State::CAPTURING;
      break;
    case State::CAPTURING: {
      capture_timer_->stop();
    }
      capture_to_mp4_button_ptr_->setText("writing to video");
      capture_to_mp4_button_ptr_->setStyleSheet("background-color: #FFFF00;");
      convertToVideo();
      state_ = State::FINALIZED;
      break;
    case State::FINALIZED:
      capture_to_mp4_button_ptr_->setText("waiting for capture");
      capture_to_mp4_button_ptr_->setStyleSheet("background-color: #00FF00;");
      state_ = State::WAITING_FOR_CAPTURE;
      break;
  }
  return;
}

void AutowareScreenCapturePanel::onTimer()
{
  std::stringstream count_text;
  count_text << std::setw(6) << std::setfill('0') << counter_;
  const std::string file = root_folder_ + "/" + count_text.str() + ".png";
  std::cerr << "file" << file << std::endl;
  getDisplayContext()->getViewManager()->getRenderPanel()->getRenderWindow()->captureScreenShot(
    file);
  counter_++;
}

void AutowareScreenCapturePanel::update()
{
  setFormatDate(
    ros_time_label_, rviz_ros_node_.lock()->get_raw_node()->get_clock()->now().seconds());
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(AutowareScreenCapturePanel, rviz_common::Panel)
